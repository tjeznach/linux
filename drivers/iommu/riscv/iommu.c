// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V IOMMU implementations.
 *
 * Copyright © 2022-2024 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "iommu-bits.h"
#include "iommu.h"

MODULE_DESCRIPTION("Driver for RISC-V IOMMU");
MODULE_AUTHOR("Tomasz Jeznach <tjeznach@rivosinc.com>");
MODULE_AUTHOR("Nick Kossifidis <mick@ics.forth.gr>");
MODULE_LICENSE("GPL");

/* Timeouts in [us] */
#define RISCV_IOMMU_QCSR_TIMEOUT	150000
#define RISCV_IOMMU_QUEUE_TIMEOUT	150000
#define RISCV_IOMMU_DDTP_TIMEOUT	10000000
#define RISCV_IOMMU_IOTINVAL_TIMEOUT	90000000

/* Number of entries per CMD/FLT queue, should be <= INT_MAX */
#define RISCV_IOMMU_DEF_CQ_COUNT	8192
#define RISCV_IOMMU_DEF_FQ_COUNT	4096

/* RISC-V IOMMU PPN <> PHYS address conversions, PHYS <=> PPN[53:10] */
#define phys_to_ppn(va)  (((va) >> 2) & (((1ULL << 44) - 1) << 10))
#define ppn_to_phys(pn)	 (((pn) << 2) & (((1ULL << 44) - 1) << 12))

#define dev_to_iommu(dev) \
	container_of((dev)->iommu->iommu_dev, struct riscv_iommu_device, iommu)

/* IOMMU PSCID allocation namespace. */
static DEFINE_IDA(riscv_iommu_pscids);
#define RISCV_IOMMU_MAX_PSCID		BIT(20)

/* Device resource-managed allocations */
struct riscv_iommu_devres {
	unsigned long addr;
	unsigned int order;
};

static void riscv_iommu_devres_pages_release(struct device *dev, void *res)
{
	struct riscv_iommu_devres *devres = res;

	free_pages(devres->addr, devres->order);
}

static int riscv_iommu_devres_pages_match(struct device *dev, void *res, void *p)
{
	struct riscv_iommu_devres *devres = res;
	struct riscv_iommu_devres *target = p;

	return devres->addr == target->addr;
}

static unsigned long riscv_iommu_get_pages(struct riscv_iommu_device *iommu, unsigned int order)
{
	struct riscv_iommu_devres *devres;
	struct page *pages;

	pages = alloc_pages_node(dev_to_node(iommu->dev),
				 GFP_KERNEL_ACCOUNT | __GFP_ZERO, order);
	if (unlikely(!pages)) {
		dev_err(iommu->dev, "Page allocation failed, order %u\n", order);
		return 0;
	}

	devres = devres_alloc(riscv_iommu_devres_pages_release,
			      sizeof(struct riscv_iommu_devres), GFP_KERNEL);

	if (unlikely(!devres)) {
		__free_pages(pages, order);
		return 0;
	}

	devres->addr = (unsigned long)page_address(pages);
	devres->order = order;

	devres_add(iommu->dev, devres);

	return devres->addr;
}

static void riscv_iommu_free_pages(struct riscv_iommu_device *iommu, unsigned long addr)
{
	struct riscv_iommu_devres devres = { .addr = addr };

	devres_release(iommu->dev, riscv_iommu_devres_pages_release,
		       riscv_iommu_devres_pages_match, &devres);
}

/*
 * Hardware queue allocation and management.
 */

/* Setup queue base, control registers and default queue length */
#define RISCV_IOMMU_QUEUE_INIT(q, name) do {					\
	struct riscv_iommu_queue *_q = q;					\
	_q->qid = RISCV_IOMMU_INTR_ ## name;					\
	_q->qbr = RISCV_IOMMU_REG_ ## name ## B;				\
	_q->qcr = RISCV_IOMMU_REG_ ## name ## CSR;				\
	_q->mask = _q->mask ?: (RISCV_IOMMU_DEF_ ## name ## _COUNT) - 1;	\
} while (0)

/* Note: offsets are the same for all queues */
#define Q_HEAD(q) ((q)->qbr + (RISCV_IOMMU_REG_CQH - RISCV_IOMMU_REG_CQB))
#define Q_TAIL(q) ((q)->qbr + (RISCV_IOMMU_REG_CQT - RISCV_IOMMU_REG_CQB))
#define Q_ITEM(q, index) ((q)->mask & (index))
#define Q_IPSR(q) BIT((q)->qid)

/*
 * Discover queue ring buffer hardware configuration, allocate in-memory
 * ring buffer or use fixed I/O memory location, configure queue base register.
 * Must be called before hardware queue is enabled.
 *
 * @queue - data structure, configured with RISCV_IOMMU_QUEUE_INIT()
 * @entry_size - queue single element size in bytes.
 */
static int riscv_iommu_queue_alloc(struct riscv_iommu_device *iommu,
				   struct riscv_iommu_queue *queue,
				   size_t entry_size)
{
	unsigned int logsz;
	unsigned long addr = 0;
	u64 qb, rb;

	/*
	 * Use WARL base register property to discover maximum allowed
	 * number of entries and optional fixed IO address for queue location.
	 */
	riscv_iommu_writeq(iommu, queue->qbr, RISCV_IOMMU_QUEUE_LOGSZ_FIELD);
	qb = riscv_iommu_readq(iommu, queue->qbr);

	/*
	 * Calculate and verify hardware supported queue length, as reported
	 * by the field LOGSZ, where max queue length is equal to 2^(LOGSZ + 1).
	 * Update queue size based on hardware supported value.
	 */
	logsz = ilog2(queue->mask);
	if (logsz > FIELD_GET(RISCV_IOMMU_QUEUE_LOGSZ_FIELD, qb))
		logsz = FIELD_GET(RISCV_IOMMU_QUEUE_LOGSZ_FIELD, qb);

	/*
	 * Use WARL base register property to discover an optional fixed IO address
	 * for queue ring buffer location. Otherwise allocate contigus system memory.
	 */
	if (FIELD_GET(RISCV_IOMMU_PPN_FIELD, qb)) {
		const size_t queue_size = entry_size << (logsz + 1);

		queue->phys = ppn_to_phys(FIELD_GET(RISCV_IOMMU_PPN_FIELD, qb));
		queue->base = devm_ioremap(iommu->dev, queue->phys, queue_size);
	} else {
		do {
			const size_t queue_size = entry_size << (logsz + 1);

			addr = riscv_iommu_get_pages(iommu, (unsigned int)get_order(queue_size));
			queue->base = (u64 *)addr;
			queue->phys = __pa(addr);
		} while (!queue->base && logsz-- > 0);
	}

	if (!queue->base)
		return -ENOMEM;

	qb = phys_to_ppn(queue->phys) |
	     FIELD_PREP(RISCV_IOMMU_QUEUE_LOGSZ_FIELD, logsz);

	/* Update base register and read back to verify hw accepted our write */
	riscv_iommu_writeq(iommu, queue->qbr, qb);
	rb = riscv_iommu_readq(iommu, queue->qbr);
	if (rb != qb) {
		if (addr)
			riscv_iommu_free_pages(iommu, addr);
		return -ENODEV;
	}

	/* Update actual queue mask */
	queue->mask = (2U << logsz) - 1;

	dev_dbg(iommu->dev, "queue #%u allocated 2^%u entries", queue->qid, logsz + 1);

	return 0;
}

/* Check interrupt queue status, IPSR */
static irqreturn_t riscv_iommu_queue_ipsr(int irq, void *data)
{
	struct riscv_iommu_queue *queue = (struct riscv_iommu_queue *)data;

	if (riscv_iommu_readl(queue->iommu, RISCV_IOMMU_REG_IPSR) & Q_IPSR(queue))
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

static int riscv_iommu_queue_vec(struct riscv_iommu_device *iommu, int n)
{
	/* Reuse IVEC.CIV mask for all interrupt vectors mapping. */
	return (iommu->ivec >> (n * 4)) & RISCV_IOMMU_IVEC_CIV;
}

/*
 * Enable queue processing in the hardware, register interrupt handler.
 *
 * @queue - data structure, already allocated with riscv_iommu_queue_alloc()
 * @irq_handler - threaded interrupt handler.
 */
static int riscv_iommu_queue_enable(struct riscv_iommu_device *iommu,
				    struct riscv_iommu_queue *queue,
				    irq_handler_t irq_handler)
{
	const unsigned int irq = iommu->irqs[riscv_iommu_queue_vec(iommu, queue->qid)];
	u32 csr;
	int rc;

	if (queue->iommu)
		return -EBUSY;

	/* Polling not implemented */
	if (!irq)
		return -ENODEV;

	queue->iommu = iommu;
	rc = request_threaded_irq(irq, riscv_iommu_queue_ipsr, irq_handler,
				  IRQF_ONESHOT | IRQF_SHARED, dev_name(iommu->dev), queue);
	if (rc) {
		queue->iommu = NULL;
		return rc;
	}

	/*
	 * Enable queue with interrupts, clear any memory fault if any.
	 * Wait for the hardware to acknowledge request and activate queue processing.
	 * Note: All CSR bitfields are in the same offsets for all queues.
	 */
	riscv_iommu_writel(iommu, queue->qcr,
			   RISCV_IOMMU_QUEUE_ENABLE |
			   RISCV_IOMMU_QUEUE_INTR_ENABLE |
			   RISCV_IOMMU_QUEUE_MEM_FAULT);

	riscv_iommu_readl_timeout(iommu, queue->qcr,
				  csr, !(csr & RISCV_IOMMU_QUEUE_BUSY),
				  10, RISCV_IOMMU_QCSR_TIMEOUT);

	if (RISCV_IOMMU_QUEUE_ACTIVE != (csr & (RISCV_IOMMU_QUEUE_ACTIVE |
						RISCV_IOMMU_QUEUE_BUSY |
						RISCV_IOMMU_QUEUE_MEM_FAULT))) {
		/* Best effort to stop and disable failing hardware queue. */
		riscv_iommu_writel(iommu, queue->qcr, 0);
		free_irq(irq, queue);
		queue->iommu = NULL;
		return -EBUSY;
	}

	/* Clear any pending interrupt flag. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, Q_IPSR(queue));

	return 0;
}

/*
 * Disable queue. Wait for the hardware to acknowledge request and
 * stop processing enqueued requests. Report errors but continue.
 */
static void riscv_iommu_queue_disable(struct riscv_iommu_queue *queue)
{
	struct riscv_iommu_device *iommu = queue->iommu;
	u32 csr;

	if (!iommu)
		return;

	free_irq(iommu->irqs[riscv_iommu_queue_vec(iommu, queue->qid)], queue);
	riscv_iommu_writel(iommu, queue->qcr, 0);
	riscv_iommu_readl_timeout(iommu, queue->qcr,
				  csr, !(csr & RISCV_IOMMU_QUEUE_BUSY),
				  10, RISCV_IOMMU_QCSR_TIMEOUT);

	if (csr & (RISCV_IOMMU_QUEUE_ACTIVE | RISCV_IOMMU_QUEUE_BUSY))
		dev_err(iommu->dev, "fail to disable hardware queue #%u, csr 0x%x\n",
			queue->qid, csr);

	queue->iommu = NULL;
}

/*
 * Returns number of available valid queue entries and the first item index or negative
 * error code.  Update shadow producer index if necessary.
 */
static int riscv_iommu_queue_consume(struct riscv_iommu_queue *queue, unsigned int *index)
{
	unsigned int head = atomic_read(&queue->head);
	unsigned int tail = atomic_read(&queue->tail);
	unsigned int last = Q_ITEM(queue, tail);
	int available = (int)(tail - head);

	*index = head;

	if (available > 0)
		return available;

	/* read hardware producer index, check reserved register bits are not set. */
	if (riscv_iommu_readl_timeout(queue->iommu, Q_TAIL(queue),
				      tail, (tail & ~queue->mask) == 0,
				      0, RISCV_IOMMU_QUEUE_TIMEOUT))
		return -EBUSY;

	if (tail == last)
		return 0;

	/* update shadow producer index */
	return (int)(atomic_add_return((tail - last) & queue->mask, &queue->tail) - head);
}

/*
 * Release processed queue entries, should match riscv_iommu_queue_consume() calls.
 */
static void riscv_iommu_queue_release(struct riscv_iommu_queue *queue, int count)
{
	const unsigned int head = atomic_add_return(count, &queue->head);

	riscv_iommu_writel(queue->iommu, Q_HEAD(queue), Q_ITEM(queue, head));
}

/* Return actual consumer index based on hardware reported queue head index. */
static unsigned int riscv_iommu_queue_cons(struct riscv_iommu_queue *queue)
{
	const unsigned int cons = atomic_read(&queue->head);
	const unsigned int last = Q_ITEM(queue, cons);
	unsigned int head;

	if (riscv_iommu_readl_timeout(queue->iommu, Q_HEAD(queue), head,
				      !(head & ~queue->mask), 0, RISCV_IOMMU_QUEUE_TIMEOUT))
		return cons;

	return cons + ((head - last) & queue->mask);
}

/* Wait for submitted item to be processed. */
static int riscv_iommu_queue_wait(struct riscv_iommu_queue *queue, unsigned int index,
				  unsigned int timeout_us)
{
	unsigned int cons = atomic_read(&queue->head);

	/* Already processed by the consumer */
	if ((int)(cons - index) > 0)
		return 0;

	/* Monitor consumer index */
	return readx_poll_timeout(riscv_iommu_queue_cons, queue, cons, (int)(cons - index) > 0,
				  0, timeout_us);
}

/* Enqueue an entry and wait to be processed if timeout_us > 0 */
static int riscv_iommu_queue_send(struct riscv_iommu_queue *queue,
				  void *entry, size_t entry_size,
				  unsigned int timeout_us)
{
	unsigned int prod;
	unsigned int head;
	unsigned int tail;
	unsigned long flags;

	/* Do not preempt submission flow. */
	local_irq_save(flags);

	/* 1. Allocate some space in the queue */
	prod = atomic_inc_return(&queue->prod) - 1;
	head = atomic_read(&queue->head);

	/* 2. Wait for space availability. */
	if ((prod - head) > queue->mask) {
		if (readx_poll_timeout(atomic_read, &queue->head,
				       head, (prod - head) < queue->mask,
				       0, RISCV_IOMMU_QUEUE_TIMEOUT))
			goto err_busy;
	} else if ((prod - head) == queue->mask) {
		const unsigned int last = Q_ITEM(queue, head);

		if (riscv_iommu_readl_timeout(queue->iommu, Q_HEAD(queue), head,
					      !(head & ~queue->mask) && head != last,
					      0, RISCV_IOMMU_QUEUE_TIMEOUT))
			goto err_busy;
		atomic_add((head - last) & queue->mask, &queue->head);
	}

	/* 3. Store entry in the ring buffer. */
	memcpy(queue->base + Q_ITEM(queue, prod) * entry_size, entry, entry_size);

	/* 4. Wait for all previous entries to be ready */
	if (readx_poll_timeout(atomic_read, &queue->tail, tail, prod == tail,
			       0, RISCV_IOMMU_QUEUE_TIMEOUT))
		goto err_busy;

	/* 5. Complete submission and restore local interrupts */
	dma_wmb();
	riscv_iommu_writel(queue->iommu, Q_TAIL(queue), Q_ITEM(queue, prod + 1));
	atomic_inc(&queue->tail);
	local_irq_restore(flags);

	if (timeout_us)
		return WARN_ON(riscv_iommu_queue_wait(queue, prod, timeout_us));

	return 0;

err_busy:
	local_irq_restore(flags);
	return -EBUSY;
}

/*
 * IOMMU Command queue chapter 3.1
 */

/* Command queue interrupt handler thread function */
static irqreturn_t riscv_iommu_cmdq_process(int irq, void *data)
{
	const struct riscv_iommu_queue *queue = (struct riscv_iommu_queue *)data;
	unsigned int ctrl;

	/* Clear MF/CQ errors, complete error recovery to be implemented. */
	ctrl = riscv_iommu_readl(queue->iommu, queue->qcr);
	if (ctrl & (RISCV_IOMMU_CQCSR_CQMF | RISCV_IOMMU_CQCSR_CMD_TO |
		    RISCV_IOMMU_CQCSR_CMD_ILL | RISCV_IOMMU_CQCSR_FENCE_W_IP)) {
		riscv_iommu_writel(queue->iommu, queue->qcr, ctrl);
		dev_warn(queue->iommu->dev,
			 "Queue #%u error; fault:%d timeout:%d illegal:%d fence_w_ip:%d\n",
			 queue->qid,
			 !!(ctrl & RISCV_IOMMU_CQCSR_CQMF),
			 !!(ctrl & RISCV_IOMMU_CQCSR_CMD_TO),
			 !!(ctrl & RISCV_IOMMU_CQCSR_CMD_ILL),
			 !!(ctrl & RISCV_IOMMU_CQCSR_FENCE_W_IP));
	}

	/* Placeholder for command queue interrupt notifiers */

	/* Clear command interrupt pending. */
	riscv_iommu_writel(queue->iommu, RISCV_IOMMU_REG_IPSR, Q_IPSR(queue));

	return IRQ_HANDLED;
}

/* Send command to the IOMMU command queue */
static int riscv_iommu_cmd_send(struct riscv_iommu_device *iommu,
				struct riscv_iommu_command *cmd,
				unsigned int timeout_us)
{
	return riscv_iommu_queue_send(&iommu->cmdq, cmd, sizeof(*cmd), timeout_us);
}

/*
 * IOMMU Fault/Event queue chapter 3.2
 */

static void riscv_iommu_fault(struct riscv_iommu_device *iommu,
			      struct riscv_iommu_fq_record *event)
{
	unsigned int err = FIELD_GET(RISCV_IOMMU_FQ_HDR_CAUSE, event->hdr);
	unsigned int devid = FIELD_GET(RISCV_IOMMU_FQ_HDR_DID, event->hdr);

	/* Placeholder for future fault handling implementation, report only. */
	if (err)
		dev_warn_ratelimited(iommu->dev,
				     "Fault %d devid: 0x%x iotval: %llx iotval2: %llx\n",
				     err, devid, event->iotval, event->iotval2);
}

/* Fault queue interrupt handler thread function */
static irqreturn_t riscv_iommu_fltq_process(int irq, void *data)
{
	struct riscv_iommu_queue *queue = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu = queue->iommu;
	struct riscv_iommu_fq_record *events;
	unsigned int ctrl, idx;
	int cnt, len;

	events = (struct riscv_iommu_fq_record *)queue->base;

	/* Clear fault interrupt pending and process all received fault events. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, Q_IPSR(queue));

	do {
		cnt = riscv_iommu_queue_consume(queue, &idx);
		for (len = 0; len < cnt; idx++, len++)
			riscv_iommu_fault(iommu, &events[Q_ITEM(queue, idx)]);
		riscv_iommu_queue_release(queue, cnt);
	} while (cnt > 0);

	/* Clear MF/OF errors, complete error recovery to be implemented. */
	ctrl = riscv_iommu_readl(iommu, queue->qcr);
	if (ctrl & (RISCV_IOMMU_FQCSR_FQMF | RISCV_IOMMU_FQCSR_FQOF)) {
		riscv_iommu_writel(iommu, queue->qcr, ctrl);
		dev_warn(iommu->dev,
			 "Queue #%u error; memory fault:%d overflow:%d\n",
			 queue->qid,
			 !!(ctrl & RISCV_IOMMU_FQCSR_FQMF),
			 !!(ctrl & RISCV_IOMMU_FQCSR_FQOF));
	}

	return IRQ_HANDLED;
}

/* Lookup and initialize device context info structure. */
static struct riscv_iommu_dc *riscv_iommu_get_dc(struct riscv_iommu_device *iommu,
						 unsigned int devid, bool fetch)
{
	const bool base_format = !(iommu->caps & RISCV_IOMMU_CAP_MSI_FLAT);
	unsigned int depth;
	unsigned long ddt, ptr, old, new;
	u8 ddi_bits[3] = { 0 };
	u64 *ddtp = NULL;

	/* Make sure the mode is valid */
	if (iommu->ddt_mode < RISCV_IOMMU_DDTP_MODE_1LVL ||
	    iommu->ddt_mode > RISCV_IOMMU_DDTP_MODE_3LVL)
		return NULL;

	/*
	 * Device id partitioning for base format:
	 * DDI[0]: bits 0 - 6   (1st level) (7 bits)
	 * DDI[1]: bits 7 - 15  (2nd level) (9 bits)
	 * DDI[2]: bits 16 - 23 (3rd level) (8 bits)
	 *
	 * For extended format:
	 * DDI[0]: bits 0 - 5   (1st level) (6 bits)
	 * DDI[1]: bits 6 - 14  (2nd level) (9 bits)
	 * DDI[2]: bits 15 - 23 (3rd level) (9 bits)
	 */
	if (base_format) {
		ddi_bits[0] = 7;
		ddi_bits[1] = 7 + 9;
		ddi_bits[2] = 7 + 9 + 8;
	} else {
		ddi_bits[0] = 6;
		ddi_bits[1] = 6 + 9;
		ddi_bits[2] = 6 + 9 + 9;
	}

	/* Make sure device id is within range */
	depth = iommu->ddt_mode - RISCV_IOMMU_DDTP_MODE_1LVL;
	if (devid >= (1 << ddi_bits[depth]))
		return NULL;

	/* Get to the level of the non-leaf node that holds the device context */
	for (ddtp = iommu->ddt_root; depth-- > 0;) {
		const int split = ddi_bits[depth];
		/*
		 * Each non-leaf node is 64bits wide and on each level
		 * nodes are indexed by DDI[depth].
		 */
		ddtp += (devid >> split) & 0x1FF;

		/*
		 * Check if this node has been populated and if not
		 * allocate a new level and populate it.
		 */
		do {
			ddt = READ_ONCE(*(unsigned long *)ddtp);
			if (ddt & RISCV_IOMMU_DDTE_VALID) {
				ddtp = __va(ppn_to_phys(ddt));
				break;
			}

			/* Fetch only, do not allocate new device context. */
			if (fetch)
				return NULL;

			ptr = riscv_iommu_get_pages(iommu, 0);
			if (!ptr)
				return NULL;

			new = phys_to_ppn(__pa(ptr)) | RISCV_IOMMU_DDTE_VALID;
			old = cmpxchg_relaxed((unsigned long *)ddtp, ddt, new);

			if (old == ddt) {
				ddtp = (u64 *)ptr;
				break;
			}

			/* Race setting DDT detected, re-read and retry. */
			riscv_iommu_free_pages(iommu, ptr);
		} while (1);
	}

	/*
	 * Grab the node that matches DDI[depth], note that when using base
	 * format the device context is 4 * 64bits, and the extended format
	 * is 8 * 64bits, hence the (3 - base_format) below.
	 */
	ddtp += (devid & ((64 << base_format) - 1)) << (3 - base_format);

	return (struct riscv_iommu_dc *)ddtp;
}

/*
 * Discover supported DDT modes starting from requested value,
 * configure DDTP register with accepted mode and root DDT address.
 * Accepted iommu->ddt_mode is updated on success.
 */
static int riscv_iommu_set_ddtp_mode(struct riscv_iommu_device *iommu,
				     unsigned int ddtp_mode)
{
	struct device *dev = iommu->dev;
	struct riscv_iommu_command cmd;
	u64 ddtp, rq_ddtp;
	unsigned int mode, rq_mode = ddtp_mode;
	int rc;

	rc = readq_relaxed_poll_timeout(iommu->reg + RISCV_IOMMU_REG_DDTP,
					ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
					10, RISCV_IOMMU_DDTP_TIMEOUT);
	if (rc < 0)
		return -EBUSY;

	/* Disallow state transition from xLVL to xLVL. */
	switch (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp)) {
	case RISCV_IOMMU_DDTP_MODE_BARE:
	case RISCV_IOMMU_DDTP_MODE_OFF:
		break;
	default:
		if (rq_mode != RISCV_IOMMU_DDTP_MODE_BARE &&
		    rq_mode != RISCV_IOMMU_DDTP_MODE_OFF)
			return -EINVAL;
		break;
	}

	do {
		rq_ddtp = FIELD_PREP(RISCV_IOMMU_DDTP_MODE, rq_mode);
		if (rq_mode > RISCV_IOMMU_DDTP_MODE_BARE)
			rq_ddtp |= phys_to_ppn(iommu->ddt_phys);

		riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP, rq_ddtp);

		rc = readq_relaxed_poll_timeout(iommu->reg + RISCV_IOMMU_REG_DDTP,
						ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
						10, RISCV_IOMMU_DDTP_TIMEOUT);
		if (rc < 0) {
			dev_warn(dev, "timeout when setting ddtp (ddt mode: %u, read: %llx)\n",
				 rq_mode, ddtp);
			return -EBUSY;
		}

		/* Verify IOMMU hardware accepts new DDTP config. */
		mode = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);

		if (rq_mode == mode)
			break;

		/* Hardware mandatory DDTP mode has not been accepted. */
		if (rq_mode < RISCV_IOMMU_DDTP_MODE_1LVL && rq_ddtp != ddtp) {
			dev_warn(dev, "DDTP update failed hw: %llx vs %llx\n", ddtp, rq_ddtp);
			return -EINVAL;
		}

		/*
		 * Mode field is WARL, an IOMMU may support a subset of
		 * directory table levels in which case if we tried to set
		 * an unsupported number of levels we'll readback either
		 * a valid xLVL or off/bare. If we got off/bare, try again
		 * with a smaller xLVL.
		 */
		if (mode < RISCV_IOMMU_DDTP_MODE_1LVL &&
		    rq_mode > RISCV_IOMMU_DDTP_MODE_1LVL) {
			dev_dbg(dev, "DDTP hw mode %u vs %u\n", mode, rq_mode);
			rq_mode--;
			continue;
		}

		/*
		 * We tried all supported modes and IOMMU hardware failed to
		 * accept new settings, something went very wrong since off/bare
		 * and at least one xLVL must be supported.
		 */
		dev_warn(dev, "DDTP hw mode %u, failed to set %u\n", mode, ddtp_mode);
		return -EINVAL;
	} while (1);

	iommu->ddt_mode = mode;
	if (mode != ddtp_mode)
		dev_warn(dev, "DDTP failover to %u mode, requested %u\n",
			 mode, ddtp_mode);

	/* Invalidate device context cache */
	riscv_iommu_cmd_iodir_inval_ddt(&cmd);
	riscv_iommu_cmd_send(iommu, &cmd, 0);

	/* Invalidate address translation cache */
	riscv_iommu_cmd_inval_vma(&cmd);
	riscv_iommu_cmd_send(iommu, &cmd, 0);

	/* IOFENCE.C */
	riscv_iommu_cmd_iofence(&cmd);
	return riscv_iommu_cmd_send(iommu, &cmd, RISCV_IOMMU_IOTINVAL_TIMEOUT);
}

static int riscv_iommu_ddt_alloc(struct riscv_iommu_device *iommu)
{
	u64 ddtp;
	unsigned int mode;

	riscv_iommu_readq_timeout(iommu, RISCV_IOMMU_REG_DDTP,
				  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
				  10, RISCV_IOMMU_DDTP_TIMEOUT);

	if (ddtp & RISCV_IOMMU_DDTP_BUSY)
		return -EBUSY;

	/*
	 * It is optional for the hardware to report a fixed address for device
	 * directory root page when DDT.MODE is OFF or BARE.
	 */
	mode = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);
	if (mode != RISCV_IOMMU_DDTP_MODE_BARE && mode != RISCV_IOMMU_DDTP_MODE_OFF) {
		/* Use WARL to discover hardware fixed DDT PPN */
		riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP,
				   FIELD_PREP(RISCV_IOMMU_DDTP_MODE, mode));
		riscv_iommu_readl_timeout(iommu, RISCV_IOMMU_REG_DDTP,
					  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
					  10, RISCV_IOMMU_DDTP_TIMEOUT);
		if (ddtp & RISCV_IOMMU_DDTP_BUSY)
			return -EBUSY;

		iommu->ddt_phys = ppn_to_phys(ddtp);
		if (iommu->ddt_phys)
			iommu->ddt_root = devm_ioremap(iommu->dev, iommu->ddt_phys, PAGE_SIZE);
		if (iommu->ddt_root)
			memset(iommu->ddt_root, 0, PAGE_SIZE);
	}

	if (!iommu->ddt_root) {
		iommu->ddt_root = (u64 *)riscv_iommu_get_pages(iommu, 0);
		iommu->ddt_phys = __pa(iommu->ddt_root);
	}

	if (!iommu->ddt_root)
		return -ENOMEM;

	return 0;
}

struct riscv_iommu_bond {
	struct list_head list;
	struct rcu_head rcu;
	struct device *dev;
};

/* This struct contains protection domain specific IOMMU driver data. */
struct riscv_iommu_domain {
	struct iommu_domain domain;
	struct list_head bonds;
	int pscid;
	int numa_node;
	int amo_enabled:1;
	unsigned int pgd_mode;
	/* paging domain */
	unsigned long pgd_root;
};

#define iommu_domain_to_riscv(iommu_domain) \
	container_of(iommu_domain, struct riscv_iommu_domain, domain)

/*
 * Send IOTLB.INVAL for whole address space for ranges larger than 2MB.
 * This limit will be replaced with range invalidations, if supported by
 * the hardware, when RISC-V IOMMU architecture specification update for
 * range invalidations update will be available.
 */
#define RISCV_IOMMU_IOTLB_INVAL_LIMIT	(2 << 20)

static void riscv_iommu_iotlb_inval(struct riscv_iommu_domain *domain,
				    unsigned long start, unsigned long end)
{
	struct riscv_iommu_bond *bond;
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_command cmd;
	unsigned long len = end - start + 1;
	unsigned long iova;

	rcu_read_lock();
	list_for_each_entry_rcu(bond, &domain->bonds, list) {
		iommu = dev_to_iommu(bond->dev);
		riscv_iommu_cmd_inval_vma(&cmd);
		riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);
		if (len > 0 && len < RISCV_IOMMU_IOTLB_INVAL_LIMIT) {
			for (iova = start; iova < end; iova += PAGE_SIZE) {
				riscv_iommu_cmd_inval_set_addr(&cmd, iova);
				riscv_iommu_cmd_send(iommu, &cmd, 0);
			}
		} else {
			riscv_iommu_cmd_send(iommu, &cmd, 0);
		}
	}

	list_for_each_entry_rcu(bond, &domain->bonds, list) {
		iommu = dev_to_iommu(bond->dev);

		riscv_iommu_cmd_iofence(&cmd);
		riscv_iommu_cmd_send(iommu, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);
	}
	rcu_read_unlock();
}

static int riscv_iommu_attach_domain(struct riscv_iommu_device *iommu,
				     struct device *dev,
				     struct iommu_domain *iommu_domain)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct riscv_iommu_domain *domain;
	struct riscv_iommu_dc *dc;
	struct riscv_iommu_bond *bond = NULL, *b;
	struct riscv_iommu_command cmd;
	u64 fsc, ta, tc;
	int i;

	if (!iommu_domain) {
		ta = 0;
		tc = 0;
		fsc = 0;
	} else if (iommu_domain->type == IOMMU_DOMAIN_IDENTITY) {
		ta = 0;
		tc = RISCV_IOMMU_DC_TC_V;
		fsc = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_MODE_BARE);
	} else if (iommu_domain->type & __IOMMU_DOMAIN_PAGING) {
		domain = iommu_domain_to_riscv(iommu_domain);

		ta = FIELD_PREP(RISCV_IOMMU_PC_TA_PSCID, domain->pscid);
		tc = RISCV_IOMMU_DC_TC_V;
		if (domain->amo_enabled)
			tc |= RISCV_IOMMU_DC_TC_SADE;
		fsc = FIELD_PREP(RISCV_IOMMU_PC_FSC_MODE, domain->pgd_mode) |
		      FIELD_PREP(RISCV_IOMMU_PC_FSC_PPN, virt_to_pfn(domain->pgd_root));

		bond = kzalloc(sizeof(*bond), GFP_KERNEL);
		if (!bond)
			return -ENOMEM;
		bond->dev = dev;
	} else {
		/* This should never happen. */
		return -ENODEV;
	}

	/* Update existing or allocate new entries in device directory */
	for (i = 0; i < fwspec->num_ids; i++) {
		dc = riscv_iommu_get_dc(iommu, fwspec->ids[i], !iommu_domain);
		if (!dc && !iommu_domain)
			continue;
		if (!dc)
			return -ENODEV;

		/* Swap device context, update TC valid bit as the last operation */
		xchg64(&dc->fsc, fsc);
		xchg64(&dc->ta, ta);
		xchg64(&dc->tc, tc);

		if (!(tc & RISCV_IOMMU_DC_TC_V))
			continue;

		/* Invalidate device context cache */
		riscv_iommu_cmd_iodir_inval_ddt(&cmd);
		riscv_iommu_cmd_iodir_set_did(&cmd, fwspec->ids[i]);
		riscv_iommu_cmd_send(iommu, &cmd, 0);

		if (FIELD_GET(RISCV_IOMMU_PC_FSC_MODE, fsc) == RISCV_IOMMU_DC_FSC_MODE_BARE)
			continue;

		/* Invalidate last valid PSCID */
		riscv_iommu_cmd_inval_vma(&cmd);
		riscv_iommu_cmd_inval_set_pscid(&cmd, FIELD_GET(RISCV_IOMMU_DC_TA_PSCID, ta));
		riscv_iommu_cmd_send(iommu, &cmd, 0);
	}

	/* Synchronize directory update */
	riscv_iommu_cmd_iofence(&cmd);
	riscv_iommu_cmd_send(iommu, &cmd, RISCV_IOMMU_IOTINVAL_TIMEOUT);

	/* Track domain to devices mapping. */
	if (bond)
		list_add_rcu(&bond->list, &domain->bonds);

	/* Remove tracking from previous domain, if needed. */
	iommu_domain = iommu_get_domain_for_dev(dev);
	if (iommu_domain && !!(iommu_domain->type & __IOMMU_DOMAIN_PAGING)) {
		domain = iommu_domain_to_riscv(iommu_domain);
		bond = NULL;
		rcu_read_lock();
		list_for_each_entry_rcu(b, &domain->bonds, list) {
			if (b->dev == dev) {
				bond = b;
				break;
			}
		}
		rcu_read_unlock();

		if (bond) {
			list_del_rcu(&bond->list);
			kfree_rcu(bond, rcu);
		}
	}

	return 0;
}

/*
 * IOVA page translation tree management.
 */

#define IOMMU_PAGE_SIZE_4K     BIT_ULL(12)
#define IOMMU_PAGE_SIZE_2M     BIT_ULL(21)
#define IOMMU_PAGE_SIZE_1G     BIT_ULL(30)
#define IOMMU_PAGE_SIZE_512G   BIT_ULL(39)

#define PT_SHIFT (PAGE_SHIFT - ilog2(sizeof(pte_t)))

static void riscv_iommu_flush_iotlb_all(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	riscv_iommu_iotlb_inval(domain, 0, ULONG_MAX);
}

static void riscv_iommu_iotlb_sync(struct iommu_domain *iommu_domain,
				   struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	riscv_iommu_iotlb_inval(domain, gather->start, gather->end);
}

static inline size_t get_page_size(size_t size)
{
	if (size >= IOMMU_PAGE_SIZE_512G)
		return IOMMU_PAGE_SIZE_512G;
	if (size >= IOMMU_PAGE_SIZE_1G)
		return IOMMU_PAGE_SIZE_1G;
	if (size >= IOMMU_PAGE_SIZE_2M)
		return IOMMU_PAGE_SIZE_2M;
	return IOMMU_PAGE_SIZE_4K;
}

#define _io_pte_present(pte)	((pte) & (_PAGE_PRESENT | _PAGE_PROT_NONE))
#define _io_pte_leaf(pte)	((pte) & _PAGE_LEAF)
#define _io_pte_none(pte)	((pte) == 0)
#define _io_pte_entry(pn, prot)	((_PAGE_PFN_MASK & ((pn) << _PAGE_PFN_SHIFT)) | (prot))

static void riscv_iommu_pte_free(struct riscv_iommu_domain *domain,
				 unsigned long pte, struct list_head *freelist)
{
	unsigned long *ptr;
	int i;

	if (!_io_pte_present(pte) || _io_pte_leaf(pte))
		return;

	ptr = (unsigned long *)pfn_to_virt(__page_val_to_pfn(pte));

	/* Recursively free all sub page table pages */
	for (i = 0; i < PTRS_PER_PTE; i++) {
		pte = READ_ONCE(ptr[i]);
		if (!_io_pte_none(pte) && cmpxchg_relaxed(ptr + i, pte, 0) == pte)
			riscv_iommu_pte_free(domain, pte, freelist);
	}

	if (freelist)
		list_add_tail(&virt_to_page(ptr)->lru, freelist);
	else
		free_page((unsigned long)ptr);
}

static unsigned long *riscv_iommu_pte_alloc(struct riscv_iommu_domain *domain,
					    unsigned long iova, size_t pgsize, gfp_t gfp)
{
	unsigned long *ptr = (unsigned long *)domain->pgd_root;
	unsigned long pte, old;
	int level = domain->pgd_mode - RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV39 + 2;
	struct page *page;

	do {
		const int shift = PAGE_SHIFT + PT_SHIFT * level;

		ptr += ((iova >> shift) & (PTRS_PER_PTE - 1));
		/*
		 * Note: returned entry might be a non-leaf if there was existing mapping
		 * with smaller granularity. Up to the caller to replace and invalidate.
		 */
		if (((size_t)1 << shift) == pgsize)
			return ptr;
pte_retry:
		pte = READ_ONCE(*ptr);
		/*
		 * This is very likely incorrect as we should not be adding new mapping
		 * with smaller granularity on top of existing 2M/1G mapping. Fail.
		 */
		if (_io_pte_present(pte) && _io_pte_leaf(pte))
			return NULL;
		/*
		 * Non-leaf entry is missing, allocate and try to add to the page table.
		 * This might race with other mappings, retry on error.
		 */
		if (_io_pte_none(pte)) {
			page = alloc_pages_node(domain->numa_node, __GFP_ZERO | gfp, 0);
			if (!page)
				return NULL;
			old = pte;
			pte = _io_pte_entry(page_to_pfn(page), _PAGE_TABLE);
			if (cmpxchg_relaxed(ptr, old, pte) != old) {
				__free_pages(page, 0);
				goto pte_retry;
			}
		}
		ptr = (unsigned long *)pfn_to_virt(__page_val_to_pfn(pte));
	} while (level-- > 0);

	return NULL;
}

static unsigned long *riscv_iommu_pte_fetch(struct riscv_iommu_domain *domain,
					    unsigned long iova, size_t *pte_pgsize)
{
	unsigned long *ptr = (unsigned long *)domain->pgd_root;
	unsigned long pte;
	int level = domain->pgd_mode - RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV39 + 2;

	do {
		const int shift = PAGE_SHIFT + PT_SHIFT * level;

		ptr += ((iova >> shift) & (PTRS_PER_PTE - 1));
		pte = READ_ONCE(*ptr);
		if (_io_pte_present(pte) && _io_pte_leaf(pte)) {
			*pte_pgsize = (size_t)1 << shift;
			return ptr;
		}
		if (_io_pte_none(pte))
			return NULL;
		ptr = (unsigned long *)pfn_to_virt(__page_val_to_pfn(pte));
	} while (level-- > 0);

	return NULL;
}

static int riscv_iommu_map_pages(struct iommu_domain *iommu_domain,
				 unsigned long iova, phys_addr_t phys,
				 size_t pgsize, size_t pgcount, int prot,
				 gfp_t gfp, size_t *mapped)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	size_t size = 0;
	size_t page_size = get_page_size(pgsize);
	unsigned long *ptr;
	unsigned long pte, old, pte_prot;

	if (!(prot & IOMMU_WRITE))
		pte_prot = _PAGE_BASE | _PAGE_READ;
	else if (domain->amo_enabled)
		pte_prot = _PAGE_BASE | _PAGE_READ | _PAGE_WRITE;
	else
		pte_prot = _PAGE_BASE | _PAGE_READ | _PAGE_WRITE | _PAGE_DIRTY;

	while (pgcount) {
		ptr = riscv_iommu_pte_alloc(domain, iova, page_size, gfp);
		if (!ptr) {
			*mapped = size;
			return -ENOMEM;
		}

		old = READ_ONCE(*ptr);
		pte = _io_pte_entry(phys_to_pfn(phys), pte_prot);
		if (cmpxchg_relaxed(ptr, old, pte) != old)
			continue;

		/* TODO: non-leaf page invalidation is pending spec update */
		riscv_iommu_pte_free(domain, old, NULL);

		size += page_size;
		iova += page_size;
		phys += page_size;
		--pgcount;
	}

	*mapped = size;

	return 0;
}

static size_t riscv_iommu_unmap_pages(struct iommu_domain *iommu_domain,
				      unsigned long iova, size_t pgsize, size_t pgcount,
				      struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	size_t size = pgcount << __ffs(pgsize);
	unsigned long *ptr, old;
	size_t unmapped = 0;
	size_t pte_size;

	while (unmapped < size) {
		ptr = riscv_iommu_pte_fetch(domain, iova, &pte_size);
		if (!ptr)
			return unmapped;

		/* partial unmap is not allowed, fail. */
		if (iova & ~(pte_size - 1))
			return unmapped;

		old = READ_ONCE(*ptr);
		if (cmpxchg_relaxed(ptr, old, 0) != old)
			continue;

		iommu_iotlb_gather_add_page(&domain->domain, gather, iova,
					    pte_size);

		iova += pte_size;
		unmapped += pte_size;
	}

	return unmapped;
}

static phys_addr_t riscv_iommu_iova_to_phys(struct iommu_domain *iommu_domain, dma_addr_t iova)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	unsigned long pte_size;
	unsigned long *ptr;

	ptr = riscv_iommu_pte_fetch(domain, iova, &pte_size);
	if (_io_pte_none(*ptr) || !_io_pte_present(*ptr))
		return 0;

	return pfn_to_phys(__page_val_to_pfn(*ptr)) | (iova & (pte_size - 1));
}

static void riscv_iommu_free_paging_domain(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	WARN_ON(!list_empty(&domain->bonds));

	if (domain->pgd_root) {
		const unsigned long pfn = virt_to_pfn(domain->pgd_root);

		riscv_iommu_pte_free(domain, _io_pte_entry(pfn, _PAGE_TABLE), NULL);
	}

	if ((int)domain->pscid > 0)
		ida_free(&riscv_iommu_pscids, domain->pscid);

	kfree(domain);
}

static bool riscv_iommu_pt_supported(struct riscv_iommu_device *iommu, int pgd_mode)
{
	switch (pgd_mode) {
	case RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV39:
		return iommu->caps & RISCV_IOMMU_CAP_S_SV39;

	case RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV48:
		return iommu->caps & RISCV_IOMMU_CAP_S_SV48;

	case RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV57:
		return iommu->caps & RISCV_IOMMU_CAP_S_SV57;
	}
	return false;
}

static int riscv_iommu_attach_paging_domain(struct iommu_domain *iommu_domain,
					    struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct page *page;

	if (!riscv_iommu_pt_supported(iommu, domain->pgd_mode))
		return -ENODEV;

	domain->numa_node = dev_to_node(iommu->dev);
	domain->amo_enabled = !!(iommu->caps & RISCV_IOMMU_CAP_AMO_HWAD);

	if (!domain->pgd_root) {
		page = alloc_pages_node(domain->numa_node,
					GFP_KERNEL_ACCOUNT | __GFP_ZERO, 0);
		if (!page)
			return -ENOMEM;
		domain->pgd_root = (unsigned long)page_to_virt(page);
	}

	return riscv_iommu_attach_domain(iommu, dev, iommu_domain);
}

static const struct iommu_domain_ops riscv_iommu_paging_domain_ops = {
	.attach_dev = riscv_iommu_attach_paging_domain,
	.free = riscv_iommu_free_paging_domain,
	.map_pages = riscv_iommu_map_pages,
	.unmap_pages = riscv_iommu_unmap_pages,
	.iova_to_phys = riscv_iommu_iova_to_phys,
	.iotlb_sync = riscv_iommu_iotlb_sync,
	.flush_iotlb_all = riscv_iommu_flush_iotlb_all,
};

static struct iommu_domain *riscv_iommu_alloc_paging_domain(struct device *dev)
{
	struct riscv_iommu_domain *domain;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD_RCU(&domain->bonds);

	domain->pscid = ida_alloc_range(&riscv_iommu_pscids, 1,
					RISCV_IOMMU_MAX_PSCID - 1, GFP_KERNEL);
	if (domain->pscid < 0) {
		kfree(domain);
		return ERR_PTR(-ENOMEM);
	}

	/*
	 * Note: RISC-V Privilege spec mandates that virtual addresses
	 * need to be sign-extended, so if (VA_BITS - 1) is set, all
	 * bits >= VA_BITS need to also be set or else we'll get a
	 * page fault. However the code that creates the mappings
	 * above us (e.g. iommu_dma_alloc_iova()) won't do that for us
	 * for now, so we'll end up with invalid virtual addresses
	 * to map. As a workaround until we get this sorted out
	 * limit the available virtual addresses to VA_BITS - 1.
	 */
	domain->domain.geometry.aperture_start = 0;
	domain->domain.geometry.aperture_end = DMA_BIT_MASK(VA_BITS - 1);
	domain->domain.geometry.force_aperture = true;

	/*
	 * Follow system address translation mode.
	 * RISC-V IOMMU ATP mode values match RISC-V CPU SATP mode values.
	 */
	domain->pgd_mode = satp_mode >> SATP_MODE_SHIFT;
	domain->numa_node = NUMA_NO_NODE;
	domain->domain.ops = &riscv_iommu_paging_domain_ops;

	return &domain->domain;
}

static int riscv_iommu_attach_identity_domain(struct iommu_domain *iommu_domain,
					      struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);

	/* Global pass-through already enabled, do nothing. */
	if (iommu->ddt_mode == RISCV_IOMMU_DDTP_MODE_BARE)
		return 0;

	return riscv_iommu_attach_domain(iommu, dev, iommu_domain);
}

static struct iommu_domain riscv_iommu_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &(const struct iommu_domain_ops) {
		.attach_dev = riscv_iommu_attach_identity_domain,
	}
};

static int riscv_iommu_device_domain_type(struct device *dev)
{
	return 0;
}

static struct iommu_group *riscv_iommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	return generic_device_group(dev);
}

static int riscv_iommu_of_xlate(struct device *dev, const struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static struct iommu_device *riscv_iommu_probe_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct riscv_iommu_device *iommu;

	if (!fwspec->iommu_fwnode->dev || !fwspec->num_ids)
		return ERR_PTR(-ENODEV);

	iommu = dev_get_drvdata(fwspec->iommu_fwnode->dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	return &iommu->iommu;
}

static void riscv_iommu_probe_finalize(struct device *dev)
{
	iommu_setup_dma_ops(dev, 0, U64_MAX);
}

static void riscv_iommu_release_device(struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);

	riscv_iommu_attach_domain(iommu, dev, NULL);
}

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K,
	.of_xlate = riscv_iommu_of_xlate,
	.identity_domain = &riscv_iommu_identity_domain,
	.domain_alloc_paging = riscv_iommu_alloc_paging_domain,
	.def_domain_type = riscv_iommu_device_domain_type,
	.device_group = riscv_iommu_device_group,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
};

static int riscv_iommu_init_check(struct riscv_iommu_device *iommu)
{
	u64 ddtp;

	/* Hardware must be configured in OFF | BARE mode at system initialization. */
	riscv_iommu_readq_timeout(iommu, RISCV_IOMMU_REG_DDTP,
				  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
				  10, RISCV_IOMMU_DDTP_TIMEOUT);
	if (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp) > RISCV_IOMMU_DDTP_MODE_BARE)
		return -EBUSY;

	/* Configure accesses to in-memory data structures for CPU-native byte order. */
	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN) != !!(iommu->fctl & RISCV_IOMMU_FCTL_BE)) {
		if (!(iommu->caps & RISCV_IOMMU_CAP_END))
			return -EINVAL;
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_FCTL,
				   iommu->fctl ^ RISCV_IOMMU_FCTL_BE);
		iommu->fctl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FCTL);
		if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN) != !!(iommu->fctl & RISCV_IOMMU_FCTL_BE))
			return -EINVAL;
	}

	/* Distribute interrupt vectors, always use first vector for CIV */
	iommu->ivec = 0;
	if (iommu->irqs_count) {
		iommu->ivec |= FIELD_PREP(RISCV_IOMMU_IVEC_FIV, 1 % iommu->irqs_count);
		iommu->ivec |= FIELD_PREP(RISCV_IOMMU_IVEC_PIV, 2 % iommu->irqs_count);
		iommu->ivec |= FIELD_PREP(RISCV_IOMMU_IVEC_PMIV, 3 % iommu->irqs_count);
	}
	riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_IVEC, iommu->ivec);

	/* Read back and verify */
	iommu->ivec = riscv_iommu_readq(iommu, RISCV_IOMMU_REG_IVEC);
	if (riscv_iommu_queue_vec(iommu, RISCV_IOMMU_IVEC_CIV) >= RISCV_IOMMU_INTR_COUNT ||
	    riscv_iommu_queue_vec(iommu, RISCV_IOMMU_IVEC_FIV) >= RISCV_IOMMU_INTR_COUNT ||
	    riscv_iommu_queue_vec(iommu, RISCV_IOMMU_IVEC_PIV) >= RISCV_IOMMU_INTR_COUNT ||
	    riscv_iommu_queue_vec(iommu, RISCV_IOMMU_IVEC_PMIV) >= RISCV_IOMMU_INTR_COUNT)
		return -EINVAL;

	dma_set_mask_and_coherent(iommu->dev,
				  DMA_BIT_MASK(FIELD_GET(RISCV_IOMMU_CAP_PAS, iommu->caps)));

	return 0;
}

void riscv_iommu_remove(struct riscv_iommu_device *iommu)
{
	iommu_device_unregister(&iommu->iommu);
	iommu_device_sysfs_remove(&iommu->iommu);
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	riscv_iommu_queue_disable(&iommu->cmdq);
	riscv_iommu_queue_disable(&iommu->fltq);
}

int riscv_iommu_init(struct riscv_iommu_device *iommu)
{
	int rc;

	RISCV_IOMMU_QUEUE_INIT(&iommu->cmdq, CQ);
	RISCV_IOMMU_QUEUE_INIT(&iommu->fltq, FQ);

	rc = riscv_iommu_init_check(iommu);
	if (rc)
		return dev_err_probe(iommu->dev, rc, "unexpected device state\n");

	rc = riscv_iommu_ddt_alloc(iommu);
	if (WARN(rc, "cannot allocate device directory\n"))
		goto err_init;

	rc = riscv_iommu_queue_alloc(iommu, &iommu->cmdq, sizeof(struct riscv_iommu_command));
	if (WARN(rc, "cannot allocate command queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_alloc(iommu, &iommu->fltq, sizeof(struct riscv_iommu_fq_record));
	if (WARN(rc, "cannot allocate fault queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(iommu, &iommu->cmdq, riscv_iommu_cmdq_process);
	if (WARN(rc, "cannot enable command queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(iommu, &iommu->fltq, riscv_iommu_fltq_process);
	if (WARN(rc, "cannot enable fault queue\n"))
		goto err_init;

	rc = riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_MAX);
	if (WARN(rc, "cannot enable iommu device\n"))
		goto err_init;

	rc = iommu_device_sysfs_add(&iommu->iommu, NULL, NULL, "riscv-iommu@%s",
				    dev_name(iommu->dev));
	if (WARN(rc, "cannot register sysfs interface\n"))
		goto err_sysfs;

	rc = iommu_device_register(&iommu->iommu, &riscv_iommu_ops, iommu->dev);
	if (WARN(rc, "cannot register iommu interface\n"))
		goto err_iommu;

	return 0;

err_iommu:
	iommu_device_sysfs_remove(&iommu->iommu);
err_sysfs:
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
err_init:
	riscv_iommu_queue_disable(&iommu->fltq);
	riscv_iommu_queue_disable(&iommu->cmdq);
	return rc;
}
