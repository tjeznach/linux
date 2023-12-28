// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V IOMMU implementations.
 *
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/compiler.h>
#include <linux/debugfs.h>
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
MODULE_ALIAS("riscv-iommu");
MODULE_LICENSE("GPL v2");

/* Timeouts in [us] */
#define RISCV_IOMMU_DDTP_TIMEOUT	50000
#define RISCV_IOMMU_QCSR_TIMEOUT	50000
#define RISCV_IOMMU_QUEUE_TIMEOUT	10000
#define RISCV_IOMMU_IOFENCE_TIMEOUT	1500000

/* Number of entries per CMD/FLT queue, should be <= INT_MAX */
#define RISCV_IOMMU_DEF_CQ_COUNT	8192
#define RISCV_IOMMU_DEF_FQ_COUNT	8192

/* RISC-V IOMMU PPN <> PHYS address conversions, PHYS <=> PPN[53:10] */
#define phys_to_ppn(va)  (((va) >> 2) & (((1ULL << 44) - 1) << 10))
#define ppn_to_phys(pn)	 (((pn) << 2) & (((1ULL << 44) - 1) << 12))

#define dev_to_iommu(dev) \
	container_of((dev)->iommu->iommu_dev, struct riscv_iommu_device, iommu)

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

static unsigned long riscv_iommu_get_pages(struct riscv_iommu_device *iommu,
					   unsigned int order)
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

	WARN_ON(devres_release(iommu->dev, riscv_iommu_devres_pages_release,
			       riscv_iommu_devres_pages_match, &devres));
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
			const unsigned int order = get_order(entry_size << (logsz + 1));

			addr = riscv_iommu_get_pages(iommu, order);
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
	if (queue->mask != (2U << logsz) - 1) {
		queue->mask = (2U << logsz) - 1;
		dev_info(iommu->dev, "queue #%u restricted to 2^%u entries",
			 queue->qid, logsz + 1);
	}

	queue->iommu = iommu;

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

/*
 * Enable queue processing in the hardware, register interrupt handler.
 *
 * @queue - data structure, already allocated with riscv_iommu_queue_alloc()
 * @irq_handler - threaded interrupt handler.
 */
static int riscv_iommu_queue_enable(struct riscv_iommu_queue *queue,
				    irq_handler_t irq_handler)
{
	struct riscv_iommu_device *iommu = queue->iommu;
	const int vec = (iommu->ivec >> (queue->qid * 4)) % RISCV_IOMMU_INTR_COUNT;
	const unsigned int irq = iommu->irqs[vec];
	u32 csr;
	int rc;

	/* Polling not implemented */
	if (!irq)
		return -ENODEV;

	rc = request_threaded_irq(irq, riscv_iommu_queue_ipsr, irq_handler,
				  IRQF_ONESHOT | IRQF_SHARED, dev_name(iommu->dev), queue);
	if (rc)
		return rc;

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
		return -EBUSY;
	}

	queue->active = true;

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
	const int vec = (iommu->ivec >> (queue->qid * 4)) % RISCV_IOMMU_INTR_COUNT;
	u32 csr;

	if (!iommu || !queue->active)
		return;

	queue->active = false;
	free_irq(iommu->irqs[vec], queue);
	riscv_iommu_writel(iommu, queue->qcr, 0);
	riscv_iommu_readl_timeout(iommu, queue->qcr,
				  csr, !(csr & RISCV_IOMMU_QUEUE_BUSY),
				  10, RISCV_IOMMU_QCSR_TIMEOUT);

	if (csr & (RISCV_IOMMU_QUEUE_ACTIVE | RISCV_IOMMU_QUEUE_BUSY))
		dev_err(iommu->dev, "fail to disable hardware queue #%u, csr 0x%x\n",
			queue->qid, csr);
}

/*
 * Returns number of available valid queue entries and the first item index or negative
 * error code.  Update shadow producer index if nessesary.
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

/*
 * Waits for available producer slot in the queue. MP safe.
 * Returns negative error code in case of timeout.
 * Submission via riscv_iommu_queue_submit() should happen as soon as possible.
 */
static int riscv_iommu_queue_aquire(struct riscv_iommu_queue *queue, unsigned int *index,
				    unsigned int timeout_us)
{
	unsigned int prod = atomic_fetch_add(1, &queue->prod);
	unsigned int head = atomic_read(&queue->head);

	*index = prod;

	if ((prod - head) > queue->mask) {
		/* Wait for queue space availability */
		if (readx_poll_timeout(atomic_read, &queue->head,
				       head, (prod - head) < queue->mask, 0, timeout_us))
			return -EBUSY;
	} else if ((prod - head) == queue->mask) {
		/*
		 * Update consumer shadow index and check reserved register bits are not set,
		 * and wait for space availability.
		 */
		const unsigned int last = Q_ITEM(queue, head);

		if (riscv_iommu_readl_timeout(queue->iommu, Q_HEAD(queue), head,
					      !(head & ~queue->mask) && head != last,
					      0, timeout_us))
			return -EBUSY;
		atomic_add((head - last) & queue->mask, &queue->head);
	}

	return 0;
}

/*
 * Ordered write to producer hardware register.
 * @index should match value allocated by riscv_iommu_queue_aquire() call.
 */
static int riscv_iommu_queue_submit(struct riscv_iommu_queue *queue, unsigned int index)
{
	unsigned int tail;

	if (readx_poll_timeout(atomic_read, &queue->tail, tail, index == tail,
			       0, RISCV_IOMMU_QUEUE_TIMEOUT))
		return -EBUSY;

	riscv_iommu_writel(queue->iommu, Q_TAIL(queue), Q_ITEM(queue, index + 1));
	atomic_inc(&queue->tail);

	return 0;
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

/* Enqueue command and wait to be processed if timeout_us > 0 */
static int riscv_iommu_queue_send(struct riscv_iommu_queue *queue,
				  struct riscv_iommu_command *cmd,
				  unsigned int timeout_us)
{
	unsigned int idx;

	if (WARN_ON(riscv_iommu_queue_aquire(queue, &idx, RISCV_IOMMU_QUEUE_TIMEOUT)))
		return -EBUSY;

	((struct riscv_iommu_command *)queue->base)[Q_ITEM(queue, idx)] = *cmd;

	if (WARN_ON(riscv_iommu_queue_submit(queue, idx)))
		return -EBUSY;

	if (timeout_us)
		return riscv_iommu_queue_wait(queue, idx, timeout_us);

	return 0;
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

static inline void riscv_iommu_cmd_inval_vma(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IOTINVAL_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IOTINVAL_FUNC_VMA);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_inval_set_addr(struct riscv_iommu_command *cmd,
						  u64 addr)
{
	cmd->dword0 |= RISCV_IOMMU_CMD_IOTINVAL_AV;
	cmd->dword1 = FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_ADDR, phys_to_pfn(addr));
}

static inline void riscv_iommu_cmd_inval_set_pscid(struct riscv_iommu_command *cmd,
						   int pscid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_PSCID, pscid) |
		       RISCV_IOMMU_CMD_IOTINVAL_PSCV;
}

static inline void riscv_iommu_cmd_inval_set_gscid(struct riscv_iommu_command *cmd,
						   int gscid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_GSCID, gscid) |
		       RISCV_IOMMU_CMD_IOTINVAL_GV;
}

static inline void riscv_iommu_cmd_iofence(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IOFENCE_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IOFENCE_FUNC_C);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_iofence_set_av(struct riscv_iommu_command *cmd,
						  u64 addr, u32 data)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IOFENCE_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IOFENCE_FUNC_C) |
		      FIELD_PREP(RISCV_IOMMU_CMD_IOFENCE_DATA, data) |
		      RISCV_IOMMU_CMD_IOFENCE_AV;
	cmd->dword1 = addr >> 2;
}

static inline void riscv_iommu_cmd_iodir_inval_ddt(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IODIR_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IODIR_FUNC_INVAL_DDT);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_iodir_inval_pdt(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IODIR_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IODIR_FUNC_INVAL_PDT);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_iodir_set_did(struct riscv_iommu_command *cmd,
						 unsigned int devid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IODIR_DID, devid) |
		       RISCV_IOMMU_CMD_IODIR_DV;
}

static inline void riscv_iommu_cmd_iodir_set_pid(struct riscv_iommu_command *cmd,
						 unsigned int pasid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IODIR_PID, pasid);
}

static inline void riscv_iommu_cmd_ats_inval(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_ATS_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_ATS_FUNC_INVAL);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_ats_set_devid(struct riscv_iommu_command *cmd,
						 unsigned int devid)
{
	const unsigned int seg = (devid & 0x0ff0000) >> 16;
	const unsigned int rid = (devid & 0x000ffff);

	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_DSEG, seg) | RISCV_IOMMU_CMD_ATS_DSV |
		       FIELD_PREP(RISCV_IOMMU_CMD_ATS_RID, rid);
}

static inline void riscv_iommu_cmd_ats_set_pid(struct riscv_iommu_command *cmd,
					       unsigned int pid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_PID, pid) | RISCV_IOMMU_CMD_ATS_PV;
}

static inline void riscv_iommu_cmd_ats_set_range(struct riscv_iommu_command *cmd,
						 unsigned long start, unsigned long end,
						 bool global_inv)
{
	size_t len = end - start + 1;
	u64 payload = 0;

	/*
	 * PCI Express specification
	 * Section 10.2.3.2 Translation Range Size (S) Field
	 */
	if (len < PAGE_SIZE)
		len = PAGE_SIZE;
	else
		len = __roundup_pow_of_two(len);

	payload = (start & ~(len - 1)) | (((len - 1) >> 12) << 11);

	if (global_inv)
		payload |= RISCV_IOMMU_CMD_ATS_INVAL_G;

	cmd->dword1 = payload;
}

static inline void riscv_iommu_cmd_ats_set_all(struct riscv_iommu_command *cmd,
					       bool global_inv)
{
	u64 payload = GENMASK_ULL(62, 11);

	if (global_inv)
		payload |= RISCV_IOMMU_CMD_ATS_INVAL_G;

	cmd->dword1 = payload;
}

static inline void riscv_iommu_cmd_prgr(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_ATS_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_ATS_FUNC_PRGR);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_prgr_set_devid(struct riscv_iommu_command *cmd,
						  unsigned int devid)
{
	const unsigned int seg = (devid & 0x0ff0000) >> 16;
	const unsigned int rid = (devid & 0x000ffff);

	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_DSEG, seg) | RISCV_IOMMU_CMD_ATS_DSV |
		       FIELD_PREP(RISCV_IOMMU_CMD_ATS_RID, rid);
	cmd->dword1 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_PRGR_DST_ID, rid);
}

static inline void riscv_iommu_cmd_prgr_set_response(struct riscv_iommu_command *cmd,
						     int group, int code)
{
	cmd->dword1 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_PRGR_RESP_CODE, code) |
		       FIELD_PREP(RISCV_IOMMU_CMD_ATS_PRGR_PRG_INDEX, group);
}

static inline void riscv_iommu_cmd_prgr_set_pid(struct riscv_iommu_command *cmd,
						unsigned int pid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_PID, pid) | RISCV_IOMMU_CMD_ATS_PV;
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

/* Fault queue interrupt hanlder thread function */
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
						 unsigned int devid)
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

	/* Disallow state transtion from xLVL to xLVL. */
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
	riscv_iommu_queue_send(&iommu->cmdq, &cmd, 0);

	/* Invalidate address translation cache */
	riscv_iommu_cmd_inval_vma(&cmd);
	riscv_iommu_queue_send(&iommu->cmdq, &cmd, 0);

	/* IOFENCE.C */
	riscv_iommu_cmd_iofence(&cmd);
	return riscv_iommu_queue_send(&iommu->cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);
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

// lockdep_assert_held(&group->mutex);
static int riscv_iommu_attach_domain(struct device *dev,
				     struct iommu_domain *domain)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	const bool was_attached = ep->attached;
	struct riscv_iommu_dc *dc;
	u64 atp, ta, tc;

	if (!domain && !was_attached)
		return 0;

	/* Find DC for the endpoint */
	dc = riscv_iommu_get_dc(iommu, ep->devid);
	if (!dc)
		return -ENODEV;

	if (!domain) {
		WRITE_ONCE(dc->tc, 0);
		ep->attached = false;
	} else {
		ta = 0;
		if (domain->type == IOMMU_DOMAIN_IDENTITY)
			atp = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_MODE_BARE);
		else
			return -ENODEV;
		WRITE_ONCE(dc->fsc, atp);
		/* Prevent incomplete PC state being observable */
		smp_wmb();
		WRITE_ONCE(dc->ta, ta);
	}

	if (!was_attached) {
		/* Configure 2nd stage translation to identity mapping */
		dc->iohgatp = FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_MODE,
					 RISCV_IOMMU_DC_IOHGATP_MODE_BARE);

		/* Configure translation context. */
		tc = RISCV_IOMMU_DC_TC_V;

		/* Prevent incomplete DC/PC state being observable */
		smp_wmb();
		WRITE_ONCE(dc->tc, tc);

		/* mark as attached */
		ep->attached = true;
	}

	if (was_attached) {
		struct riscv_iommu_queue *cmdq = &iommu->cmdq;
		struct riscv_iommu_command cmd;

		/* Invalidate device context cache */
		riscv_iommu_cmd_iodir_inval_ddt(&cmd);
		riscv_iommu_cmd_iodir_set_did(&cmd, ep->devid);
		riscv_iommu_queue_send(cmdq, &cmd, 0);

		/* IOFENCE.C */
		riscv_iommu_cmd_iofence(&cmd);
		riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);
	}

	return 0;
}

static int riscv_iommu_attach_identity_domain(struct iommu_domain *domain,
					      struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);

	/* Global pass-through already enabled, do nothing. */
	if (iommu->ddt_mode == RISCV_IOMMU_DDTP_MODE_BARE)
		return 0;

	return riscv_iommu_attach_domain(dev, domain);
}

static struct iommu_domain riscv_iommu_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &(const struct iommu_domain_ops) {
		.attach_dev = riscv_iommu_attach_identity_domain,
	}
};

static int riscv_iommu_device_domain_type(struct device *dev)
{
	return IOMMU_DOMAIN_IDENTITY;
}

static struct iommu_group *riscv_iommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	return generic_device_group(dev);
}

static int riscv_iommu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static struct iommu_device *riscv_iommu_probe_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct riscv_iommu_device *iommu;
	struct pci_dev *pdev = dev_is_pci(dev) ? to_pci_dev(dev) : NULL;
	struct riscv_iommu_endpoint *ep;
	unsigned int devid;

	/* Early bus-scan exit, will retry. */
	if (!fwspec || !fwspec->iommu_fwnode)
		return ERR_PTR(-ENODEV);

	/* Pending IOMMU driver initialization, will retry */
	if (!fwspec->iommu_fwnode->dev)
		return ERR_PTR(-ENODEV);

	iommu = dev_get_drvdata(fwspec->iommu_fwnode->dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	if (pdev)
		devid = pci_dev_id(pdev) | pci_domain_nr(pdev->bus) << 16;
	else if (fwspec->num_ids)
		devid = fwspec->ids[0];
	else
		return ERR_PTR(-ENODEV);

	ep = kzalloc(sizeof(*ep), GFP_KERNEL_ACCOUNT);
	if (!ep)
		return ERR_PTR(-ENOMEM);

	ep->devid = devid;
	ep->dev = dev;

	dev_iommu_priv_set(dev, ep);

	return &iommu->iommu;
}

static void riscv_iommu_probe_finalize(struct device *dev)
{
	iommu_setup_dma_ops(dev, 0, U64_MAX);
}

static void riscv_iommu_release_device(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	riscv_iommu_attach_domain(dev, NULL);
	dev_iommu_priv_set(dev, NULL);
	kfree(ep);
}

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K | SZ_2M | SZ_1G,
	.of_xlate = riscv_iommu_of_xlate,
	.identity_domain = &riscv_iommu_identity_domain,
	.def_domain_type = riscv_iommu_device_domain_type,
	.device_group = riscv_iommu_device_group,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
};

void riscv_iommu_remove(struct riscv_iommu_device *iommu)
{
	iommu_device_unregister(&iommu->iommu);
	iommu_device_sysfs_remove(&iommu->iommu);
	riscv_iommu_debugfs_remove(iommu);
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	riscv_iommu_queue_disable(&iommu->cmdq);
	riscv_iommu_queue_disable(&iommu->fltq);
}

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

	/* Set 1:1 mapping for interrupt vectors if available */
	iommu->ivec = iommu->irqs_count < 4 ? 0 :
		      FIELD_PREP(RISCV_IOMMU_IVEC_CIV,  0) |
		      FIELD_PREP(RISCV_IOMMU_IVEC_FIV,  1) |
		      FIELD_PREP(RISCV_IOMMU_IVEC_PMIV, 2) |
		      FIELD_PREP(RISCV_IOMMU_IVEC_PIV,  3);
	riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_IVEC, iommu->ivec);

	dma_set_mask_and_coherent(iommu->dev,
				  DMA_BIT_MASK(FIELD_GET(RISCV_IOMMU_CAP_PAS, iommu->caps)));

	return 0;
}

int riscv_iommu_init(struct riscv_iommu_device *iommu)
{
	int rc;

	RISCV_IOMMU_QUEUE_INIT(&iommu->cmdq, CQ);
	RISCV_IOMMU_QUEUE_INIT(&iommu->fltq, FQ);

	rc = riscv_iommu_init_check(iommu);
	if (rc)
		return dev_err_probe(iommu->dev, rc, "unexpected device state\n");

	riscv_iommu_debugfs_setup(iommu);

	rc = riscv_iommu_ddt_alloc(iommu);
	if (WARN(rc, "cannot allocate device directory\n"))
		goto err_init;

	rc = riscv_iommu_queue_alloc(iommu, &iommu->cmdq, sizeof(struct riscv_iommu_command));
	if (WARN(rc, "cannot allocate command queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_alloc(iommu, &iommu->fltq, sizeof(struct riscv_iommu_fq_record));
	if (WARN(rc, "cannot allocate fault queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(&iommu->cmdq, riscv_iommu_cmdq_process);
	if (WARN(rc, "cannot enable command queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(&iommu->fltq, riscv_iommu_fltq_process);
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
	riscv_iommu_debugfs_remove(iommu);
	return rc;
}
