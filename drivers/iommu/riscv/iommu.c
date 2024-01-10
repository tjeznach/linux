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
#include <linux/pci-ats.h>

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
/* Timeout for IOT.INVAL and ATS.INVAL (up to 90 seconds per PCIe spec.) */
#define RISCV_IOMMU_INVAL_TIMEOUT	90000000

/* Number of entries per CMD/FLT queue, should be <= INT_MAX */
#define RISCV_IOMMU_DEF_CQ_COUNT	8192
#define RISCV_IOMMU_DEF_FQ_COUNT	8192
#define RISCV_IOMMU_DEF_PQ_COUNT	8192

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
		return WARN_ON(riscv_iommu_queue_wait(queue, idx, timeout_us));

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

/* Register device for IOMMU device-id based tracking. */
static void riscv_iommu_add_device(struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep, *rb_ep;
	struct rb_node **new_node, *parent_node = NULL;

	mutex_lock(&iommu->eps_mutex);

	ep = dev_iommu_priv_get(dev);

	new_node = &iommu->eps.rb_node;
	while (*new_node) {
		rb_ep = rb_entry(*new_node, struct riscv_iommu_endpoint, eps_node);
		parent_node = *new_node;
		if (rb_ep->devid > ep->devid) {
			new_node = &((*new_node)->rb_left);
		} else if (rb_ep->devid < ep->devid) {
			new_node = &((*new_node)->rb_right);
		} else {
			mutex_unlock(&iommu->eps_mutex);
			return;
		}
	}

	rb_link_node(&ep->eps_node, parent_node, new_node);
	rb_insert_color(&ep->eps_node, &iommu->eps);

	mutex_unlock(&iommu->eps_mutex);
}

/* Remove device from IOMMU tracking structures. */
static void riscv_iommu_del_device(struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	mutex_lock(&iommu->eps_mutex);
	rb_erase(&ep->eps_node, &iommu->eps);
	mutex_unlock(&iommu->eps_mutex);
}

/*
 * Get device reference based on device identifier (requester id).
 * Decrement reference count with put_device() call.
 */
static struct device *riscv_iommu_get_device(struct riscv_iommu_device *iommu,
					     unsigned int devid)
{
	struct rb_node *node;
	struct riscv_iommu_endpoint *ep;
	struct device *dev = NULL;

	mutex_lock(&iommu->eps_mutex);

	node = iommu->eps.rb_node;
	while (node && !dev) {
		ep = rb_entry(node, struct riscv_iommu_endpoint, eps_node);
		if (ep->devid < devid)
			node = node->rb_right;
		else if (ep->devid > devid)
			node = node->rb_left;
		else
			dev = get_device(ep->dev);
	}

	mutex_unlock(&iommu->eps_mutex);

	return dev;
}

static int riscv_iommu_page_response(struct device *dev,
				     struct iommu_fault_event *evt,
				     struct iommu_page_response *msg)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_command cmd;
	int code;

	switch (msg->code) {
	case IOMMU_PAGE_RESP_SUCCESS:
		code = 0b0000;
		break;
	case IOMMU_PAGE_RESP_INVALID:
		code = 0b0001;
		break;
	case IOMMU_PAGE_RESP_FAILURE:
		code = 0b1111;
		break;
	default:
		return -EINVAL;
	}

	riscv_iommu_cmd_prgr(&cmd);
	riscv_iommu_cmd_prgr_set_response(&cmd, msg->grpid, code);
	riscv_iommu_cmd_prgr_set_devid(&cmd, ep->devid);
	if (msg->flags & IOMMU_PAGE_RESP_PASID_VALID)
		riscv_iommu_cmd_prgr_set_pid(&cmd, msg->pasid);
	riscv_iommu_queue_send(&iommu->cmdq, &cmd, 0);

	return 0;
}

static void riscv_iommu_page_request(struct riscv_iommu_device *iommu,
				     struct riscv_iommu_pq_record *req)
{
	struct iommu_fault_event event = { 0 };
	struct iommu_fault_page_request *prm = &event.fault.prm;
	struct device *dev;

	/* Ignore PGR Stop marker. */
	if ((req->payload & RISCV_IOMMU_PREQ_PAYLOAD_M) == RISCV_IOMMU_PREQ_PAYLOAD_L)
		return;

	/* If device is no longer tracked by the IOMMU there is no point to process PRGR */
	dev = riscv_iommu_get_device(iommu, FIELD_GET(RISCV_IOMMU_PREQ_HDR_DID, req->hdr));
	if (!dev)
		return;

	event.fault.type = IOMMU_FAULT_PAGE_REQ;
	if (req->payload & RISCV_IOMMU_PREQ_PAYLOAD_L)
		prm->flags |= IOMMU_FAULT_PAGE_REQUEST_LAST_PAGE;
	if (req->payload & RISCV_IOMMU_PREQ_PAYLOAD_W)
		prm->perm |= IOMMU_FAULT_PERM_WRITE;
	if (req->payload & RISCV_IOMMU_PREQ_PAYLOAD_R)
		prm->perm |= IOMMU_FAULT_PERM_READ;
	prm->grpid = FIELD_GET(RISCV_IOMMU_PREQ_PRG_INDEX, req->payload);
	prm->addr = FIELD_GET(RISCV_IOMMU_PREQ_UADDR, req->payload) << PAGE_SHIFT;

	if (req->hdr & RISCV_IOMMU_PREQ_HDR_PV) {
		prm->pasid = FIELD_GET(RISCV_IOMMU_PREQ_HDR_PID, req->hdr);
		prm->flags |= IOMMU_FAULT_PAGE_REQUEST_PASID_VALID;
		if (dev_is_pci(dev) && pci_prg_resp_pasid_required(to_pci_dev(dev)))
			prm->flags |= IOMMU_FAULT_PAGE_RESPONSE_NEEDS_PASID;
	}

	if (iommu_report_device_fault(dev, &event)) {
		struct iommu_page_response resp = {
			.grpid = FIELD_GET(RISCV_IOMMU_PREQ_PRG_INDEX, req->payload),
			.pasid = FIELD_GET(RISCV_IOMMU_PREQ_HDR_PID, req->hdr),
			.flags = IOMMU_PAGE_RESP_PASID_VALID,
			.code  = IOMMU_PAGE_RESP_FAILURE,
		};
		riscv_iommu_page_response(dev, &event, &resp);
	}

	put_device(dev);
}

/* Page Request queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_priq_process(int irq, void *data)
{
	struct riscv_iommu_queue *queue = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu = queue->iommu;
	struct riscv_iommu_pq_record *events;
	unsigned int ctrl, idx;
	int cnt, len;

	events = (struct riscv_iommu_pq_record *)queue->base;

	/* Clear fault interrupt pending and process all received events. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, Q_IPSR(queue));

	do {
		cnt = riscv_iommu_queue_consume(queue, &idx);
		for (len = 0; len < cnt; idx++, len++)
			riscv_iommu_page_request(iommu, &events[Q_ITEM(queue, idx)]);
		riscv_iommu_queue_release(queue, cnt);
	} while (cnt > 0);

	/* Clear MF/OF errors, complete error recovery to be implemented. */
	ctrl = riscv_iommu_readl(iommu, queue->qcr);
	if (ctrl & (RISCV_IOMMU_PQCSR_PQMF | RISCV_IOMMU_PQCSR_PQOF)) {
		riscv_iommu_writel(iommu, queue->qcr, ctrl);
		dev_warn(iommu->dev,
			 "Queue #%u error; memory fault:%d overflow:%d\n",
			 queue->qid,
			 !!(ctrl & RISCV_IOMMU_PQCSR_PQMF),
			 !!(ctrl & RISCV_IOMMU_PQCSR_PQOF));
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
	return riscv_iommu_queue_send(&iommu->cmdq, &cmd, RISCV_IOMMU_INVAL_TIMEOUT);
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
	struct riscv_iommu_endpoint *endpoint;
	struct list_head bonds;
};

/* This struct contains protection domain specific IOMMU driver data. */
struct riscv_iommu_domain {
	struct iommu_domain domain;
	struct list_head bonds;
	int pscid;
	int pasid;
	int numa_node;
	int amo_enabled:1;
	unsigned int pgd_mode;
	/* paging domain */
	unsigned long pgd_root;
	/* SVA domain */
	struct mmu_notifier notifier;
};

#define iommu_domain_to_riscv(iommu_domain) \
	container_of(iommu_domain, struct riscv_iommu_domain, domain)

/* Send IOTLB.INVAL for whole address space for ranges larger than 2MB */
#define RISCV_IOMMU_IOTLB_INVAL_LIMIT	(512 * PAGE_SIZE)

static void riscv_iommu_iotlb_inval(struct riscv_iommu_domain *domain,
				    unsigned long start, unsigned long end)
{
	struct riscv_iommu_bond *bond;
	struct riscv_iommu_endpoint *ep;
	struct riscv_iommu_queue *cmdq;
	struct riscv_iommu_command cmd;
	unsigned long len = end - start + 1;
	unsigned long iova;

	list_for_each_entry(bond, &domain->bonds, bonds) {
		cmdq = &(dev_to_iommu(bond->endpoint->dev))->cmdq;
		ep = bond->endpoint;

		riscv_iommu_cmd_inval_vma(&cmd);
		riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);
		if (len > 0 && len < RISCV_IOMMU_IOTLB_INVAL_LIMIT) {
			for (iova = start; iova < end; iova += PAGE_SIZE) {
				riscv_iommu_cmd_inval_set_addr(&cmd, iova);
				riscv_iommu_queue_send(cmdq, &cmd, 0);
			}
		} else {
			riscv_iommu_queue_send(cmdq, &cmd, 0);
		}

		if (!ep->ats_enabled)
			continue;

		riscv_iommu_cmd_iofence(&cmd);
		riscv_iommu_queue_send(cmdq, &cmd, 0);

		riscv_iommu_cmd_ats_inval(&cmd);
		if (len)
			riscv_iommu_cmd_ats_set_range(&cmd, start, end, true);
		else
			riscv_iommu_cmd_ats_set_all(&cmd, true);
		riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
	}

	list_for_each_entry(bond, &domain->bonds, bonds) {
		cmdq = &(dev_to_iommu(bond->endpoint->dev))->cmdq;

		riscv_iommu_cmd_iofence(&cmd);
		riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_INVAL_TIMEOUT);
	}
}

/* TODO: add other PDT levels - autoexpand existing PDT if needed.
 * convert to API set_pc(iommu, devid, pasid, fsc, pscid) or similar...
 * allocation for DC should include ep->pasid_supported to construct PDT
 */
static struct riscv_iommu_pc *riscv_iommu_get_pc(struct riscv_iommu_device *iommu,
						 struct riscv_iommu_endpoint *ep, int pasid)
{
	struct riscv_iommu_dc *dc;
	unsigned long ptr;

	if (ep->pc)
		return ep->pc + pasid;

	dc = riscv_iommu_get_dc(iommu, ep->devid);
	if (!dc)
		return NULL;
	if (!ep->pasid_supported)
		return (struct riscv_iommu_pc *)(&dc->ta);
	/*
	 * If PASID is supported, prepare device context with process context tree root,
	 * enabled DPE and configure paging domain under PASID #0.
	 */
	ptr = riscv_iommu_get_pages(iommu, 0);
	if (!ptr)
		return NULL;

	dc->fsc = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_PDTP_MODE_PD8) |
		  FIELD_PREP(RISCV_IOMMU_DC_FSC_PPN, virt_to_pfn(ptr));
	dc->ta = 0;
	ep->pc = (struct riscv_iommu_pc *)ptr;

	return ep->pc + pasid;
}

/* Enable PCIe endpoint ATS/PASID/PRI features */
static void riscv_iommu_enable_pdev(struct pci_dev *pdev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(&pdev->dev);

	if (ep->pasid_supported)
		ep->pasid_enabled = !pci_enable_pasid(pdev, pci_pasid_features(pdev));
	if (ep->ats_supported)
		ep->ats_enabled = !pci_enable_ats(pdev, PAGE_SHIFT);
}

static void riscv_iommu_disable_pdev(struct pci_dev *pdev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(&pdev->dev);

	if (ep->ats_enabled) {
		pci_disable_ats(pdev);
		ep->ats_enabled = false;
	}

	if (ep->pasid_enabled) {
		pci_disable_pasid(pdev);
		ep->pasid_enabled = false;
	}
}

// lockdep_assert_held(&group->mutex);
static int riscv_iommu_attach_domain(struct device *dev,
				     struct iommu_domain *domain)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	const bool was_attached = ep->attached;
	bool amo_enabled = false;
	struct riscv_iommu_dc *dc;
	struct riscv_iommu_pc *pc;
	unsigned int pscid = 0;
	u64 atp, ta, tc;

	if (!domain && !was_attached)
		return 0;

	/* Find DC and PC for the endpoint */
	dc = riscv_iommu_get_dc(iommu, ep->devid);
	if (!dc)
		return -ENODEV;

	pc = riscv_iommu_get_pc(iommu, ep, 0);
	if (!pc)
		return -ENODEV;
	ta = READ_ONCE(pc->ta);
	tc = READ_ONCE(dc->tc);

	/* Invalidate last known valid PSCID */
	if ((tc & RISCV_IOMMU_DC_TC_V) &&
	    (!ep->pasid_supported || (ta & RISCV_IOMMU_PC_TA_V)))
		pscid = FIELD_GET(RISCV_IOMMU_DC_TA_PSCID, ta);

	if (!domain) {
		if (dev_is_pci(dev))
			riscv_iommu_disable_pdev(to_pci_dev(dev));
		WRITE_ONCE(dc->tc, 0);
		ep->attached = false;
	} else {
		ta = 0;
		if (domain->type == IOMMU_DOMAIN_IDENTITY) {
			atp = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_MODE_BARE);
		} else if (domain->type & __IOMMU_DOMAIN_PAGING) {
			struct riscv_iommu_domain *pd = iommu_domain_to_riscv(domain);

			atp = FIELD_PREP(RISCV_IOMMU_PC_FSC_MODE, pd->pgd_mode) |
			      FIELD_PREP(RISCV_IOMMU_PC_FSC_PPN, virt_to_pfn(pd->pgd_root));
			ta |= FIELD_PREP(RISCV_IOMMU_PC_TA_PSCID, pd->pscid);
		} else {
			return -ENODEV;
		}
		if (ep->pasid_supported)
			ta |= RISCV_IOMMU_PC_TA_V;
		WRITE_ONCE(pc->fsc, atp);
		/* Prevent incomplete PC state being observable */
		smp_wmb();
		WRITE_ONCE(pc->ta, ta);
	}

	if (!was_attached) {
		/* Configure 2nd stage translation to identity mapping */
		dc->iohgatp = FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_MODE,
					 RISCV_IOMMU_DC_IOHGATP_MODE_BARE);

		/* Configure translation context. */
		tc = RISCV_IOMMU_DC_TC_V;
		if (amo_enabled)
			tc |= RISCV_IOMMU_DC_TC_SADE;
		if (ep->ats_supported)
			tc |= RISCV_IOMMU_DC_TC_EN_ATS;
		if (ep->pri_supported)
			tc |= RISCV_IOMMU_DC_TC_EN_PRI;
		if (ep->pasid_supported)
			tc |= RISCV_IOMMU_DC_TC_DPE | RISCV_IOMMU_DC_TC_PDTV;

		/* Prevent incomplete DC/PC state being observable */
		smp_wmb();
		WRITE_ONCE(dc->tc, tc);

		/* mark as attached */
		ep->attached = true;

		if (dev_is_pci(dev))
			riscv_iommu_enable_pdev(to_pci_dev(dev));
	}

	if (was_attached) {
		struct riscv_iommu_queue *cmdq = &iommu->cmdq;
		struct riscv_iommu_command cmd;

		/* Invalidate device & process context cache */
		riscv_iommu_cmd_iodir_inval_ddt(&cmd);
		riscv_iommu_cmd_iodir_set_did(&cmd, ep->devid);
		if (ep->pasid_supported)
			riscv_iommu_cmd_iodir_set_pid(&cmd, 0);
		riscv_iommu_queue_send(cmdq, &cmd, 0);

		/* Invalidate address translation cache for previous domain */
		if (pscid) {
			riscv_iommu_cmd_inval_vma(&cmd);
			riscv_iommu_cmd_inval_set_pscid(&cmd, pscid);
			riscv_iommu_queue_send(cmdq, &cmd, 0);
		}

		/* Invalidate ATS */
		if (ep->ats_supported) {
			riscv_iommu_cmd_ats_inval(&cmd);
			riscv_iommu_cmd_ats_set_all(&cmd, true);
			riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
			riscv_iommu_queue_send(cmdq, &cmd, 0);
		}

		/* IOFENCE.C */
		riscv_iommu_cmd_iofence(&cmd);
		riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_INVAL_TIMEOUT);
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

	riscv_iommu_iotlb_inval(domain, 0, -1UL);
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

		/* TODO: deal with __old being a valid non-leaf entry */

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

		old = READ_ONCE(*ptr);
		if (cmpxchg_relaxed(ptr, old, 0) != old)
			continue;

		iommu_iotlb_gather_add_page(&domain->domain, gather, iova,
					    pte_size);

		iova = (iova & ~(pte_size - 1)) + pte_size;
		/* unmap unalligned IOVA ? */
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
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_bond *bond;
	struct page *page;
	int rc;

	if (!riscv_iommu_pt_supported(iommu, domain->pgd_mode))
		return -ENODEV;

	if (list_empty(&domain->bonds)) {
		domain->numa_node = dev_to_node(iommu->dev);
		domain->amo_enabled = !!(iommu->caps & RISCV_IOMMU_CAP_AMO);
	}

	if (!domain->pgd_root) {
		page = alloc_pages_node(domain->numa_node,
					GFP_KERNEL_ACCOUNT | __GFP_ZERO, 0);
		if (!page)
			return -ENOMEM;
		domain->pgd_root = (unsigned long)page_to_virt(page);
	}

	// here or at domain attach ...
	bond = kzalloc(sizeof(*bond), GFP_KERNEL_ACCOUNT);
	if (!bond)
		return -ENOMEM;
	INIT_LIST_HEAD(&bond->bonds);
	bond->endpoint = ep;

	rc = riscv_iommu_attach_domain(dev, iommu_domain);
	if (rc)
		return rc;

	list_add(&bond->bonds, &domain->bonds);

	return 0;
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

static void riscv_iommu_free_sva_domain(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if ((int)domain->pscid > 0)
		ida_free(&riscv_iommu_pscids, domain->pscid);

	if (domain->notifier.ops)
		mmu_notifier_unregister(&domain->notifier, iommu_domain->mm);

	kfree(domain);
}

static void riscv_iommu_mm_invalidate(struct mmu_notifier *mn,
				      struct mm_struct *mm,
				      unsigned long start,
				      unsigned long end)
{
	riscv_iommu_iotlb_inval(container_of((mn), struct riscv_iommu_domain, notifier),
				start, end);
}

static const struct mmu_notifier_ops riscv_iommu_mm_uops = {
	.arch_invalidate_secondary_tlbs = riscv_iommu_mm_invalidate,
};

static int riscv_iommu_set_dev_pasid(struct iommu_domain *iommu_domain,
				     struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_bond *bond;
	struct riscv_iommu_pc *pc;

	/* Process Context table should be set for pasid enabled endpoints. */
	if (!ep || !ep->pasid_enabled)
		return -ENODEV;

	pc = riscv_iommu_get_pc(dev_to_iommu(dev), ep, pasid);
	if (!pc)
		return -ENOMEM;

	bond = kzalloc(sizeof(*bond), GFP_KERNEL);
	if (!bond)
		return -ENOMEM;
	bond->endpoint = ep;
	INIT_LIST_HEAD(&bond->bonds);

	if (!domain->pasid)
		domain->pasid = pasid;
	else if (domain->pasid != pasid)
		return -ENODEV;

	/* register mm notifier */
	if (!domain->notifier.ops) {
		domain->notifier.ops = &riscv_iommu_mm_uops;
		if (mmu_notifier_register(&domain->notifier, iommu_domain->mm)) {
			dev_err(dev, "can't register notifier mn!\n");
			return -ENODEV;
		}
	}

	list_add(&bond->bonds, &domain->bonds);

	if (pc->ta & RISCV_IOMMU_PC_TA_V)
		dev_err(dev, "PASID %x already configured for device %x\n", pasid, ep->devid);

	pc->fsc = FIELD_PREP(RISCV_IOMMU_PC_FSC_MODE, domain->pgd_mode) |
		  FIELD_PREP(RISCV_IOMMU_PC_FSC_PPN, virt_to_pfn(iommu_domain->mm->pgd));
	pc->ta  = FIELD_PREP(RISCV_IOMMU_PC_TA_PSCID, domain->pscid) | RISCV_IOMMU_PC_TA_V;

	return 0;
}

static void riscv_iommu_remove_dev_pasid(struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_queue *cmdq = &iommu->cmdq;
	struct iommu_domain *iommu_domain = iommu_get_domain_for_dev_pasid(dev, pasid, 0);
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_pc *pc;
	struct riscv_iommu_command cmd;

	pc = riscv_iommu_get_pc(iommu, ep, pasid);
	if (!pc)
		return;

	pc->fsc     = 0;
	pc->ta      = 0;

	riscv_iommu_cmd_iodir_inval_pdt(&cmd);
	riscv_iommu_cmd_iodir_set_did(&cmd, ep->devid);
	riscv_iommu_cmd_iodir_set_pid(&cmd, pasid);
	riscv_iommu_queue_send(cmdq, &cmd, 0);

	riscv_iommu_iotlb_inval(domain, 0, -1UL);
}

static const struct iommu_domain_ops riscv_iommu_sva_domain_ops = {
	.set_dev_pasid = riscv_iommu_set_dev_pasid,
	.free = riscv_iommu_free_sva_domain,
};

static struct iommu_domain *riscv_iommu_domain_alloc(unsigned int type)
{
	struct iommu_domain_geometry *geometry;
	struct riscv_iommu_domain *domain;
	int pscid;

	if (type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_SVA &&
	    type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	pscid = ida_alloc_range(&riscv_iommu_pscids, 1,
				RISCV_IOMMU_MAX_PSCID - 1, GFP_KERNEL);
	if (pscid < 0)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain) {
		ida_free(&riscv_iommu_pscids, pscid);
		return NULL;
	}
	INIT_LIST_HEAD(&domain->bonds);

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
	geometry = &domain->domain.geometry;
	geometry->aperture_start = 0;
	geometry->aperture_end = DMA_BIT_MASK(VA_BITS - 1);
	geometry->force_aperture = true;

	/*
	 * Follow system address translation mode.
	 * RISC-V IOMMU ATP mode values match RISC-V CPU SATP mode values.
	 */
	domain->pgd_mode = satp_mode >> SATP_MODE_SHIFT;
	domain->numa_node = NUMA_NO_NODE;
	domain->pscid = pscid;
	if (type == IOMMU_DOMAIN_SVA)
		domain->domain.ops = &riscv_iommu_sva_domain_ops;
	else
		domain->domain.ops = &riscv_iommu_paging_domain_ops;

	return &domain->domain;
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
	return 0;
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

static int riscv_iommu_dev_enable_iopf(struct device *dev)
{
	struct pci_dev *pdev = dev_is_pci(dev) ? to_pci_dev(dev) : NULL;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	int rc;

	if (!pdev || !ep || !ep->pri_supported || !iommu->pq_work)
		return -ENODEV;

	if (ep->pri_enabled)
		return -EBUSY;

	ep->pri_pasid_required = pci_prg_resp_pasid_required(pdev);
	rc = pci_reset_pri(pdev);
	if (rc)
		return rc;

	rc = iopf_queue_add_device(iommu->pq_work, dev);
	if (rc)
		return rc;

	rc = iommu_register_device_fault_handler(dev, iommu_queue_iopf, dev);
	if (rc) {
		iopf_queue_remove_device(iommu->pq_work, dev);
		return rc;
	}

	rc = pci_enable_pri(pdev, 32);
	if (rc) {
		iommu_unregister_device_fault_handler(dev);
		iopf_queue_remove_device(iommu->pq_work, dev);
		return rc;
	}

	ep->pri_enabled = true;

	return 0;
}

static int riscv_iommu_dev_disable_iopf(struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!ep->pri_enabled)
		return -EINVAL;

	pci_disable_pri(to_pci_dev(dev));
	iommu_unregister_device_fault_handler(dev);
	iopf_queue_remove_device(iommu->pq_work, dev);

	ep->pri_enabled = false;

	return 0;
}

static int riscv_iommu_dev_enable_sva(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!ep || !ep->pasid_enabled || !ep->ats_enabled)
		return -EINVAL;

	if (!ep->pri_supported)
		return 0;

	if (!ep->pri_enabled)
		return -EINVAL;

	return 0;
}

static int riscv_iommu_dev_enable_feat(struct device *dev, enum iommu_dev_features feat)
{
	if (feat == IOMMU_DEV_FEAT_IOPF)
		return riscv_iommu_dev_enable_iopf(dev);
	if (feat == IOMMU_DEV_FEAT_SVA)
		return riscv_iommu_dev_enable_sva(dev);

	return -ENODEV;
}

static int riscv_iommu_dev_disable_feat(struct device *dev, enum iommu_dev_features feat)
{
	if (feat == IOMMU_DEV_FEAT_IOPF)
		return riscv_iommu_dev_disable_iopf(dev);
	if (feat == IOMMU_DEV_FEAT_SVA)
		return 0;

	return -ENODEV;
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
	RB_CLEAR_NODE(&ep->eps_node);

	if (pdev) {
		if (iommu->caps & RISCV_IOMMU_CAP_ATS)
			ep->ats_supported = pci_ats_supported(pdev);
		if (ep->ats_supported)
			ep->ats_queue_depth = pci_ats_queue_depth(pdev);
		if (ep->ats_supported && iommu->iommu.max_pasids)
			ep->pasid_supported = pci_pasid_features(pdev) >= 0;
		if (ep->pasid_supported)
			ep->max_pasid = pci_max_pasids(pdev);
		if (ep->max_pasid <= 0)
			ep->pasid_supported = false;
		if (ep->pasid_supported && ep->max_pasid < iommu->iommu.max_pasids)
			iommu->iommu.max_pasids = ep->max_pasid;
		if (ep->ats_supported && ep->pasid_supported)
			ep->pri_supported = pci_pri_supported(pdev);
	}

	dev_iommu_priv_set(dev, ep);

	return &iommu->iommu;
}

static void riscv_iommu_probe_finalize(struct device *dev)
{
	riscv_iommu_add_device(dev);
	iommu_setup_dma_ops(dev, 0, U64_MAX);
}

static void riscv_iommu_release_device(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	riscv_iommu_attach_domain(dev, NULL);
	riscv_iommu_del_device(dev);
	dev_iommu_priv_set(dev, NULL);
	kfree(ep);
}

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K | SZ_2M | SZ_1G,
	.of_xlate = riscv_iommu_of_xlate,
	.identity_domain = &riscv_iommu_identity_domain,
	.domain_alloc = riscv_iommu_domain_alloc,
	.def_domain_type = riscv_iommu_device_domain_type,
	.device_group = riscv_iommu_device_group,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
	.remove_dev_pasid = riscv_iommu_remove_dev_pasid,
	.dev_enable_feat = riscv_iommu_dev_enable_feat,
	.dev_disable_feat = riscv_iommu_dev_disable_feat,
	.page_response = riscv_iommu_page_response,
};

void riscv_iommu_remove(struct riscv_iommu_device *iommu)
{
	iommu_device_unregister(&iommu->iommu);
	iommu_device_sysfs_remove(&iommu->iommu);
	riscv_iommu_debugfs_remove(iommu);
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	riscv_iommu_queue_disable(&iommu->cmdq);
	riscv_iommu_queue_disable(&iommu->fltq);
	riscv_iommu_queue_disable(&iommu->priq);
	iopf_queue_free(iommu->pq_work);
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

	/* Check PASID capabilities */
	if (iommu->caps & RISCV_IOMMU_CAP_PD20)
		iommu->iommu.max_pasids = 1u << 20;
	else if (iommu->caps & RISCV_IOMMU_CAP_PD17)
		iommu->iommu.max_pasids = 1u << 17;
	else if (iommu->caps & RISCV_IOMMU_CAP_PD8)
		iommu->iommu.max_pasids = 1u << 8;

	dma_set_mask_and_coherent(iommu->dev,
				  DMA_BIT_MASK(FIELD_GET(RISCV_IOMMU_CAP_PAS, iommu->caps)));

	return 0;
}

int riscv_iommu_init(struct riscv_iommu_device *iommu)
{
	int rc;

	RISCV_IOMMU_QUEUE_INIT(&iommu->cmdq, CQ);
	RISCV_IOMMU_QUEUE_INIT(&iommu->fltq, FQ);
	RISCV_IOMMU_QUEUE_INIT(&iommu->priq, PQ);
	mutex_init(&iommu->eps_mutex);

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

	rc = riscv_iommu_queue_alloc(iommu, &iommu->priq, sizeof(struct riscv_iommu_pq_record));
	if (WARN(rc, "cannot allocate page request queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(&iommu->cmdq, riscv_iommu_cmdq_process);
	if (WARN(rc, "cannot enable command queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(&iommu->fltq, riscv_iommu_fltq_process);
	if (WARN(rc, "cannot enable fault queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(&iommu->priq, riscv_iommu_priq_process);
	if (WARN(rc, "cannot enable page request queue\n"))
		goto err_init;

	iommu->pq_work = iopf_queue_alloc(dev_name(iommu->dev));

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
