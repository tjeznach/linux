// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V architected Ziommu implementations.
 *
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/pci.h>
#include <linux/pci-ats.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/dma-map-ops.h>
#include <asm/page.h>

#include "../dma-iommu.h"
#include "../iommu-sva.h"
#include "iommu.h"

#include <asm/csr.h>
#include <asm/delay.h>

MODULE_DESCRIPTION("IOMMU driver for RISC-V architected Ziommu implementations");
MODULE_AUTHOR("Tomasz Jeznach <tjeznach@rivosinc.com>");
MODULE_AUTHOR("Nick Kossifidis <mick@ics.forth.gr>");
MODULE_ALIAS("riscv-iommu");
MODULE_LICENSE("GPL v2");

/* Global IOMMU params. */
static int ddt_mode = RISCV_IOMMU_DDTP_MODE_3LVL;
module_param(ddt_mode, int, 0644);
MODULE_PARM_DESC(ddt_mode, "Device Directory Table mode.");

static int cmdq_length = 1024;
module_param(cmdq_length, int, 0644);
MODULE_PARM_DESC(cmdq_length, "Command queue length.");

static int fltq_length = 1024;
module_param(fltq_length, int, 0644);
MODULE_PARM_DESC(fltq_length, "Fault queue length.");

static int priq_length = 1024;
module_param(priq_length, int, 0644);
MODULE_PARM_DESC(priq_length, "Page request interface queue length.");

/* TODO: Enable MSI remapping */
#define RISCV_IMSIC_BASE	0x28000000

/* 1 second */
#define RISCV_IOMMU_TIMEOUT		riscv_timebase

/* RISC-V IOMMU PPN <> PHYS address conversions, PHYS <=> PPN[53:10] */
#define phys_to_ppn(va)  (((va) >> 2) & (((1ULL << 44) - 1) << 10))
#define ppn_to_phys(pn)	 (((pn) << 2) & (((1ULL << 44) - 1) << 12))

#define iommu_domain_to_riscv(iommu_domain) \
    container_of(iommu_domain, struct riscv_iommu_domain, domain)

#define iommu_device_to_riscv(iommu_device) \
    container_of(iommu_device, struct riscv_iommu, iommu)

static const struct iommu_domain_ops riscv_iommu_domain_ops;
static const struct iommu_ops riscv_iommu_ops;

/**********************************\
* Common queue management routines *
\**********************************/

/* Note: offsets are the same for all queues */
#define Q_HEAD(q) ((q)->qbr + (RISCV_IOMMU_REG_CQH - RISCV_IOMMU_REG_CQB))
#define Q_TAIL(q) ((q)->qbr + (RISCV_IOMMU_REG_CQT - RISCV_IOMMU_REG_CQB))

static unsigned riscv_iommu_queue_consume(struct riscv_iommu_device *iommu,
					  struct riscv_iommu_queue *q,
					  unsigned *ready)
{
	u32 tail = riscv_iommu_readl(iommu, Q_TAIL(q));
	*ready = q->lui;

	BUG_ON(q->cnt <= tail);
	if (q->lui <= tail)
		return tail - q->lui;
	return q->cnt - q->lui;
}

static void riscv_iommu_queue_release(struct riscv_iommu_device *iommu,
				      struct riscv_iommu_queue *q,
				      unsigned count)
{
	q->lui = (q->lui + count) & (q->cnt - 1);
	riscv_iommu_writel(iommu, Q_HEAD(q), q->lui);
}

static u32 riscv_iommu_queue_ctrl(struct riscv_iommu_device *iommu,
				  struct riscv_iommu_queue *q, u32 val)
{
	cycles_t end_cycles = RISCV_IOMMU_TIMEOUT + get_cycles();

	riscv_iommu_writel(iommu, q->qcr, val);
	do {
		val = riscv_iommu_readl(iommu, q->qcr);
		if (!(val & RISCV_IOMMU_QUEUE_BUSY))
			break;
		cpu_relax();
	} while (get_cycles() < end_cycles);

	return val;
}

static void riscv_iommu_queue_free(struct riscv_iommu_device *iommu,
				   struct riscv_iommu_queue *q)
{
	size_t size = q->len * q->cnt;

	riscv_iommu_queue_ctrl(iommu, q, 0);

	if (q->base) {
		if (q->in_iomem)
			iounmap(q->base);
		else
			dmam_free_coherent(iommu->dev, size, q->base, q->base_dma);
	}
	if (q->irq)
		free_irq(q->irq, q);
}

static irqreturn_t riscv_iommu_cmdq_irq_check(int irq, void *data);
static irqreturn_t riscv_iommu_cmdq_process(int irq, void *data);
static irqreturn_t riscv_iommu_fltq_irq_check(int irq, void *data);
static irqreturn_t riscv_iommu_fltq_process(int irq, void *data);
static irqreturn_t riscv_iommu_priq_irq_check(int irq, void *data);
static irqreturn_t riscv_iommu_priq_process(int irq, void *data);

static int riscv_iommu_queue_init(struct riscv_iommu_device *iommu, int queue_id)
{
	struct device *dev = iommu->dev;
	struct riscv_iommu_queue *q = NULL;
	size_t queue_size = 0;
	irq_handler_t irq_check;
	irq_handler_t irq_process;
	const char* name;
	int count = 0;
	int irq = 0;
	unsigned order = 0;
	u64 qbr_val = 0;
	u64 qbr_readback = 0;
	u64 qbr_paddr = 0;
	int ret = 0;

	switch (queue_id) {
	case RISCV_IOMMU_COMMAND_QUEUE:
		q = &iommu->cmdq;
		q->len = sizeof(struct riscv_iommu_command);
		count = iommu->cmdq_len;
		irq = iommu->irq_cmdq;
		irq_check = riscv_iommu_cmdq_irq_check;
		irq_process = riscv_iommu_cmdq_process;
		q->qbr = RISCV_IOMMU_REG_CQB;
		q->qcr = RISCV_IOMMU_REG_CQCSR;
		name = "cmdq";
		break;
	case RISCV_IOMMU_FAULT_QUEUE:
		q = &iommu->fltq;
		q->len = sizeof(struct riscv_iommu_fq_record);
		count = iommu->fltq_len;
		irq = iommu->irq_fltq;
		irq_check = riscv_iommu_fltq_irq_check;
		irq_process = riscv_iommu_fltq_process;
		q->qbr = RISCV_IOMMU_REG_FQB;
		q->qcr = RISCV_IOMMU_REG_FQCSR;
		name = "fltq";
		break;
	case RISCV_IOMMU_PAGE_REQUEST_QUEUE:
		q = &iommu->priq;
		q->len  = sizeof(struct riscv_iommu_pq_record);
		count = iommu->priq_len;
		irq = iommu->irq_priq;
		irq_check = riscv_iommu_priq_irq_check;
		irq_process = riscv_iommu_priq_process;
		q->qbr = RISCV_IOMMU_REG_PQB;
		q->qcr = RISCV_IOMMU_REG_PQCSR;
		name = "priq";
		break;
	default:
		dev_err(dev, "invalid queue interrupt index in queue_init!\n");
		return -EINVAL;
	}

	/* Polling not implemented */
	if (!irq)
		return -ENODEV;

	/* Allocate queue in memory and set the base register */
	order = ilog2(count);
	do {
		queue_size = q->len * (1ULL << order);
		q->base = dmam_alloc_coherent(dev, queue_size, &q->base_dma,
					      GFP_KERNEL);
		if (q->base || queue_size < PAGE_SIZE)
			break;

		order--;
	} while (1);

	if (!q->base) {
		dev_err(dev, "failed to allocate %s queue (cnt: %u)\n",
			name, count);
		return -ENOMEM;
	}

	q->cnt = 1ULL << order;

	qbr_val = phys_to_ppn(q->base_dma) |
		  FIELD_PREP(RISCV_IOMMU_QUEUE_SIZE_FIELD, order - 1);

	riscv_iommu_writeq(iommu, q->qbr, qbr_val);

	/*
	 * Queue base registers are WARL, so it's possible that whatever we wrote
	 * there was illegal/not supported by the hw in which case we need to make
	 * sure we set a supported PPN and/or queue size.
	 */
	qbr_readback = riscv_iommu_readq(iommu, q->qbr);
	if (qbr_readback == qbr_val)
		goto irq;

	dmam_free_coherent(dev, queue_size, q->base, q->base_dma);

	/* Get supported queue size */
	order = FIELD_GET(RISCV_IOMMU_QUEUE_SIZE_FIELD, qbr_readback) + 1;
	q->cnt = 1ULL << order;
	queue_size = q->len * q->cnt;	

	/*
	 * In case we also failed to set PPN, it means the field is hardcoded and the
	 * queue resides in I/O memory instead, so get its physical address and
	 * ioremap it.
	 */
	qbr_paddr = ppn_to_phys(qbr_readback);
	if (qbr_paddr != q->base_dma) {
		dev_info(dev, "hardcoded ppn in %s base register, using io memory for the queue\n",
			 name);
		dev_info(dev, "queue length for %s set to %i\n", name, q->cnt);
	 	q->in_iomem = true;
		q->base = ioremap(qbr_paddr, queue_size);
		if (!q->base) {
			dev_err(dev, "failed to map %s queue (cnt: %u)\n",
				name, q->cnt);
			return -ENOMEM;
		}
		q->base_dma = qbr_paddr;
	} else {
		/*
		 * We only failed to set the queue size, re-try to allocate memory with
		 * the queue size supported by the hw.
		 */
		dev_info(dev, "hardcoded queue size in %s base register\n", name);
		dev_info(dev, "retrying with queue length: %i\n", q->cnt);
		q->base = dmam_alloc_coherent(dev, queue_size, &q->base_dma,
					      GFP_KERNEL);
		if (!q->base) {
			dev_err(dev, "failed to allocate %s queue (cnt: %u)\n",
				name, q->cnt);
			return -ENOMEM;
		}
	}

	qbr_val = phys_to_ppn(q->base_dma) |
		  FIELD_PREP(RISCV_IOMMU_QUEUE_SIZE_FIELD, order - 1);
	riscv_iommu_writeq(iommu, q->qbr, qbr_val);

	/* Final check to make sure hw accepted our write */
	qbr_readback = riscv_iommu_readq(iommu, q->qbr);
	if (qbr_readback != qbr_val) {
		dev_err(dev, "failed to set base register for %s\n", name);
		goto fail;	
	}

 irq:	 
	if (request_threaded_irq(irq, irq_check, irq_process, IRQF_ONESHOT | IRQF_SHARED,
				 dev_name(dev), q)) {
		dev_err(dev, "fail to request irq %d for %s\n", irq, name);
		goto fail;
	}

	q->irq = irq;

	/* Note: All RIO_xQ_EN/IE fields are in the same offsets */
	ret = riscv_iommu_queue_ctrl(iommu, q, RISCV_IOMMU_QUEUE_ENABLE | RISCV_IOMMU_QUEUE_INTR_ENABLE);
	if (ret & RISCV_IOMMU_QUEUE_BUSY) {
		dev_err(dev, "%s init timeout\n", name);
		ret = -EBUSY;
		goto fail;
	}

	return 0;

 fail:
	riscv_iommu_queue_free(iommu, q);
	return 0;
}


/***********************************\
* I/O MMU Command queue chapter 3.1 *
\***********************************/

static inline void riscv_iommu_cmd_iodir_pasid(struct riscv_iommu_command *cmd,
						unsigned devid, unsigned pasid)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_IODIR_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_IODIR_FUNC_INVAL_PDT) |
		      FIELD_PREP(RISCV_IOMMU_CMD_IODIR_DID, devid) | RISCV_IOMMU_CMD_IODIR_DV |
		      FIELD_PREP(RISCV_IOMMU_CMD_IODIR_PID, pasid);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_inval_vma(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_IOTINVAL_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_IOTINVAL_FUNC_VMA);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_inval_set_addr(struct riscv_iommu_command *cmd, u64 addr)
{
	cmd->dword0 |= RISCV_IOMMU_CMD_IOTINVAL_AV;
	cmd->dword1 = addr;
}

static inline void riscv_iommu_cmd_inval_set_pscid(struct riscv_iommu_command *cmd,
						   unsigned pscid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_PSCID, pscid) |
			RISCV_IOMMU_CMD_IOTINVAL_PSCV;
}

static inline void riscv_iommu_cmd_inval_set_gscid(struct riscv_iommu_command *cmd,
						   unsigned gscid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_GSCID, gscid) |
			RISCV_IOMMU_CMD_IOTINVAL_GV;
}

static inline void riscv_iommu_cmd_iofence(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_IOFENCE_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_IOFENCE_FUNC_C);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_iofence_set_av(struct riscv_iommu_command *cmd, u64 addr,
				 u32 data)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_IOFENCE_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_IOFENCE_FUNC_C) |
		      FIELD_PREP(RISCV_IOMMU_CMD_IOFENCE_DATA, data) | RISCV_IOMMU_CMD_IOFENCE_AV;
	cmd->dword1 = (addr >> 2);
}

static inline void riscv_iommu_cmd_ats_pgr(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_ATS_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_ATS_FUNC_PRGR);
	cmd->dword1 = 0;
}

/* TODO: Convert into lock-less MPSC implementation. */
static bool riscv_iommu_post_sync(struct riscv_iommu_device *iommu,
				  struct riscv_iommu_command *cmd, bool sync)
{
	u32 head, tail, next, last;
	unsigned long flags;


	spin_lock_irqsave(&iommu->cq_lock, flags);
	head = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_CQH) & (iommu->cmdq.cnt - 1);
	tail = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_CQT) & (iommu->cmdq.cnt - 1);
	last = iommu->cmdq.lui;
	if (tail != last) {
		spin_unlock_irqrestore(&iommu->cq_lock, flags);
		/* TRY AGAIN */
		dev_err(iommu->dev, "IOMMU CQT: %x != %x (1st)\n", last, tail);
		spin_lock_irqsave(&iommu->cq_lock, flags);
		tail = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_CQT) & (iommu->cmdq.cnt - 1);
		last = iommu->cmdq.lui;
		if (tail != last) {
			spin_unlock_irqrestore(&iommu->cq_lock, flags);
			dev_err(iommu->dev, "IOMMU CQT: %x != %x (2nd)\n", last, tail);
			spin_lock_irqsave(&iommu->cq_lock, flags);
		}
	}

	next = (last + 1) & (iommu->cmdq.cnt - 1);
	if (next != head) {
		struct riscv_iommu_command *ptr = iommu->cmdq.base;
		ptr[last] = *cmd;
		wmb();
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_CQT, next);
		iommu->cmdq.lui = next;
	}

	spin_unlock_irqrestore(&iommu->cq_lock, flags);

	if (sync && head != next) {
		cycles_t start_time = get_cycles();
		while (1) {
			last = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_CQH) &
			       (iommu->cmdq.cnt - 1);
			if (head < next && last >= next)
				break;
			if (head > next && last < head && last >= next)
				break;
			if (RISCV_IOMMU_TIMEOUT < (get_cycles() - start_time)) {
				dev_err(iommu->dev, "IOFENCE TIMEOUT\n");
				return false;
			}
			cpu_relax();
		}
	}

	return next != head;
}

static bool riscv_iommu_post(struct riscv_iommu_device *iommu,
			     struct riscv_iommu_command *cmd)
{
	return riscv_iommu_post_sync(iommu, cmd, false);
}

static bool riscv_iommu_iodir_inv_all(struct riscv_iommu_device *iommu)
{
	struct riscv_iommu_command cmd = {
		.dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_IODIR_OPCODE) |
			  FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_IODIR_FUNC_INVAL_DDT),
		.dword1 = 0,
	};
	return riscv_iommu_post(iommu, &cmd);
}

static bool riscv_iommu_iodir_inv_devid(struct riscv_iommu_device *iommu,
					unsigned devid)
{
	struct riscv_iommu_command cmd = {
		.dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_IODIR_OPCODE) |
			  FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_IODIR_FUNC_INVAL_DDT) |
			  FIELD_PREP(RISCV_IOMMU_CMD_IODIR_DID, devid) | RISCV_IOMMU_CMD_IODIR_DV,
		.dword1 = 0,
	};
	return riscv_iommu_post(iommu, &cmd);
}

static bool riscv_iommu_iodir_inv_pasid(struct riscv_iommu_device *iommu,
					unsigned devid, unsigned pasid)
{
	struct riscv_iommu_command cmd;
	riscv_iommu_cmd_iodir_pasid(&cmd, devid, pasid);
	return riscv_iommu_post(iommu, &cmd);
}

static bool riscv_iommu_iofence_sync(struct riscv_iommu_device *iommu)
{
	struct riscv_iommu_command cmd;
	riscv_iommu_cmd_iofence(&cmd);
	return riscv_iommu_post_sync(iommu, &cmd, true);
}

static void riscv_iommu_mm_invalidate(struct mmu_notifier *mn,
    struct mm_struct *mm, unsigned long start, unsigned long end)
{
	struct riscv_iommu_command cmd;
	struct riscv_iommu_domain *domain = container_of(mn, struct riscv_iommu_domain, mn);
	/* TODO: add ATS.INVAL if needed, cleanup GSCID/PSCID passing, IOVA range flush */
	riscv_iommu_cmd_inval_vma(&cmd);
	riscv_iommu_cmd_inval_set_gscid(&cmd, 0);
	riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);
	riscv_iommu_post(domain->iommu, &cmd);
	riscv_iommu_iofence_sync(domain->iommu);
}


static void riscv_iommu_mm_release(struct mmu_notifier *mn, struct mm_struct *mm)
{
	/* TODO: removed from notifier, cleanup PSCID mapping, flush IOTLB */
}

static const struct mmu_notifier_ops riscv_iommu_mmuops = {
	.release = riscv_iommu_mm_release,
	.invalidate_range = riscv_iommu_mm_invalidate,
};

/* Command queue primary interrupt handler */
static irqreturn_t riscv_iommu_cmdq_irq_check(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu = container_of(q, struct riscv_iommu_device, cmdq);
	u32 ipsr = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_IPSR);
	if (ipsr & RISCV_IOMMU_IPSR_CIP)
		return IRQ_WAKE_THREAD;
	return IRQ_NONE;
}

/* Command queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_cmdq_process(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu;
	unsigned ctrl;

	iommu = container_of(q, struct riscv_iommu_device, cmdq);

	/* Error reporting, clear error reports if any. */
	ctrl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_CQCSR);
	if (ctrl & (RISCV_IOMMU_CQCSR_CQMF |
		    RISCV_IOMMU_CQCSR_CMD_TO |
		    RISCV_IOMMU_CQCSR_CMD_ILL)) {
		riscv_iommu_queue_ctrl(iommu, &iommu->cmdq, ctrl);
		dev_warn_ratelimited(iommu->dev,
			"Command queue error: fault: %d tout: %d err: %d\n",
			!!(ctrl & RISCV_IOMMU_CQCSR_CQMF),
			!!(ctrl & RISCV_IOMMU_CQCSR_CMD_TO),
			!!(ctrl & RISCV_IOMMU_CQCSR_CMD_ILL));
	}

	/* Clear fault interrupt pending. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, RISCV_IOMMU_IPSR_CIP);

	return IRQ_HANDLED;
}


/********************************\
* Fault/event queue, chapter 3.2 *
\********************************/

static void riscv_iommu_fault_report(struct riscv_iommu_device *iommu,
				     struct riscv_iommu_fq_record *event)
{
	unsigned err, devid;

	err = FIELD_GET(RISCV_IOMMU_FQ_HDR_CAUSE, event->hdr);
	devid = FIELD_GET(RISCV_IOMMU_FQ_HDR_DID, event->hdr);

	dev_warn_ratelimited(iommu->dev, "Fault %d devid: %d" " iotval: %llx iotval2: %llx\n",
			     err, devid, event->iotval, event->iotval2);
}

/* Fault/event queue primary interrupt handler */
static irqreturn_t riscv_iommu_fltq_irq_check(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu = container_of(q, struct riscv_iommu_device, fltq);
	u32 ipsr = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_IPSR);
	if (ipsr & RISCV_IOMMU_IPSR_FIP)
		return IRQ_WAKE_THREAD;
	return IRQ_NONE;
}

/* Fault queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_fltq_process(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_fq_record *events;
	unsigned cnt, len, idx, ctrl;

	iommu = container_of(q, struct riscv_iommu_device, fltq);
	events = (struct riscv_iommu_fq_record *)q->base;

	/* Error reporting, clear error reports if any. */
	ctrl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FQCSR);
	if (ctrl & (RISCV_IOMMU_FQCSR_FQMF | RISCV_IOMMU_FQCSR_FWOF)) {
		riscv_iommu_queue_ctrl(iommu, &iommu->fltq, ctrl);
		dev_warn_ratelimited(iommu->dev,
			"Fault queue error: fault: %d full: %d\n",
			!!(ctrl & RISCV_IOMMU_FQCSR_FQMF),
			!!(ctrl & RISCV_IOMMU_FQCSR_FWOF));
	}

	/* Clear fault interrupt pending. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, RISCV_IOMMU_IPSR_FIP);

	/* Report fault events. */
	do {
		cnt = riscv_iommu_queue_consume(iommu, q, &idx);
		if (!cnt)
			break;
		for (len = 0; len < cnt; idx++, len++)
			riscv_iommu_fault_report(iommu, &events[idx]);
		riscv_iommu_queue_release(iommu, q, cnt);
	} while (1);

	return IRQ_HANDLED;
}


/*********************************\
* Page request queue, chapter 3.3 *
\*********************************/

static struct riscv_iommu_endpoint *
riscv_iommu_find_ep(struct riscv_iommu_device *iommu, unsigned devid)
{
	struct rb_node *node;
	struct riscv_iommu_endpoint *ep;

	lockdep_assert_held(&iommu->eps_mutex);

	node = iommu->eps.rb_node;
	while (node) {
		ep = rb_entry(node, struct riscv_iommu_endpoint, node);
		if (ep->devid < devid)
			node = node->rb_right;
		else if (ep->devid > devid)
			node = node->rb_left;
		else
			return ep;
	}

	return NULL;
}

static void riscv_iommu_page_request(struct riscv_iommu_device *iommu,
				     struct riscv_iommu_pq_record *req)
{
	struct iommu_fault_event event = { 0 };
	struct iommu_fault_page_request *prm = &event.fault.prm;
	struct riscv_iommu_endpoint *ep;

	/* Ignore PGR Stop marker. */
	if ((req->payload & RISCV_IOMMU_PREQ_PAYLOAD_M) == RISCV_IOMMU_PREQ_PAYLOAD_L)
		return;

	event.fault.type = IOMMU_FAULT_PAGE_REQ;

	mutex_lock(&iommu->eps_mutex);
	ep = riscv_iommu_find_ep(iommu, FIELD_GET(RISCV_IOMMU_PREQ_HDR_DID, req->hdr));
	if (!ep) {
		/* TODO: Handle invalid page request */
		mutex_unlock(&iommu->eps_mutex);
		return;
	}

	if (req->payload & RISCV_IOMMU_PREQ_PAYLOAD_L)
		prm->flags |= IOMMU_FAULT_PAGE_REQUEST_LAST_PAGE;
	if (req->payload & RISCV_IOMMU_PREQ_PAYLOAD_W)
		prm->perm |= IOMMU_FAULT_PERM_WRITE;
	if (req->payload & RISCV_IOMMU_PREQ_PAYLOAD_R)
		prm->perm |= IOMMU_FAULT_PERM_READ;

	prm->grpid = FIELD_GET(RISCV_IOMMU_PREQ_PRG_INDEX, req->payload);
	prm->addr = FIELD_GET(RISCV_IOMMU_PREQ_UADDR, req->payload);

	if (req->hdr & RISCV_IOMMU_PREQ_HDR_PV) {
		prm->flags |= IOMMU_FAULT_PAGE_REQUEST_PASID_VALID;
		/* TODO: where to find this bit */
		prm->flags |= IOMMU_FAULT_PAGE_RESPONSE_NEEDS_PASID;
		prm->pasid = FIELD_GET(RISCV_IOMMU_PREQ_HDR_PID, req->hdr);
	}

	iommu_report_device_fault(ep->dev, &event);
	mutex_unlock(&iommu->eps_mutex);
}

static int riscv_iommu_page_response(struct device *dev,
				     struct iommu_fault_event *evt,
				     struct iommu_page_response *msg)
{

	dev_info(dev, "page response received. code: %d.\n", msg->code);
	/* TODO: post PRGR message to the IOMMU */
	return 0;
}

/* Page request interface queue primary interrupt handler */
static irqreturn_t riscv_iommu_priq_irq_check(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu = container_of(q, struct riscv_iommu_device, priq);
	u32 ipsr = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_IPSR);
	if (ipsr & RISCV_IOMMU_IPSR_PIP)
		return IRQ_WAKE_THREAD;
	return IRQ_NONE;
}

/* Page request interface queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_priq_process(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_pq_record *requests;
	unsigned cnt, len, idx, ctrl;

	iommu = container_of(q, struct riscv_iommu_device, priq);
	requests = (struct riscv_iommu_pq_record *)q->base;

	/* Error reporting, clear error reports if any. */
	ctrl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_PQCSR);
	if (ctrl & (RISCV_IOMMU_PQCSR_PQMF | RISCV_IOMMU_PQCSR_PQOF)) {
		riscv_iommu_queue_ctrl(iommu, &iommu->priq, ctrl);
		dev_warn_ratelimited(iommu->dev,
			"Page request queue error: fault: %d full: %d\n",
			!!(ctrl & RISCV_IOMMU_PQCSR_PQMF),
			!!(ctrl & RISCV_IOMMU_PQCSR_PQOF));
	}

	/* Clear page request interrupt pending. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, RISCV_IOMMU_IPSR_PIP);

	/* Process page requests. */
	do {
		cnt = riscv_iommu_queue_consume(iommu, q, &idx);
		if (!cnt)
			break;
		for (len = 0; len < cnt; idx++, len++)
			riscv_iommu_page_request(iommu, &requests[idx]);
		riscv_iommu_queue_release(iommu, q, cnt);
	} while (1);

	return IRQ_HANDLED;
}


/*********************\
* Endpoint management *
\*********************/

/* Endpoint features/capabilities */
static void riscv_iommu_disable_pci_ep(struct device *dev)
{
	struct pci_dev *pdev;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!dev_is_pci(dev))
		return;

	pdev = to_pci_dev(dev);
	if (pdev->ats_enabled)
		pci_disable_ats(pdev);

	if (pdev->pri_enabled)
		pci_disable_pri(pdev);

	if (pdev->pasid_enabled)
		pci_disable_pasid(pdev);

	ep->pasid_bits = 0;
}

static void riscv_iommu_enable_pci_ep(struct device *dev)
{
	int ret, feat, num;
	struct pci_dev *pdev;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!dev_is_pci(dev))
		return;

	pdev = to_pci_dev(dev);

	/* Enable PASID */
	feat = pci_pasid_features(pdev);
	num = pci_max_pasids(pdev);
	ret = pci_enable_pasid(pdev, feat);
	if (ret) {
		dev_warn(dev, "Can't enable PASID (rc: %d) cap: %u\n", ret, pdev->pasid_cap);
		return;
	}

	/* Reset the PRI state of the device */
	ret = pci_reset_pri(pdev);
	if (ret) {
		dev_warn(dev, "Can't reset PRI (rc: %d) en: %d\n", ret, pdev->pri_enabled);
		pci_disable_pasid(pdev);
		return;
	}

	/* Enable PRI */
	ret = pci_enable_pri(pdev, 32);
	if (ret) {
		dev_warn(dev, "Can't enable PRI (rc: %d)\n", ret);
		pci_disable_pasid(pdev);
		return;
	}

	/* Enable ATS */
	ret = pci_enable_ats(pdev, PAGE_SHIFT);
	if (ret) {
		dev_warn(dev, "Can't enable ATS (rc: %d)\n", ret);
		pci_disable_pri(pdev);
		pci_disable_pasid(pdev);
		return;
	}

	ep->sva_supported = 1;
	ep->pri_supported = 1;
	ep->pri_enabled = 1;
	ep->ats_enabled = 1;
	ep->pasid_enabled = 1;

	ep->pasid_feat = feat;
	ep->pasid_bits = ilog2(num);

	dev_info(dev, "PASID/ATS support enabled, %d bits\n", ep->pasid_bits);
}

static int riscv_iommu_enable_sva(struct device *dev)
{
	int ret;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (!ep || !ep->iommu || !ep->iommu->pq_work)
		return -EINVAL;

	if (!ep->sva_supported)
		return -ENODEV;

	if (!ep->pasid_enabled || !ep->pri_enabled || !ep->ats_enabled)
		return -EINVAL;

	ret = iopf_queue_add_device(ep->iommu->pq_work, dev);
	if (ret)
		return ret;

	return iommu_register_device_fault_handler(dev, iommu_queue_iopf, dev);
}

static int riscv_iommu_disable_sva(struct device *dev)
{
	int ret;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	ret = iommu_unregister_device_fault_handler(dev);
	if (!ret)
		ret = iopf_queue_remove_device(ep->iommu->pq_work, dev);

	return ret;
}

static int riscv_iommu_enable_iopf(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (ep && ep->pri_supported)
		return 0;

	return -EINVAL;
}

static int riscv_iommu_dev_enable_feat(struct device *dev,
				       enum iommu_dev_features feat)
{
	switch (feat) {
	case IOMMU_DEV_FEAT_IOPF:
		return riscv_iommu_enable_iopf(dev);

	case IOMMU_DEV_FEAT_SVA:
		return riscv_iommu_enable_sva(dev);

	default:
		return -ENODEV;
	}
}

static int riscv_iommu_dev_disable_feat(struct device *dev,
					enum iommu_dev_features feat)
{
	switch (feat) {
	case IOMMU_DEV_FEAT_IOPF:
		return 0;

	case IOMMU_DEV_FEAT_SVA:
		return riscv_iommu_disable_sva(dev);

	default:
		return -ENODEV;
	}
}

static int
riscv_iommu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static bool riscv_iommu_capable(struct device *dev, enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
	case IOMMU_CAP_PRE_BOOT_PROTECTION:
		return true;

	default:
		break;
	}

	return false;
}

/// TODO: transition to DC management calls, teardown
/* Lookup or initialize device directory info structure. */
static struct riscv_iommu_dc *riscv_iommu_get_dc(struct riscv_iommu_device
						 *iommu, u32 devid)
{
	const bool base_format = !(iommu->cap & RISCV_IOMMU_CAP_MSI_FLAT);
	unsigned depth = iommu->ddt_mode - RISCV_IOMMU_DDTP_MODE_1LVL;
	u8 ddi_bits[3] = {0};
	u64 *ddt = NULL;

	if (iommu->ddt_mode == RISCV_IOMMU_DDTP_MODE_OFF ||
	    iommu->ddt_mode == RISCV_IOMMU_DDTP_MODE_BARE)
		return NULL;

	/* Make sure the mode is valid */
	if (iommu->ddt_mode > RISCV_IOMMU_DDTP_MODE_MAX)
		return NULL;

	/*
	 * Device id partitioning for base format:
	 * DDI[0]: bits 0 - 6	(1st level) (7 bits)
	 * DDI[1]: bits 7 - 15	(2nd level) (9 bits)
	 * DDI[2]: bits 16 - 23	(3rd level) (8 bits)
	 *
	 * For extended format:
	 * DDI[0]: bits 0 - 5	(1st level) (6 bits)
	 * DDI[1]: bits 6 - 14	(2nd level) (9 bits)
	 * DDI[2]: bits 15 - 23	(3rd level) (9 bits)
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
	if (devid >= (1 << ddi_bits[depth]))
		return NULL;

	/* Get to the level of the non-leaf node that holds the device context */
	for (ddt = (u64 *) iommu->ddtp; depth-- > 0;) {
		const int split = ddi_bits[depth];
		/*
		 * Each non-leaf node is 64bits wide and on each level
		 * nodes are indexed by DDI[depth].
		 */
		ddt += (devid >> split) & 0x1FF;

		/*
		 * Check if this node has been populated and if not
		 * allocate a new level and populate it.
		 */
		if (*ddt & RISCV_IOMMU_DDTE_VALID) {
			ddt = __va(ppn_to_phys(*ddt));
		} else {
			// TODO: DDTP page walk management
			unsigned long ddtp = get_zeroed_page(GFP_KERNEL);
			if (!ddtp)
				return NULL;
			*ddt = phys_to_ppn(__pa(ddtp)) | RISCV_IOMMU_DDTE_VALID;
			ddt = (u64 *) ddtp;
		}
	}

	/*
	 * Grab the node that matches DDI[depth], note that when using base
	 * format the device context is 4 * 64bits, and the extended format
	 * is 8 * 64bits, hence the (3 - base_format) below.
	 */
	ddt += (devid & ((64 << base_format) - 1)) << (3 - base_format);
	return (struct riscv_iommu_dc *)ddt;
}

static struct riscv_iommu_device *riscv_iommu_get_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec || fwspec->ops != &riscv_iommu_ops ||
	    !fwspec->iommu_fwnode || !fwspec->iommu_fwnode->dev)
		return NULL;

	return dev_get_drvdata(fwspec->iommu_fwnode->dev);
}

static struct iommu_device *riscv_iommu_probe_device(struct device *dev)
{
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_endpoint *ep, *rb_ep;
	struct rb_node **new_node, *parent_node = NULL;
	struct pci_dev *pdev = dev_is_pci(dev) ? to_pci_dev(dev) : NULL;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	iommu = riscv_iommu_get_device(dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	ep = kzalloc(sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return ERR_PTR(-ENOMEM);

	ep->dev = dev;
	ep->iommu = iommu;
	if (pdev) {
		ep->devid = pci_dev_id(pdev);
		ep->domid = pci_domain_nr(pdev->bus);
	} else {
		/* TODO: Make this generic, for now hardcode domain id to 0 */
		ep->devid = fwspec->ids[0];
		ep->domid = 0;
	}

	dev_info(iommu->dev, "adding device to iommu with devid %i in domain %i\n", ep->devid, ep->domid);

	INIT_LIST_HEAD(&ep->domains);
	INIT_LIST_HEAD(&ep->regions);
	INIT_LIST_HEAD(&ep->bindings);

	/* insert into IOMMU endpoint mappings */
	mutex_lock(&iommu->eps_mutex);
	new_node = &(iommu->eps.rb_node);
	while (*new_node) {
		rb_ep = rb_entry(*new_node, struct riscv_iommu_endpoint, node);
		parent_node = *new_node;
		if (rb_ep->devid > ep->devid) {
			new_node = &((*new_node)->rb_left);
		} else if (rb_ep->devid < ep->devid) {
			new_node = &((*new_node)->rb_right);
		} else {
			dev_warn(dev, "device %u already in the tree\n", rb_ep->devid);
			break;
		}
	}

	rb_link_node(&ep->node, parent_node, new_node);
	rb_insert_color(&ep->node, &iommu->eps);
	mutex_unlock(&iommu->eps_mutex);

	dev_iommu_priv_set(dev, ep);

	return &iommu->iommu;
}

static void riscv_iommu_probe_finalize(struct device *dev)
{
	set_dma_ops(dev, NULL);
	iommu_setup_dma_ops(dev, 0, U64_MAX);
}

static void riscv_iommu_release_device(struct device *dev)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_device *iommu = ep->iommu;

	dev_info(dev, "device with devid %i released\n", ep->devid);

	/* Device must be already removed from protection domain */
	WARN_ON(ep->domain);

	riscv_iommu_disable_pci_ep(dev);
	ep->dc->tc = 0ULL;
	// ep->dc->fsc = cpu_to_le64(virt_to_pfn(ep->iommu->zero) | SATP_MODE);
	ep->dc->iohgatp = 0ULL;
	wmb();
	riscv_iommu_iodir_inv_devid(iommu, ep->devid);

	mutex_lock(&iommu->eps_mutex);
	rb_erase(&ep->node, &iommu->eps);
	mutex_unlock(&iommu->eps_mutex);

	set_dma_ops(dev, NULL);
	dev_iommu_priv_set(dev, NULL);

	kfree(ep);
}

static struct iommu_group *riscv_iommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	return generic_device_group(dev);
}

static void riscv_iommu_get_resv_regions(struct device *dev,
					 struct list_head *head)
{
	struct iommu_resv_region *entry, *new_entry;
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	list_for_each_entry(entry, &ep->regions, list) {
		new_entry = kmemdup(entry, sizeof(*entry), GFP_KERNEL);
		if (new_entry)
			list_add_tail(&new_entry->list, head);
	}

	iommu_dma_get_resv_regions(dev, head);
}



/*******************\
* Domain management *
\*******************/

/* TODO: move to local (private) allocators for VMID / PSCID
   allocate GSCID unique PSCID
 */
static ioasid_t iommu_sva_alloc_pscid(void)
{
	/* TODO: Provide anonymous pasid value */
	struct mm_struct mm = {
		.pasid = INVALID_IOASID,
	};

	if (iommu_sva_alloc_pasid(&mm, 1, (1 << 20) - 1))
		return INVALID_IOASID;

	return mm.pasid;
}

static struct iommu_domain *riscv_iommu_domain_alloc(unsigned type)
{
	struct riscv_iommu_domain *domain;

	if (type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_DMA_FQ &&
	    type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_IDENTITY &&
	    type != IOMMU_DOMAIN_BLOCKED &&
	    type != IOMMU_DOMAIN_SVA)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	/* based on domain type ?? */
	domain->domain.ops = &riscv_iommu_domain_ops;

	mutex_init(&domain->lock);
	INIT_LIST_HEAD(&domain->endpoints);

	return &domain->domain;
}

static void riscv_iommu_domain_free(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->mn.ops && iommu_domain->mm)
		mmu_notifier_unregister(&domain->mn, iommu_domain->mm);

	if (domain->pgtbl.cookie)
		free_io_pgtable_ops(&domain->pgtbl.ops);

	if (domain->pgd_root)
		free_pages((unsigned long)domain->pgd_root,
			   domain->g_stage ? 2 : 0);

	kfree(domain);
}

static int riscv_iommu_domain_finalize(struct riscv_iommu_domain *domain,
				       struct riscv_iommu_device *iommu)
{
	struct iommu_domain_geometry *geometry;

	/* Domain assigned to another iommu */
	if (domain->iommu && domain->iommu != iommu)
		return -EINVAL;
	/* Domain already initialized */
	else if (domain->iommu)
		return 0;

	/*
	 * TODO: Before using VA_BITS and satp_mode here, verify they
	 * are supported by the iommu, through the capabilities register.
	 */
	
	geometry = &domain->domain.geometry;
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
	geometry->aperture_start = 0;
	geometry->aperture_end = DMA_BIT_MASK(VA_BITS - 1);
	geometry->force_aperture = true;

	domain->iommu = iommu;
	domain->id = iommu_sva_alloc_pscid();
	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY) {
		domain->mode = RISCV_IOMMU_DC_FSC_MODE_BARE;
		return 0;
	}

	/* XXX: Fix this for RV32 */
	domain->mode = satp_mode >> 60;
	domain->pgd_root = (pgd_t *) __get_free_pages(GFP_KERNEL | __GFP_ZERO,
						      domain->g_stage ? 2 : 0);

	if (!domain->pgd_root)
		return -ENOMEM;

	if (!alloc_io_pgtable_ops(RISCV_IOMMU, &domain->pgtbl.cfg, domain))
		return -ENOMEM;

	return 0;
}

static u64 riscv_iommu_domain_atp(struct riscv_iommu_domain *domain)
{
	u64 atp = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, domain->mode);
	if (domain->mode != RISCV_IOMMU_DC_FSC_MODE_BARE)
		atp |= FIELD_PREP(RISCV_IOMMU_DC_FSC_PPN, virt_to_pfn(domain->pgd_root));
	if (domain->g_stage)
		atp |= FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_GSCID, domain->id);
	return atp;
}

static int riscv_iommu_attach_dev(struct iommu_domain *iommu_domain,
				  struct device *dev)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_dc *dc;
	struct iommu_resv_region *entry;
	int ret;
	u64 val;
	int i;

	dc = riscv_iommu_get_dc(ep->iommu, ep->devid);
	if (!dc)
		return -ENOMEM;

	// TODO: add endpoint lock ?
	mutex_lock(&domain->lock);

	/* allocate root pages, initialize io-pgtable ops, etc. */
	ret = riscv_iommu_domain_finalize(domain, ep->iommu);
	if (ret < 0) {
		dev_err(dev, "can not finalize domain: %d\n", ret);
		mutex_unlock(&domain->lock);
		return ret;
	}

	dev_info(dev, "domain id %u %s type %d attach\n", domain->id,
		 domain->g_stage ? "G-Stage" : "S-Stage", domain->domain.type);

	/* Update device context:
	   G-Stage : only if domain->g_stage
	   S-Stage : only if domain->s_stage. rely on OS not to update S-Stage while G-Stage valid.
	 */
	if (domain->g_stage) {
		dc->iohgatp = cpu_to_le64(riscv_iommu_domain_atp(domain));
		// trace S-Stage
		dev_info(dev, "IOMMU first-stage: %llx", dc->fsc);
		dc->fsc = RISCV_IOMMU_DC_FSC_MODE_BARE;	// pass-through for now.
	} else {
		val = FIELD_PREP(RISCV_IOMMU_DC_TA_PSCID, domain->id);
		dc->ta = cpu_to_le64(val);
		dc->fsc = cpu_to_le64(riscv_iommu_domain_atp(domain));
		wmb();
	}

	/* Initialize MSI remapping */
	if (!(ep->iommu->cap & RISCV_IOMMU_CAP_MSI_FLAT))
		goto skip_msiptp;

	/* FIXME: implement remapping device */
	val = get_zeroed_page(GFP_KERNEL);
	if (!val) {
		mutex_unlock(&domain->lock);
		return -ENOMEM;
	}

	domain->msi_root = (struct riscv_iommu_msi_pte *)val;

	for (i = 0; i < 256; i++) {
		domain->msi_root[i].pte =
		    pte_val(pfn_pte
			    (phys_to_pfn(RISCV_IMSIC_BASE) + i,
			     __pgprot(_PAGE_WRITE | _PAGE_PRESENT)));
	}

	entry = iommu_alloc_resv_region(RISCV_IMSIC_BASE, PAGE_SIZE * 256, 0,
					IOMMU_RESV_SW_MSI, GFP_KERNEL);
	if (entry) {
		list_add_tail(&entry->list, &ep->regions);
	}

	val = virt_to_pfn(domain->msi_root) |
	      FIELD_PREP(RISCV_IOMMU_DC_MSIPTP_MODE, RISCV_IOMMU_DC_MSIPTP_MODE_FLAT);
	dc->msiptp = cpu_to_le64(val);

	/* Single page of MSIPTP, 256 IMSIC files */
	dc->msi_addr_mask = cpu_to_le64(255);
	dc->msi_addr_pattern = cpu_to_le64(RISCV_IMSIC_BASE >> 12);

 skip_msiptp:
	/* Mark device context as valid */
	wmb();
	if (ep->ats_enabled)
		dc->tc = cpu_to_le64(RISCV_IOMMU_DC_TC_EN_ATS | RISCV_IOMMU_DC_TC_V);
	else
		dc->tc = cpu_to_le64(RISCV_IOMMU_DC_TC_V);
	ep->dc = dc;
	mutex_unlock(&domain->lock);
	riscv_iommu_iodir_inv_devid(ep->iommu, ep->devid);

	// enable PCI capabilities
	riscv_iommu_enable_pci_ep(dev);

	return 0;
}

static int riscv_iommu_set_dev_pasid(struct iommu_domain *iommu_domain,
				     struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_dc *dc = ep->dc;
	struct riscv_iommu_pc *pc = ep->pc;
	struct mm_struct *mm;

	if (!iommu_domain || !iommu_domain->mm)
		return -EINVAL;
	if (!ep || !ep->sva_supported)
		return -ENODEV;
	if (!pc)
		pc = (struct riscv_iommu_pc *)get_zeroed_page(GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	/* register mm notifier */
	mm = iommu_domain->mm;
	domain->pscid = pasid;
	domain->iommu = ep->iommu;
	domain->mn.ops = &riscv_iommu_mmuops;

	if (mmu_notifier_register(&domain->mn, mm))
	    return -ENODEV;

	/* Use PASID for PSCID tag */
	pc[pasid].ta = cpu_to_le64(FIELD_PREP(RISCV_IOMMU_PC_TA_PSCID, pasid) |
		       RISCV_IOMMU_PC_TA_V);
	/* TODO: get SXL value for the process, use 32 bit or SATP mode */
	pc[pasid].fsc = cpu_to_le64(virt_to_pfn(mm->pgd) | satp_mode);
	/* update DC with sva->mm */
	if (!(ep->dc->tc & RISCV_IOMMU_DC_TC_PDTV)) {
		/* migrate to PD, domain mappings moved to PASID:0 */
		pc[0].ta = dc->ta;
		pc[0].fsc = dc->fsc;
		dc->fsc = cpu_to_le64(virt_to_pfn(pc) |
			  FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_PDTP_MODE_PD8));
		/* XXX: What is (1ULL << 32) ? it's in the custom region */
		/* TODO: EN_ATS only for eps that support ATS */
		dc->tc = cpu_to_le64(RISCV_IOMMU_DC_TC_PDTV |
			 RISCV_IOMMU_DC_TC_EN_ATS | RISCV_IOMMU_DC_TC_V | (1ULL << 32));
		ep->pc = pc;
		wmb();

		/* TODO: transition to PD steps */
		riscv_iommu_iodir_inv_devid(ep->iommu, ep->devid);
	} else {
		wmb();
		riscv_iommu_iodir_inv_pasid(ep->iommu, ep->devid, pasid);
	}

	riscv_iommu_iofence_sync(ep->iommu);

	return 0;
}

static void riscv_iommu_remove_dev_pasid(struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_command cmd;

	/* remove SVA iommu-domain */

	/* invalidate TA.V */
	ep->pc[pasid].ta = 0;
	wmb();

	/* 1. invalidate PDT entry */
	riscv_iommu_cmd_iodir_pasid(&cmd, ep->devid, pasid);
	riscv_iommu_post(ep->iommu, &cmd);

	/* 2. invalidate all matching IOATC entries */
	riscv_iommu_cmd_inval_vma(&cmd);
	riscv_iommu_cmd_inval_set_gscid(&cmd, 0);
	riscv_iommu_cmd_inval_set_pscid(&cmd, pasid);
	riscv_iommu_post(ep->iommu, &cmd);

	/* 3. Wait IOATC flush to happen */
	riscv_iommu_iofence_sync(ep->iommu);
}

/* mark domain as second-stage translation */
static int riscv_iommu_enable_nesting(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	mutex_lock(&domain->lock);
	if (list_empty(&domain->endpoints))
		domain->g_stage = true;
	mutex_unlock(&domain->lock);

	return domain->g_stage ? 0 : -EBUSY;
}

static int riscv_iommu_map_pages(struct iommu_domain *iommu_domain,
				 unsigned long iova, phys_addr_t phys,
				 size_t pgsize, size_t pgcount, int prot,
				 gfp_t gfp, size_t *mapped)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_BLOCKED)
		return -ENODEV;

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY) {
		*mapped = pgsize * pgcount;
		return 0;
	}

	if (!domain->pgtbl.ops.map_pages)
		return -ENODEV;

	return domain->pgtbl.ops.map_pages(&domain->pgtbl.ops, iova, phys,
					   pgsize, pgcount, prot, gfp, mapped);
}

static size_t riscv_iommu_unmap_pages(struct iommu_domain *iommu_domain,
				      unsigned long iova, size_t pgsize,
				      size_t pgcount,
				      struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return pgsize * pgcount;

	if (!domain->pgtbl.ops.unmap_pages)
		return 0;

	return domain->pgtbl.ops.unmap_pages(&domain->pgtbl.ops, iova, pgsize,
					     pgcount, gather);
}

static void riscv_iommu_flush_iotlb_all(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_command cmd;

	riscv_iommu_cmd_inval_vma(&cmd);

	if (!domain->g_stage)
		riscv_iommu_cmd_inval_set_pscid(&cmd, domain->id);
	if (domain->g_stage)
		riscv_iommu_cmd_inval_set_gscid(&cmd, domain->id);
	else if (domain->nested)
		riscv_iommu_cmd_inval_set_gscid(&cmd, domain->nested->id);

	riscv_iommu_post(domain->iommu, &cmd);
	riscv_iommu_iofence_sync(domain->iommu);
}

static void riscv_iommu_iotlb_sync(struct iommu_domain *iommu_domain,
				   struct iommu_iotlb_gather *gather)
{
	riscv_iommu_flush_iotlb_all(iommu_domain);
}

static void riscv_iommu_iotlb_sync_map(struct iommu_domain *iommu_domain,
				       unsigned long iova, size_t size)
{
	riscv_iommu_flush_iotlb_all(iommu_domain);
}

static phys_addr_t riscv_iommu_iova_to_phys(struct iommu_domain *iommu_domain,
					    dma_addr_t iova)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return (phys_addr_t) iova;

	if (!domain->pgtbl.ops.iova_to_phys)
		return 0;

	return domain->pgtbl.ops.iova_to_phys(&domain->pgtbl.ops, iova);
}


/************************\
* Translation mode setup *
\************************/

static u64 riscv_iommu_get_ddtp(struct riscv_iommu_device *iommu)
{
	u64 ddtp;
	cycles_t end_cycles = RISCV_IOMMU_TIMEOUT + get_cycles();

	/* Wait for DDTP.BUSY to be cleared and return latest value */
	do {
		ddtp = riscv_iommu_readq(iommu, RISCV_IOMMU_REG_DDTP);
		if (!(ddtp & RISCV_IOMMU_DDTP_BUSY))
			break;
		cpu_relax();
	} while (get_cycles() < end_cycles);

	return ddtp;
}

static void riscv_iommu_ddt_cleanup(struct riscv_iommu_device *iommu)
{
	if (iommu->ddtp) {
		// TODO: teardown whole tree.
		if (iommu->ddtp_in_iomem) {
			iounmap((void*) iommu->ddtp);
		} else
			free_page(iommu->ddtp);
		iommu->ddtp = 0;
	}
}

static int riscv_iommu_enable(struct riscv_iommu_device *iommu, unsigned requested_mode)
{
	struct device *dev = iommu->dev;
	u64 ddtp = 0;
	u64 ddtp_paddr = 0;
	unsigned mode = requested_mode;
	unsigned mode_readback = 0;

	ddtp = riscv_iommu_get_ddtp(iommu);
	if (ddtp & RISCV_IOMMU_DDTP_BUSY)
		return -EBUSY;

	/* Disallow state transtion from xLVL to xLVL. */
	switch (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp)) {
	case RISCV_IOMMU_DDTP_MODE_BARE:
	case RISCV_IOMMU_DDTP_MODE_OFF:
		break;
	default:
		if ((mode != RISCV_IOMMU_DDTP_MODE_BARE) && (mode != RISCV_IOMMU_DDTP_MODE_OFF))
			return -EINVAL;
		break;
	}

 retry:
	switch (mode) {
	case RISCV_IOMMU_DDTP_MODE_BARE:
	case RISCV_IOMMU_DDTP_MODE_OFF:
		riscv_iommu_ddt_cleanup(iommu);
		ddtp = FIELD_PREP(RISCV_IOMMU_DDTP_MODE, mode);
		break;
	case RISCV_IOMMU_DDTP_MODE_1LVL:
	case RISCV_IOMMU_DDTP_MODE_2LVL:
	case RISCV_IOMMU_DDTP_MODE_3LVL:
		if (!iommu->ddtp) {
			/*
			 * We haven't initialized ddtp yet, since it's WARL make
			 * sure that we don't have a hardwired PPN field there
			 * that points to i/o memory instead.
			 */
			riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP, 0);
			ddtp = riscv_iommu_get_ddtp(iommu);
			ddtp_paddr = ppn_to_phys(ddtp);
			if (ddtp_paddr) {
				dev_warn(dev, "ddtp at 0x%llx\n", ddtp_paddr);
				iommu->ddtp = (unsigned long) ioremap(ddtp_paddr, PAGE_SIZE);
				iommu->ddtp_in_iomem = true;
			} else {
				iommu->ddtp = get_zeroed_page(GFP_KERNEL);
			}
		}
		if (!iommu->ddtp)
			return -ENOMEM;

		ddtp = FIELD_PREP(RISCV_IOMMU_DDTP_MODE, mode) |
			phys_to_ppn(__pa(iommu->ddtp));

		break;
	default:
		return -EINVAL;
	}

	riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP, ddtp);
	ddtp = riscv_iommu_get_ddtp(iommu);
	if (ddtp & RISCV_IOMMU_DDTP_BUSY) {
		dev_warn(dev, "timeout when setting ddtp (ddt mode: %i)\n", mode);
		/* XXX: Should we cleanup ddt ? set iommu->ddt_mode and hope for the best ? */
		return -EBUSY;
	}

	mode_readback = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);
	dev_info(dev, "mode_readback: %i, mode: %i\n", mode_readback, mode);
	if (mode_readback != mode) {
		/*
		 * Mode field is WARL, an I/O MMU may support a subset of
		 * directory table levels in which case if we tried to set
		 * an unsupported number of levels we'll readback either
		 * a valid xLVL or off/bare. If we got off/bare, try again
		 * with a smaller xLVL.
		 */
		if (mode_readback < RISCV_IOMMU_DDTP_MODE_1LVL &&
		    mode > RISCV_IOMMU_DDTP_MODE_1LVL) {
			mode--;
			goto retry;
		}

		/*
		 * We tried all supported xLVL modes and still got off/bare instead,
		 * an I/O MMU must support at least one supported xLVL mode so something
		 * went very wrong.
		 */
		if (mode_readback < RISCV_IOMMU_DDTP_MODE_1LVL &&
		    mode == RISCV_IOMMU_DDTP_MODE_1LVL)
			goto fail;

		/*
		 * We tried setting off or bare and got something else back, something
		 * went very wrong since off/bare is always legal.
		 */
		if (mode < RISCV_IOMMU_DDTP_MODE_1LVL)
			goto fail;

		/*
		 * We tried setting an xLVL mode but got another xLVL mode that
		 * we don't support (e.g. a custom one).
		 */
		if (mode_readback > RISCV_IOMMU_DDTP_MODE_MAX)
			goto fail;		

		/* We tried setting an xLVL mode but got another supported xLVL mode */
		mode = mode_readback;
	}

	if (mode != requested_mode)
		dev_warn(dev, "unsupported DDT mode requested (%i), using %i instead\n",
			 requested_mode, mode);

	iommu->ddt_mode = mode;
	dev_info(dev, "ddt_mode: %i\n", iommu->ddt_mode);
	return 0;

 fail:
	dev_err(dev, "failed to set DDT mode, tried: %i and got %i\n", mode, mode_readback);
	riscv_iommu_ddt_cleanup(iommu);
	return -EINVAL;
}


/**************************************\
* Common I/O MMU driver probe/teardown *
\**************************************/


static const struct iommu_domain_ops riscv_iommu_domain_ops = {
	.free = riscv_iommu_domain_free,
	.attach_dev = riscv_iommu_attach_dev,
	.set_dev_pasid = riscv_iommu_set_dev_pasid,
	.map_pages = riscv_iommu_map_pages,
	.unmap_pages = riscv_iommu_unmap_pages,
	.iova_to_phys = riscv_iommu_iova_to_phys,
	.iotlb_sync = riscv_iommu_iotlb_sync,
	.iotlb_sync_map = riscv_iommu_iotlb_sync_map,
	.flush_iotlb_all = riscv_iommu_flush_iotlb_all,
	.enable_nesting = riscv_iommu_enable_nesting,
};

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K,
	.capable = riscv_iommu_capable,
	.domain_alloc = riscv_iommu_domain_alloc,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
	.remove_dev_pasid = riscv_iommu_remove_dev_pasid,
	.device_group = riscv_iommu_device_group,
	.get_resv_regions = riscv_iommu_get_resv_regions,
	.of_xlate = riscv_iommu_of_xlate,
	.dev_enable_feat = riscv_iommu_dev_enable_feat,
	.dev_disable_feat = riscv_iommu_dev_disable_feat,
	.page_response = riscv_iommu_page_response,
	.default_domain_ops = &riscv_iommu_domain_ops,
};

void riscv_iommu_remove(struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_get_drvdata(dev);

	iommu_device_unregister(&iommu->iommu);
	iommu_device_sysfs_remove(&iommu->iommu);
	riscv_iommu_enable(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	riscv_iommu_queue_free(iommu, &iommu->cmdq);
	riscv_iommu_queue_free(iommu, &iommu->fltq);
	riscv_iommu_queue_free(iommu, &iommu->priq);
	iopf_queue_free(iommu->pq_work);
	kfree(iommu);
}

int riscv_iommu_init_common(struct riscv_iommu_device *iommu)
{
	struct device *dev = iommu->dev;
	u32 fctl = 0;
	int ret;

	iommu->eps = RB_ROOT;

	/* TODO: Version check */

	fctl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FCTL);
#ifdef CONFIG_CPU_BIG_ENDIAN
	if (!(cap & RISCV_IOMMU_CAP_END)) {
		dev_err(dev, "IOMMU doesn't support Big Endian\n");
		return -EIO;
	} else if (!(fctl & RISCV_IOMMU_FCTL_BE) {
		fctl |= FIELD_PREP(RISCV_IOMMU_FCTL_BE, 1);
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_FCTL, fctl);
	}
#endif

	if (iommu->cap & RISCV_IOMMU_CAP_PD20)
		iommu->iommu.max_pasids = 1u << 20;
	else if (iommu->cap & RISCV_IOMMU_CAP_PD17)
		iommu->iommu.max_pasids = 1u << 17;
	else if (iommu->cap & RISCV_IOMMU_CAP_PD8)
		iommu->iommu.max_pasids = 1u << 8;


	/*
	 * Assign queue lengths from module parameters if not already
	 * set on the device tree.
	 */
	if (!iommu->cmdq_len)
		iommu->cmdq_len = cmdq_length;
	if (!iommu->fltq_len)
		iommu->fltq_len = fltq_length;
	if (!iommu->priq_len)
		iommu->priq_len = priq_length;

	/* Clear any pending interrupt flag. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR,
			   	  RISCV_IOMMU_IPSR_CIP |
			   	  RISCV_IOMMU_IPSR_FIP |
			   	  RISCV_IOMMU_IPSR_PMIP |
			   	  RISCV_IOMMU_IPSR_PIP);

	spin_lock_init(&iommu->cq_lock);
	mutex_init(&iommu->eps_mutex);

	ret = riscv_iommu_queue_init(iommu, RISCV_IOMMU_COMMAND_QUEUE);
	if (ret)
		goto fail;

	ret = riscv_iommu_queue_init(iommu, RISCV_IOMMU_FAULT_QUEUE);
	if (ret)
		goto fail;

	if (!(iommu->cap & RISCV_IOMMU_CAP_ATS))
		goto no_ats;

	/* PRI functionally depends on ATS’s capabilities. */
	iommu->pq_work = iopf_queue_alloc(dev_name(dev));
	if (!iommu->pq_work) {
		dev_err(dev, "failed to allocate iopf queue\n");
		ret = -ENOMEM;
		goto fail;
	}

	ret = riscv_iommu_queue_init(iommu, RISCV_IOMMU_PAGE_REQUEST_QUEUE);
	if (ret)
		goto fail;

 no_ats:
	if (iommu_default_passthrough()) {
		dev_info(dev, "iommu set to passthrough mode\n");
		ret = riscv_iommu_enable(iommu, RISCV_IOMMU_DDTP_MODE_BARE);
	} else
		ret = riscv_iommu_enable(iommu, ddt_mode);
	if (ret) {
		dev_err(dev, "cannot enable iommu device (%d)\n", ret);
		goto fail;
	}

	ret = riscv_iommu_sysfs_add(iommu);
	if (ret) {
		dev_err(dev, "cannot register sysfs interface (%d)\n", ret);
		goto fail;
	}

	ret = iommu_device_register(&iommu->iommu, &riscv_iommu_ops, dev);
	if (ret) {
		dev_err(dev, "cannot register iommu interface (%d)\n", ret);
		iommu_device_sysfs_remove(&iommu->iommu);
		goto fail;
	}

	dev_set_drvdata(dev, iommu);

	return 0;

 fail:
	riscv_iommu_enable(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	riscv_iommu_queue_free(iommu, &iommu->priq);
	riscv_iommu_queue_free(iommu, &iommu->fltq);
	riscv_iommu_queue_free(iommu, &iommu->cmdq);
	iopf_queue_free(iommu->pq_work);
	return ret;
}
