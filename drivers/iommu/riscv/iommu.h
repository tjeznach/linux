/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * RISC-V Ziommu - IOMMU Interface Specification.
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#ifndef _RISCV_IOMMU_H_
#define _RISCV_IOMMU_H_

#include <linux/types.h>
#include <linux/iova.h>
#include <linux/io.h>
#include <linux/idr.h>
#include <linux/mmu_notifier.h>
#include <linux/list.h>
#include <linux/iommu.h>
#include <linux/io-pgtable.h>
#include <linux/mmu_notifier.h>

#include "iommu-bits.h"

#define IOMMU_PAGE_SIZE_4K		BIT_ULL(12)
#define IOMMU_PAGE_SIZE_2M		BIT_ULL(21)
#define IOMMU_PAGE_SIZE_1G		BIT_ULL(30)
#define IOMMU_PAGE_SIZE_512G	BIT_ULL(39)

struct riscv_iommu_queue {
	dma_addr_t base_dma;
	void *base;
	size_t len;		/* single item length */
	u32 cnt;		/* items count */
	u32 lui;		/* last used index, consumer/producer share */
	unsigned qbr;		/* queue base register offset */
	unsigned qcr;		/* queue control and status register offset */
	int irq;		/* registered interrupt number */
	bool in_iomem;		/* indicates queue data are in I/O memory  */
};

enum riscv_queue_ids {
	RISCV_IOMMU_COMMAND_QUEUE	= 0,
	RISCV_IOMMU_FAULT_QUEUE		= 1,
	RISCV_IOMMU_PAGE_REQUEST_QUEUE	= 2
};

/* TODO: Readback icvec */
enum riscv_default_virqs {
	RISCV_IOMMU_INTR_CQ = 0,
	RISCV_IOMMU_INTR_FQ = 1,
	RISCV_IOMMU_INTR_PM = 2,
	RISCV_IOMMU_INTR_PQ = 3
};

struct riscv_iommu_device {
	struct iommu_device iommu;	/* iommu core interface */
	struct device *dev;		/* iommu hardware */

	/* hardware control register space */
	void __iomem *reg;
	u64 reg_phys;

	/* IRQs for the various queues */
	int irq_cmdq;
	int irq_fltq;
	int irq_pm;
	int irq_priq;

	/* Queue lengths */
	int cmdq_len;
	int fltq_len;
	int priq_len;

	/* supported and enabled hardware capabilities */
	u64 cap;

	/* global lock, to be removed */
	spinlock_t cq_lock;

	unsigned long zero;	/* shared zeroed page */
	unsigned long sync;	/* Notification page */
	unsigned long ddtp;	/* device directory table root pointer */
	unsigned ddt_mode;	/* device directory table mode */
	bool ddtp_in_iomem;

	/* I/O page fault queue */
	struct iopf_queue *pq_work;

	/* hardware ring buffers */
	struct riscv_iommu_queue cmdq;
	struct riscv_iommu_queue fltq;
	struct riscv_iommu_queue priq;

	/* Connected end-points */
	struct rb_root eps;
	struct mutex eps_mutex;
};

struct riscv_iommu_domain {
	struct iommu_domain domain;
	struct io_pgtable pgtbl;

	struct list_head endpoints;
	struct list_head notifiers;
	struct mutex lock;

	struct mmu_notifier mn;                 /* mmu_notifier handle */

	/* remove: could be a list of iommus */
	struct riscv_iommu_device *iommu;

	bool g_stage;				/* TODO: convert to domain-mode ? */
	struct riscv_iommu_domain *nested;	/* G-Stage protection domain if any */
	struct riscv_iommu_msi_pte *msi_root;	/* INT mapping */

	unsigned id;		/* GSCID or PSCID */
	unsigned mode;		/* RIO_ATP_MODE_* enum */
	ioasid_t pscid;		// this is a domain property

	pgd_t *pgd_root;	/* page table root pointer */
};

/* Private dev_iommu_priv object, device-domain relationship. */
struct riscv_iommu_endpoint {
	struct device *dev;			/* owned by a device $dev */
	unsigned devid;      			/* PCI bus:device:function number */
	unsigned domid;    			/* PCI domain number, segment */

	struct riscv_iommu_device *iommu;	/* -> iommu (virtual, collection of) */
	struct riscv_iommu_domain *domain;	/* -> attached domain, only one at a time, nesting via domain->domain */
	struct list_head domains;		/* -> collection of endpoints attached to the same domain */
	struct rb_node node;    		/* -> iommu-device lookup by devid */

	struct riscv_iommu_dc *dc;		/* -> device context pointer, can be tracked by iommu->dc(devid) */
	struct riscv_iommu_pc *pc;		/* -> process context root, can be tracked by iommu->dc(devid)->pc(pasid) */

	struct list_head regions;		// msi list
	struct list_head bindings;		// sva list

	/* end point info bits */
	unsigned pasid_bits;
	unsigned pasid_feat;
	unsigned pri_supported:1;
	unsigned sva_supported:1;
	unsigned pri_enabled:1;
	unsigned ats_enabled:1;
	unsigned pasid_enabled:1;

};

/* Helper functions and macros */

static inline u32 riscv_iommu_readl(struct riscv_iommu_device *iommu,
				    unsigned offset)
{
	return readl_relaxed(iommu->reg + offset);
}

static inline void riscv_iommu_writel(struct riscv_iommu_device *iommu,
				      unsigned offset, u32 val)
{
	writel_relaxed(val, iommu->reg + offset);
}

static inline u64 riscv_iommu_readq(struct riscv_iommu_device *iommu,
				    unsigned offset)
{
	return readq_relaxed(iommu->reg + offset);
}

static inline void riscv_iommu_writeq(struct riscv_iommu_device *iommu,
				      unsigned offset, u64 val)
{
	writeq_relaxed(val, iommu->reg + offset);
}

int riscv_iommu_init_common(struct riscv_iommu_device *iommu);
void riscv_iommu_remove(struct device *dev);

int riscv_iommu_sysfs_add(struct riscv_iommu_device *iommu);

#endif
