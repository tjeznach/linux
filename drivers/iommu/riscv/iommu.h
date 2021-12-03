/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright © 2021-2022, Rivos Inc.
 *
 * RISC-V Ziommu - IOMMU Interface Specification.
 *
 * Authors: Tomasz Jeznach <tjeznach@rivosinc.com>
 *
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

#include "iommu-bits.h"

struct riscv_iommu_queue {
	dma_addr_t base_dma;
	void *base;
	u32 len;		/* single item length */
	u32 cnt;		/* items count */
	u32 lui;		/* last used index, consumer/producer share */
	u32 qbr;		/* queue base register offset */
	u32 qcr;		/* queue control and status register offset */
	int irq;		/* registered interrupt number */
};

struct riscv_iommu_device {
	struct iommu_device iommu;	/* iommu core interface */
	struct device *dev;		/* iommu hardware */

	/* hardware control register space */
	void __iomem *reg;
	u64 reg_phys;
	u64 reg_size;

	/* supported and enabled hardware capabilities */
	u64 cap;

	/* global lock, to be removed */
	spinlock_t cq_lock;

	unsigned long zero;	/* shared zeroed page */
	unsigned long sync;	/* Notification page */
	unsigned long ddtp;	/* device directory table root pointer */
	unsigned ddt_mode;	/* device directory table mode */

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

	/* remove: could be a list of iommus */
	struct riscv_iommu_device *iommu;

	bool g_stage;				/* TODO: convert to domain-mode ? */
	struct riscv_iommu_domain *nested;	/* G-Stage protection domain if any */
	struct riscv_iommu_msipte *msi_root;	/* INT mapping */

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

int riscv_iommu_probe(struct device *dev, phys_addr_t reg_phys, size_t reg_size);
void riscv_iommu_remove(struct device *dev);

int riscv_iommu_sysfs_add(struct riscv_iommu_device *iommu);

#endif
