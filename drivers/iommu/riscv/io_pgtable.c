// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright (C) 2022 Rivos Inc.
 * Author: Tomasz Jeznach <tjeznach@rivosinc.com>
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/io-pgtable.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>

#include "iommu.h"

#define io_pgtable_ops_to_domain(x) \
	container_of(container_of((x), struct io_pgtable, ops), \
			     struct riscv_iommu_domain, pgtbl)

static void riscv_iommu_free_pgtable(struct io_pgtable *iop)
{
	printk("io-pgtable for riscv released\n");
}

static pte_t *riscv_iommu_pgd_walk(struct riscv_iommu_domain *domain,
				   unsigned long iova,
				   unsigned long (*pd_alloc)(gfp_t), gfp_t gfp)
{
	/* TODO: merge dev/iopgtable */
	pgd_t *pgd;
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;
	unsigned long pfn;

	pgd = pgd_offset_pgd(domain->pgd_root, iova);
	if (pgd_none(*pgd)) {
		pfn = pd_alloc ? virt_to_pfn(pd_alloc(gfp)) : 0;
		if (!pfn)
			return NULL;
		set_pgd(pgd, pfn_pgd(pfn, __pgprot(_PAGE_TABLE)));
	}

	p4d = p4d_offset(pgd, iova);
	if (p4d_none(*p4d)) {
		pfn = pd_alloc ? virt_to_pfn(pd_alloc(gfp)) : 0;
		if (!pfn)
			return NULL;
		set_p4d(p4d, __p4d((pfn << _PAGE_PFN_SHIFT) | _PAGE_TABLE));
	}

	pud = pud_offset(p4d, iova);
	if (pud_none(*pud)) {
		pfn = pd_alloc ? virt_to_pfn(pd_alloc(gfp)) : 0;
		if (!pfn)
			return NULL;
		set_pud(pud, __pud((pfn << _PAGE_PFN_SHIFT) | _PAGE_TABLE));
	}

	pmd = pmd_offset(pud, iova);
	if (pmd_none(*pmd)) {
		pfn = pd_alloc ? virt_to_pfn(pd_alloc(gfp)) : 0;
		if (!pfn)
			return NULL;
		set_pmd(pmd, __pmd((pfn << _PAGE_PFN_SHIFT) | _PAGE_TABLE));
	}

	return pte_offset_kernel(pmd, iova);
}

static int riscv_iommu_map_pages(struct io_pgtable_ops *ops,
				 unsigned long iova, phys_addr_t phys,
				 size_t pgsize, size_t pgcount, int prot,
				 gfp_t gfp, size_t *mapped)
{
	struct riscv_iommu_domain *domain = io_pgtable_ops_to_domain(ops);
	size_t size = 0;
	pte_t *pte;
	pte_t pte_val;
	pgprot_t pte_prot;

	if (domain->domain.type == IOMMU_DOMAIN_BLOCKED)
		return -ENODEV;

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY) {
		// TODO: should we be here ?
		*mapped = pgsize * pgcount;
		return 0;
	}

	if (pgsize != PAGE_SIZE) {
		return -EIO;
	}

	pte_prot = (prot & IOMMU_WRITE) ?
		__pgprot(_PAGE_BASE | _PAGE_READ | _PAGE_WRITE | _PAGE_DIRTY) :
		__pgprot(_PAGE_BASE | _PAGE_READ);

	while (pgcount--) {
		pte = riscv_iommu_pgd_walk(domain, iova, get_zeroed_page, gfp);
		if (!pte) {
			*mapped = size;
			return -ENOMEM;
		}

		pte_val = pfn_pte(phys_to_pfn(phys), pte_prot);

		set_pte(pte, pte_val);

		size += PAGE_SIZE;
		iova += PAGE_SIZE;
		phys += PAGE_SIZE;
	}

	*mapped = size;
	return 0;
}

static size_t riscv_iommu_unmap_pages(struct io_pgtable_ops *ops,
				      unsigned long iova, size_t pgsize,
				      size_t pgcount,
				      struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = io_pgtable_ops_to_domain(ops);
	size_t size = 0;
	pte_t *pte;

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return pgsize * pgcount;

	if (pgsize != PAGE_SIZE) {
		return -EIO;
	}

	while (pgcount--) {
		pte = riscv_iommu_pgd_walk(domain, iova, NULL, 0);
		if (!pte)
			return size;

		// release pages
		set_pte(pte, __pte(0));

		size += PAGE_SIZE;
		iova += PAGE_SIZE;
	}

	return size;
}

static phys_addr_t riscv_iommu_iova_to_phys(struct io_pgtable_ops *ops,
					    unsigned long iova)
{
	struct riscv_iommu_domain *domain = io_pgtable_ops_to_domain(ops);
	pte_t *pte;

	if (domain->domain.type == IOMMU_DOMAIN_IDENTITY)
		return (phys_addr_t) iova;

	pte = riscv_iommu_pgd_walk(domain, iova, NULL, 0);
	if (!pte || !pte_present(*pte))
		return 0;

	return (pfn_to_phys(pte_pfn(*pte)) | (iova & PAGE_MASK));
}

static void riscv_iommu_tlb_inv_all(void *cookie)
{
	printk("IOMMU TLB INVAL ALL\n");
}

static void riscv_iommu_tlb_inv_walk(unsigned long iova, size_t size,
				     size_t granule, void *cookie)
{
	printk("IOMMU TLB INVAL\n");
}

static void riscv_iommu_tlb_add_page(struct iommu_iotlb_gather *gather,
				     unsigned long iova, size_t granule,
				     void *cookie)
{
	printk("IOMMU TLB ADD PAGE\n");
}

static const struct iommu_flush_ops riscv_iommu_flush_ops = {
	.tlb_flush_all	= riscv_iommu_tlb_inv_all,
	.tlb_flush_walk	= riscv_iommu_tlb_inv_walk,
	.tlb_add_page	= riscv_iommu_tlb_add_page,
};

/* NOTE: cfg should point to riscv_iommu_domain structure member pgtbl.cfg */
static struct io_pgtable *riscv_iommu_alloc_pgtable(struct io_pgtable_cfg *cfg,
						    void *cookie)
{
//	struct riscv_iommu_domain *domain = cookie;
	struct io_pgtable *iop = container_of(cfg, struct io_pgtable, cfg);

	cfg->pgsize_bitmap = SZ_4K,
	cfg->ias = 57;	// va mode, SvXX -> ias
	cfg->oas = 57;	// pa mode, or SvXX+4 -> oas
        cfg->tlb = &riscv_iommu_flush_ops;

	iop->ops.map_pages    = riscv_iommu_map_pages;
	iop->ops.unmap_pages  = riscv_iommu_unmap_pages;
	iop->ops.iova_to_phys = riscv_iommu_iova_to_phys;

	printk("io-pgtable for riscv allocated\n");

	return iop;
}

struct io_pgtable_init_fns io_pgtable_riscv_init_fns = {
	.alloc = riscv_iommu_alloc_pgtable,
	.free = riscv_iommu_free_pgtable,
};
