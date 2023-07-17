// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * RISCV IOMMU as a PCIe device
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/bitfield.h>

#include "iommu.h"

/* Rivos Inc. assigned PCI Vendor and Device IDs */
#ifndef PCI_VENDOR_ID_RIVOS
#define PCI_VENDOR_ID_RIVOS             0x1efd
#endif

#ifndef PCI_DEVICE_ID_RIVOS_IOMMU
#define PCI_DEVICE_ID_RIVOS_IOMMU       0xedf1
#endif

static int riscv_iommu_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct riscv_iommu_device *iommu;
	u64 icvec;
	int ret;

	ret = pci_enable_device_mem(pdev);
	if (ret < 0)
		return ret;

	ret = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (ret < 0)
		goto fail;

	ret = -ENOMEM;

	iommu = devm_kzalloc(dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		goto fail;

	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM))
		goto fail;

	if (pci_resource_len(pdev, 0) < RISCV_IOMMU_REG_SIZE)
		goto fail;

	iommu->reg_phys = pci_resource_start(pdev, 0);
	if (!iommu->reg_phys)
		goto fail;

	iommu->reg = devm_ioremap(dev, iommu->reg_phys, RISCV_IOMMU_REG_SIZE);
	if (!iommu->reg)
		goto fail;

	iommu->dev = dev;
	dev_set_drvdata(dev, iommu);

	/* Check device reported capabilities. */
	iommu->cap = riscv_iommu_readq(iommu, RISCV_IOMMU_REG_CAP);

	/* The PCI driver only uses MSIs, make sure the IOMMU supports this */
	switch (FIELD_GET(RISCV_IOMMU_CAP_IGS, iommu->cap)) {
	case RISCV_IOMMU_CAP_IGS_MSI:
	case RISCV_IOMMU_CAP_IGS_BOTH:
		break;
	default:
		dev_err(dev, "unable to use message-signaled interrupts\n");
		ret = -ENODEV;
		goto fail;
	}

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	pci_set_master(pdev);

	/* Allocate and assign IRQ vectors for the various events */
	ret = pci_alloc_irq_vectors(pdev, 1, RISCV_IOMMU_INTR_COUNT, PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_err(dev, "unable to allocate irq vectors\n");
		goto fail;
	}

	ret = -ENODEV;

	iommu->irq_cmdq = msi_get_virq(dev, RISCV_IOMMU_INTR_CQ);
	if (!iommu->irq_cmdq) {
		dev_warn(dev, "no MSI vector %d for the command queue\n",
			 RISCV_IOMMU_INTR_CQ);
		goto fail;
	}

	iommu->irq_fltq = msi_get_virq(dev, RISCV_IOMMU_INTR_FQ);
	if (!iommu->irq_fltq) {
		dev_warn(dev, "no MSI vector %d for the fault/event queue\n",
			 RISCV_IOMMU_INTR_FQ);
		goto fail;
	}

	if (iommu->cap & RISCV_IOMMU_CAP_HPM) {
		iommu->irq_pm = msi_get_virq(dev, RISCV_IOMMU_INTR_PM);
		if (!iommu->irq_pm) {
			dev_warn(dev,
				 "no MSI vector %d for performance monitoring\n",
				 RISCV_IOMMU_INTR_PM);
			goto fail;
		}
	}

	if (iommu->cap & RISCV_IOMMU_CAP_ATS) {
		iommu->irq_priq = msi_get_virq(dev, RISCV_IOMMU_INTR_PQ);
		if (!iommu->irq_priq) {
			dev_warn(dev,
				 "no MSI vector %d for page-request queue\n",
				 RISCV_IOMMU_INTR_PQ);
			goto fail;
		}
	}

	/* Set simple 1:1 mapping for MSI vectors */
	icvec = FIELD_PREP(RISCV_IOMMU_IVEC_CIV, RISCV_IOMMU_INTR_CQ) |
	    FIELD_PREP(RISCV_IOMMU_IVEC_FIV, RISCV_IOMMU_INTR_FQ);

	if (iommu->cap & RISCV_IOMMU_CAP_HPM)
		icvec |= FIELD_PREP(RISCV_IOMMU_IVEC_PMIV, RISCV_IOMMU_INTR_PM);

	if (iommu->cap & RISCV_IOMMU_CAP_ATS)
		icvec |= FIELD_PREP(RISCV_IOMMU_IVEC_PIV, RISCV_IOMMU_INTR_PQ);

	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IVEC, icvec);

	ret = riscv_iommu_init(iommu);
	if (!ret)
		return ret;

 fail:
	pci_free_irq_vectors(pdev);
	pci_clear_master(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	/* Note: devres_release_all() will release iommu and iommu->reg */
	return ret;
}

static void riscv_iommu_pci_remove(struct pci_dev *pdev)
{
	riscv_iommu_remove(dev_get_drvdata(&pdev->dev));
	pci_free_irq_vectors(pdev);
	pci_clear_master(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static int riscv_iommu_suspend(struct device *dev)
{
	dev_warn(dev, "RISC-V IOMMU PM not implemented");
	return -ENODEV;
}

static int riscv_iommu_resume(struct device *dev)
{
	dev_warn(dev, "RISC-V IOMMU PM not implemented");
	return -ENODEV;
}

static DEFINE_SIMPLE_DEV_PM_OPS(riscv_iommu_pm_ops, riscv_iommu_suspend,
				riscv_iommu_resume);

static const struct pci_device_id riscv_iommu_pci_tbl[] = {
	{PCI_VENDOR_ID_RIVOS, PCI_DEVICE_ID_RIVOS_IOMMU,
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0,}
};

MODULE_DEVICE_TABLE(pci, riscv_iommu_pci_tbl);

static const struct of_device_id riscv_iommu_of_match[] = {
	{.compatible = "riscv,pci-iommu",},
	{},
};

MODULE_DEVICE_TABLE(of, riscv_iommu_of_match);

static struct pci_driver riscv_iommu_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = riscv_iommu_pci_tbl,
	.probe = riscv_iommu_pci_probe,
	.remove = riscv_iommu_pci_remove,
	.driver = {
		   .pm = pm_sleep_ptr(&riscv_iommu_pm_ops),
		   .of_match_table = riscv_iommu_of_match,
		   },
};

module_driver(riscv_iommu_pci_driver, pci_register_driver, pci_unregister_driver);
