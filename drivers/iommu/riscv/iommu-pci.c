// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
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

#define DRV_NAME       "riscv-iommu-pci"
#define DRV_VERSION    "0.0.9"

/* Rivos Inc. assigned PCI Vendor and Device IDs */
#ifndef PCI_VENDOR_ID_RIVOS
#define PCI_VENDOR_ID_RIVOS             0x1efd
#endif

#ifndef PCI_DEVICE_ID_RIVOS_IOMMU
#define PCI_DEVICE_ID_RIVOS_IOMMU       0xedf1
#endif

/* RISCV IOMMU as a PCIe device */
static int riscv_iommu_pci_init(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct riscv_iommu_device *iommu = NULL;
	u64 icvec = 0;
	size_t reg_size = 0;
	int ret = 0;

	iommu = devm_kzalloc(dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

	iommu->dev = dev;

	ret = pci_request_mem_regions(pdev, DRV_NAME);
	if (ret < 0)
		return ret;

	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM))
		return -ENODEV;

	reg_size = pci_resource_len(pdev, 0);
	if (reg_size < RISCV_IOMMU_REG_SIZE)
		return -ENODEV;

	iommu->reg_phys = pci_resource_start(pdev, 0);
	if (!iommu->reg_phys)
		return -ENODEV;

	iommu->reg = ioremap(iommu->reg_phys, reg_size);
	if (!iommu->reg) {
		dev_err(dev, "unable to map hardware register region\n");
		ret = -ENOMEM;
		goto fail;
	}

	iommu->cap = riscv_iommu_readq(iommu, RISCV_IOMMU_REG_CAP);

	/* The PCI driver only uses MSIs, make sure the IOMMU supports this */
	ret = FIELD_GET(RISCV_IOMMU_CAP_IGS, iommu->cap);
	if (ret == RISCV_IOMMU_CAP_IGS_WSI) {
		dev_err(dev, "IOMMU only supports wire-signaled interrupts\n");
		ret = -ENODEV;
		goto fail;
	}

	/* Allocate and assign IRQ vectors for the various events */
	ret = pci_alloc_irq_vectors(pdev, 1, RISCV_IOMMU_INTR_COUNT, PCI_IRQ_MSIX);
	if (ret < 0)
		return ret;

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
			dev_warn(dev, "no MSI vector %d for performance monitoring\n",
				 RISCV_IOMMU_INTR_PM);
			goto fail;
		}
	}

	if (iommu->cap & RISCV_IOMMU_CAP_ATS) {
		iommu->irq_priq = msi_get_virq(dev, RISCV_IOMMU_INTR_PQ);
		if (!iommu->irq_priq) {
			dev_warn(dev, "no MSI vector %d for page-request queue\n",
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

	ret = riscv_iommu_init_common(iommu);
	if (!ret)
		return ret;
 fail:
 	if (iommu->reg)
 		iounmap(iommu->reg);
 	if (iommu)
 		kfree(iommu);
 	return ret;
}

static int riscv_iommu_pci_probe(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	int ret;

	ret = pci_enable_device_mem(pdev);
	if (ret < 0)
		return ret;

	pci_set_master(pdev);

	ret = riscv_iommu_pci_init(pdev);
	if (ret < 0) {
		pci_free_irq_vectors(pdev);
		pci_clear_master(pdev);
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		return ret;
	}

	return 0;
}

static void riscv_iommu_pci_remove(struct pci_dev *pdev)
{
	riscv_iommu_remove(&pdev->dev);
	pci_free_irq_vectors(pdev);
	pci_clear_master(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static int riscv_iommu_suspend(struct device *dev)
{
	/* TODO: Silence IOMMU translations. */
	return 0;
}

static int riscv_iommu_resume(struct device *dev)
{
	/* TODO: Restore IOMMU state. */
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(riscv_iommu_pm_ops,
				riscv_iommu_suspend, riscv_iommu_resume);

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
	.name = DRV_NAME,
	.id_table = riscv_iommu_pci_tbl,
	.probe = riscv_iommu_pci_probe,
	.remove = riscv_iommu_pci_remove,
	.driver = {
		   .pm = pm_sleep_ptr(&riscv_iommu_pm_ops),
		   .of_match_table = riscv_iommu_of_match,
		   },
};

static int __init riscv_iommu_init_module(void)
{
	return pci_register_driver(&riscv_iommu_pci_driver);
}

static void __exit riscv_iommu_cleanup_module(void)
{
	pci_unregister_driver(&riscv_iommu_pci_driver);
}

module_init(riscv_iommu_init_module);
module_exit(riscv_iommu_cleanup_module);
