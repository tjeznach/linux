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

#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "iommu-bits.h"
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
	int rc, vec;

	rc = pci_enable_device_mem(pdev);
	if (rc)
		return rc;

	rc = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (rc)
		goto fail;

	pci_set_master(pdev);

	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM))
		goto fail;

	if (pci_resource_len(pdev, 0) < RISCV_IOMMU_REG_SIZE)
		goto fail;

	iommu = devm_kzalloc(dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		goto fail;

	iommu->dev = dev;
	iommu->reg = pci_iomap(pdev, 0, RISCV_IOMMU_REG_SIZE);

	if (!iommu->reg)
		goto fail;

	dev_set_drvdata(dev, iommu);

	/* Check device reported capabilities / features. */
	iommu->caps = riscv_iommu_readq(iommu, RISCV_IOMMU_REG_CAP);
	iommu->fctl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FCTL);

	/* The PCI driver only uses MSIs, make sure the IOMMU supports this */
	switch (FIELD_GET(RISCV_IOMMU_CAP_IGS, iommu->caps)) {
	case RISCV_IOMMU_CAP_IGS_MSI:
	case RISCV_IOMMU_CAP_IGS_BOTH:
		break;
	default:
		dev_err(dev, "unable to use message-signaled interrupts\n");
		rc = -ENODEV;
		goto fail_unmap;
	}

	/* Allocate and assign IRQ vectors for the various events */
	rc = pci_alloc_irq_vectors(pdev, 1, RISCV_IOMMU_INTR_COUNT,
				   PCI_IRQ_MSIX | PCI_IRQ_MSI);
	if (rc <= 0) {
		dev_err(dev, "unable to allocate irq vectors\n");
		goto fail_unmap;
	}
	for (vec = 0; vec < rc; vec++) {
		iommu->irqs[vec] = msi_get_virq(dev, vec);
		if (!iommu->irqs[vec])
			break;
	}
	iommu->irqs_count = vec;

	/* Enable message-signaled interrupts, fctl.WSI */
	if (iommu->fctl & RISCV_IOMMU_FCTL_WSI) {
		iommu->fctl ^= RISCV_IOMMU_FCTL_WSI;
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_FCTL, iommu->fctl);
	}

	rc = riscv_iommu_init(iommu);
	if (!rc)
		return 0;

fail_unmap:
	iounmap(iommu->reg);
	pci_free_irq_vectors(pdev);
fail:
	pci_release_regions(pdev);
	pci_clear_master(pdev);
	pci_disable_device(pdev);
	return rc;
}

static void riscv_iommu_pci_remove(struct pci_dev *pdev)
{
	struct riscv_iommu_device *iommu = dev_get_drvdata(&pdev->dev);

	riscv_iommu_remove(iommu);
	iounmap(iommu->reg);
	pci_free_irq_vectors(pdev);
	pci_release_regions(pdev);
	pci_clear_master(pdev);
	pci_disable_device(pdev);
}

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
		.of_match_table = riscv_iommu_of_match,
		.suppress_bind_attrs = true,
	},
};

module_driver(riscv_iommu_pci_driver, pci_register_driver, pci_unregister_driver);
