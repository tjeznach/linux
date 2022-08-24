// SPDX-License-Identifier: GPL-2.0-only
/*
 * QEMU EDU device driver.
 *
 * Copyright © 2022-2024 Rivos Inc.
 */

#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/pci_regs.h>
#include <linux/iommu.h>
#include <linux/io.h>

static int sva_disabled;
module_param(sva_disabled, int, 0644);

static int edu_index;

struct qemu_edu_device {
	struct miscdevice miscdev;
	void __iomem *reg;
	bool sva_enabled;
	int irq;
	struct mutex lock;
};

struct qemu_edu_ctx {
	struct qemu_edu_device *dev;
	struct iommu_sva *sva;
	unsigned int pasid;
};

static int qemu_edu_open(struct inode *inode, struct file *fp)
{
	struct qemu_edu_device *dev = fp->private_data;
	struct qemu_edu_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->sva = NULL;
	ctx->pasid = 0;

	if (dev->sva_enabled && !sva_disabled) {
		ctx->sva = iommu_sva_bind_device(dev->miscdev.parent, current->mm);
		if (IS_ERR(ctx->sva)) {
			ret = PTR_ERR(ctx->sva);
			dev_err(dev->miscdev.parent, "SVA allocation failed: %d.\n", ret);
			kfree(ctx);
			return -ENODEV;
		}
		ctx->pasid = iommu_sva_get_pasid(ctx->sva);
		if (ctx->pasid == IOMMU_PASID_INVALID) {
			dev_err(dev->miscdev.parent, "PASID allocation failed.\n");
			iommu_sva_unbind_device(ctx->sva);
			kfree(ctx);
			return -ENODEV;
		}
	}

	/* keep sva context linked to the file */
	fp->private_data = ctx;

	return 0;
}

static int qemu_edu_release(struct inode *inode, struct file *fp)
{
	struct qemu_edu_ctx *ctx = fp->private_data;

	if (ctx->sva)
		iommu_sva_unbind_device(ctx->sva);

	kfree(ctx);
	return 0;
}

static ssize_t qemu_edu_write(struct file *fp, const char __user *buf, size_t count, loff_t *ppos)
{
	int ret;
	struct page *pages[1];
	struct qemu_edu_ctx *ctx = fp->private_data;
	struct qemu_edu_device *dev = ctx->dev;
	size_t dma_len;

	u64 src = (u64)buf;	// from user buffer to device
	/* There's nothing in the internal address space except a 4K
	 * buffer at 0x40000, so create an offset to that.
	 *
	 * Handily also works around a uClibc bug truncating the offset of pread/pwrite.
	 */
	u64 dst = ((u64)*ppos & 0xfff) | 0x40000;
	u64 cmd = 0x01;
	u64 cnt = count;

	/* There's nothing in the internal address space except a 4K
	 * buffer at 0x40000; in some broken cases (e.g. a buggy
	 * uClibc pread/pwrite) the offset is passed as zero.  To work
	 * around these, also accept offsets to the first page
	 * (redirect to 0x40000).
	 */
	if (!(dst & ~0xfff))
		dst |= 0x40000;

	/* page crossing not supported */

	if ((offset_in_page(buf) + cnt) > 4096)
		return -EINVAL;

	if (!ctx->sva) {
		ret = pin_user_pages_fast((unsigned long)buf, 1, 0, pages);
		if (ret != 1) {
			pr_err("Failure locking pages.\n");
			return -ENOMEM;
		}
		dma_len = thp_size(pages[0]);
		src = dma_map_page(dev->miscdev.parent, pages[0],
				offset_in_page(buf), dma_len, DMA_TO_DEVICE);
		ret = dma_mapping_error(dev->miscdev.parent, src);
		if (ret) {
			pr_err("Failure mapping pages.\n");
			return -ENOMEM;
		}
	} else {
		cmd |= 0x08 | (ctx->pasid << 8);
	}

	mutex_lock(&dev->lock);
	writeq(src, dev->reg + 0x80);
	writeq(dst, dev->reg + 0x88);
	writeq(cnt, dev->reg + 0x90);
	writeq(cmd, dev->reg + 0x98);

	while ((readq(dev->reg + 0x98) & 1) && !signal_pending(current))
		schedule_timeout_interruptible(msecs_to_jiffies(10));
	mutex_unlock(&dev->lock);

	if (!ctx->sva) {
		dma_unmap_page(dev->miscdev.parent, src, dma_len, DMA_TO_DEVICE);
		unpin_user_pages_dirty_lock(pages, 1, false);
	}

	return 0;
}

static ssize_t qemu_edu_read(struct file *fp, char __user *buf, size_t count, loff_t *ppos)
{
	int ret;
	struct page *pages[1];
	struct qemu_edu_ctx *ctx = fp->private_data;
	struct qemu_edu_device *dev = ctx->dev;
	size_t dma_len;

	u64 src = ((u64)*ppos & 0xfff) | 0x40000; /* see above */
	u64 dst = (u64)buf;	// from device to user buffer
	u64 cmd = 0x03;
	u64 cnt = count;

	if (!(src & ~0xfff))	/* See qemu_edu_write */
		src |= 0x40000;

	/* page crossing not supported */
	if ((offset_in_page(buf) + cnt) > 4096)
		return -EINVAL;

	if (!ctx->sva) {
		ret = pin_user_pages_fast((unsigned long)buf, 1, FOLL_WRITE, pages);
		if (ret != 1) {
			pr_err("Failure locking pages.\n");
			return -ENOMEM;
		}
		dma_len = thp_size(pages[0]);
		dst = dma_map_page(dev->miscdev.parent, pages[0],
				offset_in_page(buf), dma_len, DMA_FROM_DEVICE);
		ret = dma_mapping_error(dev->miscdev.parent, dst);
		if (ret) {
			pr_err("Failure mapping pages.\n");
			return -ENOMEM;
		}
	} else {
		cmd |= 0x08 | (ctx->pasid << 8);
	}

	mutex_lock(&dev->lock);
	writeq(src, dev->reg + 0x80);
	writeq(dst, dev->reg + 0x88);
	writeq(cnt, dev->reg + 0x90);
	writeq(cmd, dev->reg + 0x98);
	while ((readq(dev->reg + 0x98) & 1) && !signal_pending(current))
		schedule_timeout_interruptible(msecs_to_jiffies(10));
	mutex_unlock(&dev->lock);

	if (!ctx->sva) {
		dma_unmap_page(dev->miscdev.parent, dst, dma_len, DMA_FROM_DEVICE);
		unpin_user_pages_dirty_lock(pages, 1, true);
	}

	return 0;
}

static const struct file_operations qemu_edu_fops = {
	.owner		= THIS_MODULE,
	.open		= qemu_edu_open,
	.release	= qemu_edu_release,
	.read		= qemu_edu_read,
	.write		= qemu_edu_write,
};

static irqreturn_t qemu_edu_irq_handler(int irq, void *data)
{
	int r = IRQ_WAKE_THREAD;
	/* Check device status, if spurious IRQ_NONE. */
	/* FIXME: Ack IRQ? */
	return r;
}

static irqreturn_t qemu_edu_irq_thread(int irq, void *data)
{
	struct qemu_edu_device *dev = data;

	dev_info(dev->miscdev.parent, "IRQ received!\n");
	return IRQ_HANDLED;
}

static int qemu_edu_request_irq(struct pci_dev *pdev)
{
	int ret, irq;
	struct device *dev = &pdev->dev;
	struct qemu_edu_device *edu_dev = pci_get_drvdata(pdev);

	ret = pci_alloc_irq_vectors(pdev,
				    /* Just one vector please */ 1, 1,
				    PCI_IRQ_MSI | PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_err(dev, "Failed to allocate MSI (%d)\n", ret);
		return ret;
	}

	/* Get Linux IRQ number from the MSI vector #0: */
	irq = pci_irq_vector(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "IRQ vector invalid (%d)\n", irq);
		return irq;
	}
	edu_dev->irq = irq;

	ret = devm_request_threaded_irq(dev, irq,
					qemu_edu_irq_handler, qemu_edu_irq_thread,
					IRQF_ONESHOT, dev_name(dev), edu_dev);
	if (ret < 0) {
		dev_err(dev, "Request for IRQ%d failed (%d)\n", irq, ret);
		return ret;
	}

	return ret;
}

static int qemu_edu_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct qemu_edu_device *edu_dev;
	char *name;

	ret = pcim_enable_device(pdev);
	if (ret < 0) {
		dev_err(dev, "Can not enable device: %d.\n", ret);
		return ret;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret < 0)
		return ret;

	edu_dev = kzalloc(sizeof(*edu_dev), GFP_KERNEL);
	if (!edu_dev)
		return -ENOMEM;


	if (edu_index) {
		name = kasprintf(GFP_KERNEL, "qemu-edu%d", edu_index);
		edu_index++;
	} else {
		name = kasprintf(GFP_KERNEL, "qemu-edu");
		edu_index++;
	}


	edu_dev->miscdev.fops = &qemu_edu_fops;
	edu_dev->miscdev.parent = dev;
	edu_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	edu_dev->miscdev.name = name;
	mutex_init(&edu_dev->lock);

	edu_dev->reg = pci_ioremap_bar(pdev, 0);
	if (!edu_dev->reg) {
		dev_err(dev, "Unable to map BAR0.\n");
		kfree(edu_dev->miscdev.name);
		kfree(edu_dev);
		return -ENODEV;
	}

	if (iommu_dev_enable_feature(dev, IOMMU_DEV_FEAT_IOPF)) {
		dev_warn(dev, "Unable to turn on user IOPFfeature.\n");
	} else if (iommu_dev_enable_feature(dev, IOMMU_DEV_FEAT_SVA)) {
		dev_warn(dev, "Unable to turn on user SVA feature.\n");
	} else {
		dev_info(dev, "SVA feature enabled.\n");
		edu_dev->sva_enabled = true;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, edu_dev);

	ret = qemu_edu_request_irq(pdev);
	if (ret)
		dev_warn(dev, "IRQ setup failed\n");

	ret = misc_register(&edu_dev->miscdev);
	if (ret < 0) {
		iommu_dev_disable_feature(dev, IOMMU_DEV_FEAT_SVA);
		iommu_dev_disable_feature(dev, IOMMU_DEV_FEAT_IOPF);
		iounmap(edu_dev->reg);
		kfree(edu_dev->miscdev.name);
		kfree(edu_dev);
		return ret;
	}

	return 0;
}

static void qemu_edu_remove(struct pci_dev *pdev)
{
	struct qemu_edu_device *edu_dev = pci_get_drvdata(pdev);

	iommu_dev_disable_feature(edu_dev->miscdev.parent, IOMMU_DEV_FEAT_IOPF);
	iommu_dev_disable_feature(edu_dev->miscdev.parent, IOMMU_DEV_FEAT_SVA);
	misc_deregister(&edu_dev->miscdev);
	iounmap(edu_dev->reg);
	kfree(edu_dev->miscdev.name);
	kfree(edu_dev);
	pci_set_drvdata(pdev, NULL);
}

static int qemu_edu_suspend(struct device *dev)
{
	return 0;
}

static int qemu_edu_resume(struct device *dev)
{
	return 0;
}

static const struct pci_device_id qemu_edu_pci_ids[] = {
	{ PCI_DEVICE(0x1234, 0x11e8) },	/* QEMU:EDU */
	{0, },
};

MODULE_DEVICE_TABLE(pci, qemu_edu_pci_ids);

static SIMPLE_DEV_PM_OPS(qemu_edu_pm_ops,
	qemu_edu_suspend,
	qemu_edu_resume);

static struct pci_driver qemu_edu_driver = {
	.name     = "qemu-edu",
	.id_table = qemu_edu_pci_ids,
	.probe    = qemu_edu_probe,
	.remove   = qemu_edu_remove,
	.driver	= {
		.pm = &qemu_edu_pm_ops,
	},
};

static int __init qemu_edu_init(void)
{
	return pci_register_driver(&qemu_edu_driver);
}

static void __exit qemu_edu_exit(void)
{
	pci_unregister_driver(&qemu_edu_driver);
}

MODULE_LICENSE("GPL v2");

module_init(qemu_edu_init);
module_exit(qemu_edu_exit);

