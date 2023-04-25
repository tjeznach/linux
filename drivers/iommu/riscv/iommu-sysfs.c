
// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V architected Ziommu implementations.
 *
 * Copyright © 2022-2023 Rivos Inc.
 *
 * Author: Tomasz Jeznach <tjeznach@rivosinc.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <asm/page.h>

#include "iommu.h"

#define sysfs_dev_to_iommu(dev) \
	container_of(dev_get_drvdata(dev), struct riscv_iommu_device, iommu)

static ssize_t address_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct riscv_iommu_device *iommu = sysfs_dev_to_iommu(dev);
	return sprintf(buf, "%llx\n", iommu->reg_phys);
}

static DEVICE_ATTR_RO(address);

#define ATTR_RD_REG32(name, offset)					\
	ssize_t reg_ ## name ## _show(struct device *dev,		\
			struct device_attribute *attr, char *buf)	\
{									\
	struct riscv_iommu_device *iommu = sysfs_dev_to_iommu(dev);	\
	return sprintf(buf, "0x%x\n",					\
			riscv_iommu_readl(iommu, offset));		\
}

#define ATTR_RD_REG64(name, offset)					\
	ssize_t reg_ ## name ## _show(struct device *dev,		\
			struct device_attribute *attr, char *buf)	\
{									\
	struct riscv_iommu_device *iommu = sysfs_dev_to_iommu(dev);	\
	return sprintf(buf, "0x%llx\n",					\
			riscv_iommu_readq(iommu, offset));		\
}

#define ATTR_WR_REG32(name, offset)					\
	ssize_t reg_ ## name ## _store(struct device *dev,		\
			struct device_attribute *attr,			\
			const char *buf, size_t len)			\
{									\
	struct riscv_iommu_device *iommu = sysfs_dev_to_iommu(dev);	\
	unsigned long val;						\
	int ret;							\
	ret = kstrtoul(buf, 0, &val);					\
	if (ret)							\
		return ret;						\
	riscv_iommu_writel(iommu, offset, val);				\
	return len;							\
}

#define ATTR_WR_REG64(name, offset)					\
	ssize_t reg_ ## name ## _store(struct device *dev,		\
			struct device_attribute *attr,			\
			const char *buf, size_t len)			\
{									\
	struct riscv_iommu_device *iommu = sysfs_dev_to_iommu(dev);	\
	unsigned long long val;						\
	int ret;							\
	ret = kstrtoull(buf, 0, &val);					\
	if (ret)							\
		return ret;						\
	riscv_iommu_writeq(iommu, offset, val);				\
	return len;							\
}

#define ATTR_RO_REG32(name, offset)					\
static ATTR_RD_REG32(name, offset);					\
static DEVICE_ATTR_RO(reg_ ## name)

#define ATTR_RW_REG32(name, offset)					\
static ATTR_RD_REG32(name, offset);					\
static ATTR_WR_REG32(name, offset);					\
static DEVICE_ATTR_RW(reg_ ## name)

#define ATTR_RO_REG64(name, offset)					\
static ATTR_RD_REG64(name, offset);					\
static DEVICE_ATTR_RO(reg_ ## name)

#define ATTR_RW_REG64(name, offset)					\
static ATTR_RD_REG64(name, offset);					\
static ATTR_WR_REG64(name, offset);					\
static DEVICE_ATTR_RW(reg_ ## name)

ATTR_RO_REG64(cap, RISCV_IOMMU_REG_CAP);
ATTR_RO_REG64(fctl, RISCV_IOMMU_REG_FCTL);
ATTR_RO_REG32(cqh, RISCV_IOMMU_REG_CQH);
ATTR_RO_REG32(cqt, RISCV_IOMMU_REG_CQT);
ATTR_RO_REG32(cqcsr, RISCV_IOMMU_REG_CQCSR);
ATTR_RO_REG32(fqh, RISCV_IOMMU_REG_FQH);
ATTR_RO_REG32(fqt, RISCV_IOMMU_REG_FQT);
ATTR_RO_REG32(fqcsr, RISCV_IOMMU_REG_FQCSR);
ATTR_RO_REG32(pqh, RISCV_IOMMU_REG_PQH);
ATTR_RO_REG32(pqt, RISCV_IOMMU_REG_PQT);
ATTR_RO_REG32(pqcsr, RISCV_IOMMU_REG_PQCSR);
ATTR_RO_REG32(ipsr, RISCV_IOMMU_REG_IPSR);
ATTR_RO_REG32(ivec, RISCV_IOMMU_REG_IVEC);
ATTR_RW_REG64(tr_iova, RISCV_IOMMU_REG_TR_REQ_IOVA);
ATTR_RW_REG64(tr_ctrl, RISCV_IOMMU_REG_TR_REQ_CTL);
ATTR_RW_REG64(tr_response, RISCV_IOMMU_REG_TR_RESPONSE);
ATTR_RW_REG32(iocntovf, RISCV_IOMMU_REG_IOCOUNTOVF);
ATTR_RW_REG32(iocntinh, RISCV_IOMMU_REG_IOCOUNTINH);
ATTR_RW_REG64(iohpmcycles, RISCV_IOMMU_REG_IOHPMCYCLES);
ATTR_RW_REG64(iohpmevt_1, RISCV_IOMMU_REG_IOHPMEVT(0));
ATTR_RW_REG64(iohpmevt_2, RISCV_IOMMU_REG_IOHPMEVT(1));
ATTR_RW_REG64(iohpmevt_3, RISCV_IOMMU_REG_IOHPMEVT(2));
ATTR_RW_REG64(iohpmevt_4, RISCV_IOMMU_REG_IOHPMEVT(3));
ATTR_RW_REG64(iohpmevt_5, RISCV_IOMMU_REG_IOHPMEVT(4));
ATTR_RW_REG64(iohpmevt_6, RISCV_IOMMU_REG_IOHPMEVT(5));
ATTR_RW_REG64(iohpmevt_7, RISCV_IOMMU_REG_IOHPMEVT(6));
ATTR_RW_REG64(iohpmctr_1, RISCV_IOMMU_REG_IOHPMCTR(0));
ATTR_RW_REG64(iohpmctr_2, RISCV_IOMMU_REG_IOHPMCTR(1));
ATTR_RW_REG64(iohpmctr_3, RISCV_IOMMU_REG_IOHPMCTR(2));
ATTR_RW_REG64(iohpmctr_4, RISCV_IOMMU_REG_IOHPMCTR(3));
ATTR_RW_REG64(iohpmctr_5, RISCV_IOMMU_REG_IOHPMCTR(4));
ATTR_RW_REG64(iohpmctr_6, RISCV_IOMMU_REG_IOHPMCTR(5));
ATTR_RW_REG64(iohpmctr_7, RISCV_IOMMU_REG_IOHPMCTR(6));

static struct attribute *riscv_iommu_attrs[] = {
	&dev_attr_address.attr,
	&dev_attr_reg_cap.attr,
	&dev_attr_reg_fctl.attr,
	&dev_attr_reg_cqh.attr,
	&dev_attr_reg_cqt.attr,
	&dev_attr_reg_cqcsr.attr,
	&dev_attr_reg_fqh.attr,
	&dev_attr_reg_fqt.attr,
	&dev_attr_reg_fqcsr.attr,
	&dev_attr_reg_pqh.attr,
	&dev_attr_reg_pqt.attr,
	&dev_attr_reg_pqcsr.attr,
	&dev_attr_reg_ipsr.attr,
	&dev_attr_reg_ivec.attr,
	&dev_attr_reg_tr_iova.attr,
	&dev_attr_reg_tr_ctrl.attr,
	&dev_attr_reg_tr_response.attr,
	&dev_attr_reg_iocntovf.attr,
	&dev_attr_reg_iocntinh.attr,
	&dev_attr_reg_iohpmcycles.attr,
	&dev_attr_reg_iohpmctr_1.attr,
	&dev_attr_reg_iohpmevt_1.attr,
	&dev_attr_reg_iohpmctr_2.attr,
	&dev_attr_reg_iohpmevt_2.attr,
	&dev_attr_reg_iohpmctr_3.attr,
	&dev_attr_reg_iohpmevt_3.attr,
	&dev_attr_reg_iohpmctr_4.attr,
	&dev_attr_reg_iohpmevt_4.attr,
	&dev_attr_reg_iohpmctr_5.attr,
	&dev_attr_reg_iohpmevt_5.attr,
	&dev_attr_reg_iohpmctr_6.attr,
	&dev_attr_reg_iohpmevt_6.attr,
	&dev_attr_reg_iohpmctr_7.attr,
	&dev_attr_reg_iohpmevt_7.attr,
	NULL,
};

static struct attribute_group riscv_iommu_group = {
	.name = "riscv-iommu",
	.attrs = riscv_iommu_attrs,
};

const struct attribute_group *riscv_iommu_groups[] = {
	&riscv_iommu_group,
	NULL,
};

int riscv_iommu_sysfs_add(struct riscv_iommu_device *iommu) {
	return iommu_device_sysfs_add(&iommu->iommu, NULL,
		riscv_iommu_groups, "riscv-iommu@%llx", iommu->reg_phys);
}

