/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RISC-V IOMMU Register Layout and Data Structures.
 *
 * Copyright (C) 2022-2023 Rivos Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _RISCV_IOMMU_BITS_H_
#define _RISCV_IOMMU_BITS_H_

#include <linux/types.h>
#include <linux/bits.h>

/*
 * RISC-V IOMMU register layout and data structures.
 *
 * Based on RISC-V IOMMU Architecture Specification, Version 0.9, 01/2023
 * https://github.com/riscv-non-isa/riscv-iommu
 */

/* Latest spec is 0.9 */
#define RIO_SPEC_DOT_VER        0x09

/* Register Layout */
#define RIO_REG_CAP             0x0000
#define RIO_REG_FCTL            0x0008
#define RIO_REG_DDTP            0x0010
#define RIO_REG_CQB             0x0018
#define RIO_REG_CQH             0x0020
#define RIO_REG_CQT             0x0024
#define RIO_REG_FQB             0x0028
#define RIO_REG_FQH             0x0030
#define RIO_REG_FQT             0x0034
#define RIO_REG_PQB             0x0038
#define RIO_REG_PQH             0x0040
#define RIO_REG_PQT             0x0044
#define RIO_REG_CQCSR           0x0048
#define RIO_REG_FQCSR           0x004C
#define RIO_REG_PQCSR           0x0050
#define RIO_REG_IPSR            0x0054
#define RIO_REG_IOCNTOVF        0x0058
#define RIO_REG_IOCNTINH        0x005C
#define RIO_REG_IOHPMCYCLES     0x0060
#define RIO_REG_IOHPMCTR_BASE   0x0068
#define RIO_REG_IOHPMCTR_END    0x0158
#define RIO_REG_IOHPMEVT_BASE   0x0160
#define RIO_REG_IOHPMEVT_END    0x0250
#define RIO_REG_TR_REQ_IOVA     0x0258
#define RIO_REG_TR_REQ_CTRL     0x0260
#define RIO_REG_TR_RESPONSE     0x0268

#define RIO_REG_IVEC            0x02F8
#define RIO_REG_MSI_CONFIG      0x0300

#define RIO_REG_SIZE            0x1000

/* Capabilities supported by the IOMMU, RIO_REG_CAP */
#define RIO_CAP_REV             GENMASK_ULL(7, 0)
#define RIO_CAP_SV32            BIT_ULL(8)
#define RIO_CAP_SV39            BIT_ULL(9)
#define RIO_CAP_SV48            BIT_ULL(10)
#define RIO_CAP_SV57            BIT_ULL(11)
#define RIO_CAP_SVPBMT          BIT_ULL(15)
#define RIO_CAP_SV32X4          BIT_ULL(16)
#define RIO_CAP_SV39X4          BIT_ULL(17)
#define RIO_CAP_SV48X4          BIT_ULL(18)
#define RIO_CAP_SV57X4          BIT_ULL(19)
#define RIO_CAP_MSI_FLAT        BIT_ULL(22)
#define RIO_CAP_MSI_MRIF        BIT_ULL(23)
#define RIO_CAP_AMO             BIT_ULL(24)
#define RIO_CAP_ATS             BIT_ULL(25)
#define RIO_CAP_T2GPA           BIT_ULL(26)
#define RIO_CAP_END             BIT_ULL(27)
#define RIO_CAP_IGS             GENMASK_ULL(29, 28)
#define RIO_CAP_HPM             BIT_ULL(30)
#define RIO_CAP_DBG             BIT_ULL(31)
#define RIO_CAP_PAS             GENMASK_ULL(37, 32)
#define RIO_CAP_PD8             BIT_ULL(38)
#define RIO_CAP_PD17            BIT_ULL(39)
#define RIO_CAP_PD20            BIT_ULL(40)

/* Features control register, RIO_REG_FCTL */
#define RIO_FCTL_BE             BIT_ULL(0)
#define RIO_FCTL_WIS            BIT_ULL(1)
#define RIO_FCTL_GXL            BIT_ULL(2)

/* Interrupt generation support */
#define RIO_IGS_MSI		0
#define RIO_IGS_WSI		1
#define RIO_IGS_ANY		2

/* Device directory table pointer */
#define RIO_DDTP_BUSY           BIT_ULL(4)
#define RIO_DDTP_MODE           GENMASK_ULL(3, 0)
#define RIO_DDTP_PPN            GENMASK_ULL(53, 10)

#define RIO_DDTP_MODE_OFF       0
#define RIO_DDTP_MODE_BARE      1
#define RIO_DDTP_MODE_1LVL      2
#define RIO_DDTP_MODE_2LVL      3
#define RIO_DDTP_MODE_3LVL      4
#define RIO_DDTP_MODE_MAX       RIO_DDTP_MODE_3LVL

#define RIO_DDTE_VALID          BIT_ULL(0)
#define RIO_DDTE_PPN            GENMASK_ULL(53, 10)

/* Command queue base register */
#define RIO_CQ_LOG2SZ           GENMASK_ULL(4, 0)
#define RIO_CQ_PPN              GENMASK_ULL(53, 10)

/* Command queue control and status register */
#define RIO_CQ_EN               BIT(0)
#define RIO_CQ_IE               BIT(1)
#define RIO_CQ_FAULT            BIT(8)
#define RIO_CQ_TIMEOUT          BIT(9)
#define RIO_CQ_ERROR            BIT(10)
#define RIO_CQ_FENCE_W_IP       BIT(11)
#define RIO_CQ_ACTIVE           BIT(16)
#define RIO_CQ_BUSY             BIT(17)

/* Fault queue base register */
#define RIO_FQ_LOG2SZ           GENMASK_ULL(4, 0)
#define RIO_FQ_PPN              GENMASK_ULL(53, 10)

/* Fault queue control and status register */
#define RIO_FQ_EN               BIT(0)
#define RIO_FQ_IE               BIT(1)
#define RIO_FQ_FAULT            BIT(8)
#define RIO_FQ_FULL             BIT(9)
#define RIO_FQ_ACTIVE           BIT(16)
#define RIO_FQ_BUSY             BIT(17)

/* Page request queue base register */
#define RIO_PQ_LOG2SZ           GENMASK_ULL(4, 0)
#define RIO_PQ_PPN              GENMASK_ULL(53, 10)

/* Page request queue control and status register */
#define RIO_PQ_EN               BIT(0)
#define RIO_PQ_IE               BIT(1)
#define RIO_PQ_FAULT            BIT(8)
#define RIO_PQ_FULL             BIT(9)
#define RIO_PQ_ACTIVE           BIT(16)
#define RIO_PQ_BUSY             BIT(17)

/* Interrupt Sources, used for IPSR and IVEC indexing. */
#define RIO_INT_CQ              0
#define RIO_INT_FQ              1
#define RIO_INT_PM              2
#define RIO_INT_PQ              3
#define RIO_INT_COUNT           4

#define RIO_IPSR_CQIP           BIT(RIO_INT_CQ)
#define RIO_IPSR_FQIP           BIT(RIO_INT_FQ)
#define RIO_IPSR_PMIP           BIT(RIO_INT_PM)
#define RIO_IPSR_PQIP           BIT(RIO_INT_PQ)

/* HPM event counter register. */
#define RIO_IOHPMCTR_NUM_REGS   31
#define RIO_IOHPMEVT_NUM_REGS   31

#define RIO_IOCNTOVF_CY         BIT_ULL(0)
#define RIO_IOCNTINH_CY         BIT_ULL(0)

#define RIO_IOHPMCYCLES_CNTR    GENMASK_ULL(62, 0)
#define RIO_IOHPMCYCLES_OF      BIT_ULL(63)

#define RIO_IOHPMEVT_EID        GENMASK_ULL(14, 0)
#define RIO_IOHPMEVT_DMASK      BIT_ULL(15)
#define RIO_IOHPMEVT_PID_PSCID  GENMASK_ULL(35, 16)
#define RIO_IOHPMEVT_DID_GSCID  GENMASK_ULL(59, 36)
#define RIO_IOHPMEVT_PV_PSCV    BIT_ULL(60)
#define RIO_IOHPMEVT_DV_GSCV    BIT_ULL(61)
#define RIO_IOHPMEVT_IDT        BIT_ULL(62)
#define RIO_IOHPMEVT_OF         BIT_ULL(63)

/* Interrupt vector mapping */
#define RIO_IVEC_CQIV           GENMASK_ULL(3, 0)
#define RIO_IVEC_FQIV           GENMASK_ULL(7, 4)
#define RIO_IVEC_PMIV           GENMASK_ULL(11, 8)
#define RIO_IVEC_PQIV           GENMASK_ULL(15, 12)

/* Translation request interface */
#define RIO_TRREQ_BUSY          BIT_ULL(0)
#define RIO_TRREQ_PRIV          BIT_ULL(1)
#define RIO_TRREQ_EXEC          BIT_ULL(2)
#define RIO_TRREQ_RO            BIT_ULL(3)
#define RIO_TRREQ_PID           GENMASK_ULL(31, 12)
#define RIO_TRREQ_PV            BIT_ULL(32)
#define RIO_TRREQ_DID           GENMASK_ULL(63, 40)

#define RIO_TRRSP_FAULT         BIT_ULL(0)
#define RIO_TRRSP_PBMT          GENMASK_ULL(8, 7)
#define RIO_TRRSP_S             BIT_ULL(9)
#define RIO_TRRSP_PPN           GENMASK_ULL(53, 10)

/* Device Context: Translation Control */
#define RIO_DCTC_VALID          BIT_ULL(0)
#define RIO_DCTC_EN_ATS         BIT_ULL(1)
#define RIO_DCTC_EN_PRI         BIT_ULL(2)
#define RIO_DCTC_T2GPA          BIT_ULL(3)
#define RIO_DCTC_DTF            BIT_ULL(4)
#define RIO_DCTC_PDTV           BIT_ULL(5)
#define RIO_DCTC_PRPR           BIT_ULL(6)
#define RIO_DCTC_GADE           BIT_ULL(7)
#define RIO_DCTC_SADE           BIT_ULL(8)
#define RIO_DCTC_DPE            BIT_ULL(9)
#define RIO_DCTC_SBE            BIT_ULL(10)
#define RIO_DCTC_SXL            BIT_ULL(11)

/* Device Context: FSC, GATP */
#define RIO_ATP_PPN             GENMASK_ULL(43, 0)
#define RIO_ATP_GSCID           GENMASK_ULL(59, 44)
#define RIO_ATP_MODE            GENMASK_ULL(63, 60)

/* ATP.MODE: pass-through */
#define RIO_ATP_MODE_BARE       0

/* ATP.MODE when fctl.GXL == 1 or tc.SXL == 1 */
#define RIO_ATP_MODE_SV32       8

/* ATP.MODE when fctl.GXL == 0 or tc.SXL == 0 */
#define RIO_ATP_MODE_SV39       8
#define RIO_ATP_MODE_SV48       9
#define RIO_ATP_MODE_SV57       10

/* FSC mode field when TC.RIO_TC_PDTV is set */
#define RIO_PDTP_MODE_BARE      0

#define RIO_PDTP_MODE_PD8       1
#define RIO_PDTP_MODE_PD17      2
#define RIO_PDTP_MODE_PD20      3

#define RIO_PDTE_VALID          BIT_ULL(0)
#define RIO_PDTE_PPN            GENMASK_ULL(53, 10)

/* Device Context MSI Page Table Pointer */
#define RIO_PCTA_V              BIT_ULL(0)
#define RIO_PCTA_ENS            BIT_ULL(1)
#define RIO_PCTA_SUM            BIT_ULL(2)
#define RIO_PCTA_PSCID          GENMASK_ULL(31, 12)

#define RIO_DCMSI_PPN           GENMASK_ULL(43, 0)
#define RIO_DCMSI_MODE          GENMASK_ULL(63, 60)

#define RIO_DCMSI_MODE_OFF      0
#define RIO_DCMSI_MODE_FLAT     1

#define RIO_MSIPTE_V            BIT_ULL(0)
#define RIO_MSIPTE_M            GENMASK_ULL(2, 1)
#define RIO_MSIPTE_C            BIT_ULL(63)

#define RIO_MSIPTE_M_MRIF       1
#define RIO_MSIPTE_M_BASIC      3

/* riscv_iommu_msipte.msipte when RIO_MSIPTE_M == BASIC (3) */
#define RIO_MSIPTE_PPN          GENMASK_ULL(53, 10)

/* riscv_iommu_msipte.msipte when RIO_MSIPTE_M == MRIF (1) */
#define RIO_MSIPTE_MRIF_ADDR    GENMASK_ULL(53, 7)

/* riscv_iommu_msipte.notice when RIO_MSIPTE_M == MRIF (1) */
#define RIO_MSIPTE_NPPN_PPN     GENMASK_ULL(53, 10)
#define RIO_MSIPTE_NPPN_N10     BIT_ULL(60)
#define RIO_MSIPTE_NPPN_N90     GENMASK_ULL(9, 0)

/* Page Request record */
#define RIO_PRR_PID             GENMASK_ULL(31, 12)
#define RIO_PRR_PV              BIT_ULL(32)
#define RIO_PRR_PRIV            BIT_ULL(33)
#define RIO_PRR_X               BIT_ULL(34)
#define RIO_PRR_DID             GENMASK_ULL(63, 40)

/* riscv_iommu_command.request opcode and function mask */
#define RIO_CMD_OP              GENMASK_ULL(9, 0)

#define RIO_CMD_IOTINVAL_VMA    0x001
#define RIO_CMD_IOTINVAL_GVMA   0x081
#define RIO_CMD_IOFENCE_C       0x002
#define RIO_CMD_IODIR_DDT       0x003
#define RIO_CMD_IODIR_PDT       0x083
#define RIO_CMD_ATS_INVAL       0x004
#define RIO_CMD_ATS_PRGR        0x084

/* opcode == IOTINVAL.* */
#define RIO_IOTINVAL_AV         BIT_ULL(10)
#define RIO_IOTINVAL_PSCV       BIT_ULL(32)
#define RIO_IOTINVAL_GV         BIT_ULL(33)
#define RIO_IOTINVAL_PSCID      GENMASK_ULL(31, 12)
#define RIO_IOTINVAL_GSCID      GENMASK_ULL(59, 44)

/* opcode == IOFENCE.* */
#define RIO_IOFENCE_AV          BIT_ULL(10)
#define RIO_IOFENCE_WSI         BIT_ULL(11)
#define RIO_IOFENCE_PR          BIT_ULL(12)
#define RIO_IOFENCE_PW          BIT_ULL(13)
#define RIO_IOFENCE_DATA        GENMASK_ULL(63, 32)

/* opcode == IODIR.* */
#define RIO_IODIR_DV            BIT_ULL(33)
#define RIO_IODIR_PID           GENMASK_ULL(31, 12)
#define RIO_IODIR_DID           GENMASK_ULL(63, 40)

/* opcode == ATS */
#define RIO_ATS_PV              BIT_ULL(32)
#define RIO_ATS_DSV             BIT_ULL(33)
#define RIO_ATS_PID             GENMASK_ULL(31, 12)
#define RIO_ATS_RID             GENMASK_ULL(55, 40)
#define RIO_ATS_DSEG            GENMASK_ULL(63, 56)

/* riscv_iommu_event.reason */
#define RIO_EVENT_CAUSE         GENMASK_ULL(11, 0)
#define RIO_EVENT_PID           GENMASK_ULL(31, 12)
#define RIO_EVENT_PV            BIT_ULL(32)
#define RIO_EVENT_PRIV          BIT_ULL(33)
#define RIO_EVENT_TTYP          GENMASK_ULL(39, 34)
#define RIO_EVENT_DID           GENMASK_ULL(63, 40)

/*
 * HPM Event IDs.
 * IOMMU Specification, Chapter 5.23. Performance-monitoring event selector.
 */
#define RIO_HPMEVENT_INVALID    0	/* Invalid event, do not count */
#define RIO_HPMEVENT_URQ        1	/* Untranslated requests */
#define RIO_HPMEVENT_TRQ        2	/* Translated requests */
#define RIO_HPMEVENT_ATS_RQ     3	/* ATS translation requests */
#define RIO_HPMEVENT_TLB_MISS   4	/* TLB misses */
#define RIO_HPMEVENT_DD_WALK    5	/* Device directory walks */
#define RIO_HPMEVENT_PD_WALK    6	/* Process directory walks */
#define RIO_HPMEVENT_S_VS_WALKS 7	/* S/VS-Stage page table walks */
#define RIO_HPMEVENT_G_WALKS    8	/* G-Stage page table walks */
#define RIO_HPMEVENT_MAX        9	/* Value to denote maximum Event IDs */

/*
 * RISC-V IOMMU Fault Transaction Type / Exception Codes
 * IOMMU Specification, Chapter 3.2. Fault/Event-Queue
 */
#define RIO_TTYP_NONE               0	/* Fault not caused by trx */
#define RIO_TTYP_URX                1	/* Untranslated read for execute trx */
#define RIO_TTYP_URD                2	/* Untranslated read transaction */
#define RIO_TTYP_UWR                3	/* Untranslated write/AMO trx */
#define RIO_TTYP_TRX                5	/* Translated read for execute trx */
#define RIO_TTYP_TRD                6	/* Translated read transaction */
#define RIO_TTYP_TWR                7	/* Translated write/AMO transaction */
#define RIO_TTYP_ATS                8	/* PCIe ATS Translation Request */
#define RIO_TTYP_MRQ                9	/* Message Request */

#define RIO_CAUSE_EX_FAULT          1	/* Instruction access fault */
#define RIO_CAUSE_RD_ALIGN          4	/* Read address misaligned */
#define RIO_CAUSE_RD_FAULT          5	/* Read access fault */
#define RIO_CAUSE_WR_ALIGN          6	/* Write/AMO address misaligned */
#define RIO_CAUSE_WR_FAULT          7	/* Write/AMO access fault */
#define RIO_CAUSE_EX_FAULT_S       12	/* Instruction page fault */
#define RIO_CAUSE_RD_FAULT_S       13	/* Read page fault */
#define RIO_CAUSE_WR_FAULT_S       15	/* Write/AMO page fault */
#define RIO_CAUSE_EX_FAULT_G       20	/* Instruction guest page fault */
#define RIO_CAUSE_RD_FAULT_G       21	/* Read guest-page fault */
#define RIO_CAUSE_WR_FAULT_G       23	/* Write/AMO guest-page fault */
#define RIO_CAUSE_DMA_DISABLED    256	/* Inbound transactions disallowed */
#define RIO_CAUSE_DDT_FAULT       257	/* DDT entry load access fault */
#define RIO_CAUSE_DDT_INVALID     258	/* DDT entry not valid */
#define RIO_CAUSE_DDT_UNSUPPORTED 259	/* DDT entry misconfigured */
#define RIO_CAUSE_REQ_INVALID     260	/* Transaction type disallowed */
#define RIO_CAUSE_MSI_PTE_FAULT   261	/* MSI PTE load access fault */
#define RIO_CAUSE_MSI_INVALID     262	/* MSI PTE not valid */
#define RIO_CAUSE_MSI_UNSUPPORTED 263	/* MSI PTE entry misconfigured */
#define RIO_CAUSE_MRIF_FAULT      264	/* MRIF access fault */
#define RIO_CAUSE_PDT_FAULT       265	/* PDT load access fault */
#define RIO_CAUSE_PDT_INVALID     266	/* PDT not valid */
#define RIO_CAUSE_PDT_UNSUPPORTED 267	/* PDT entry misconfigured */
#define RIO_CAUSE_DDT_CORRUPTED   268	/* DDT entry corrupted */
#define RIO_CAUSE_PDT_CORRUPTED   269	/* PDT entry corrupted */
#define RIO_CAUSE_MSI_CORRUPTED   270	/* MSI entry corrupted */
#define RIO_CAUSE_MRIF_CORRUPTED  271	/* DDT entry corrupted */
#define RIO_CAUSE_ERROR           272	/* Internal data error */
#define RIO_CAUSE_MSI_FAULT       273	/* MSI write access fault */
#define RIO_CAUSE_PT_CORRUPTED    274	/* Page Table Corrupted */

/*
 * Device Context.
 * IOMMU Specification, Chapter 2.1. Device-Directory-Table.
 */
struct riscv_iommu_dc {
	uint64_t tc;		/* Translation Control */
	uint64_t gatp;		/* second-stage address translation and protection */
	uint64_t ta;		/* Translation attributes */
	uint64_t fsc;		/* first-stage address translation and protection */
	uint64_t msiptp;	/* MSI Page Table Pointer (extended context) */
	uint64_t msi_addr_mask;
	uint64_t msi_addr_pattern;
	uint64_t _reserved;
};

/*
 * MSI Page Table Entry
 * The RISC-V Advanced Interrupt Architecture, Chapter 9.5. MSI Page Tables
 */
struct riscv_iommu_msipte {
	uint64_t msipte;
	uint64_t notice;
};

/*
 * Page Request Queue element
 * IOMMU Specification, Chapter 3.3. Page-Request-Queue
 */
struct riscv_iommu_page_request {
	uint64_t request;
	uint64_t payload;
};

/*
 * Process Directory Entry.
 * IOMMU Specification, Chapter 2.2. Process-Directory-Table.
 */
struct riscv_iommu_pc {
	uint64_t ta;
	uint64_t fsc;
};

/*
 * I/O Management Unit Command.
 * IOMMU Specification, Chapter 3.1. Command-Queue.
 */
struct riscv_iommu_command {
	uint64_t request;
	uint64_t address;
};

/*
 * Fault Queue element.
 * IOMMU Specification, Chapter 3.2. Fault/Event-Queue.
 */
struct riscv_iommu_event {
	uint64_t reason;
	uint64_t _reserved;
	uint64_t iova;
	uint64_t phys;
};

#endif
