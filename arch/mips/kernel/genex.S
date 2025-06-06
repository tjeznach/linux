/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 2000, 2001, 2003 Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2002, 2007  Maciej W. Rozycki
 * Copyright (C) 2001, 2012 MIPS Technologies, Inc.  All rights reserved.
 */
#include <linux/init.h>

#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/cacheops.h>
#include <asm/irqflags.h>
#include <asm/regdef.h>
#include <asm/fpregdef.h>
#include <asm/mipsregs.h>
#include <asm/stackframe.h>
#include <asm/sync.h>
#include <asm/thread_info.h>

	__INIT

/*
 * General exception vector for all other CPUs.
 *
 * Be careful when changing this, it has to be at most 128 bytes
 * to fit into space reserved for the exception handler.
 */
NESTED(except_vec3_generic, 0, sp)
	.set	push
	.set	noat
	mfc0	k1, CP0_CAUSE
	andi	k1, k1, 0x7c
#ifdef CONFIG_64BIT
	dsll	k1, k1, 1
#endif
	PTR_L	k0, exception_handlers(k1)
	jr	k0
	.set	pop
	END(except_vec3_generic)

/*
 * General exception handler for CPUs with virtual coherency exception.
 *
 * Be careful when changing this, it has to be at most 256 (as a special
 * exception) bytes to fit into space reserved for the exception handler.
 */
NESTED(except_vec3_r4000, 0, sp)
	.set	push
	.set	arch=r4000
	.set	noat
	mfc0	k1, CP0_CAUSE
	li	k0, 31<<2
	andi	k1, k1, 0x7c
	.set	push
	.set	noreorder
	.set	nomacro
	beq	k1, k0, handle_vced
	 li	k0, 14<<2
	beq	k1, k0, handle_vcei
#ifdef CONFIG_64BIT
	 dsll	k1, k1, 1
#endif
	.set	pop
	PTR_L	k0, exception_handlers(k1)
	jr	k0

	/*
	 * Big shit, we now may have two dirty primary cache lines for the same
	 * physical address.  We can safely invalidate the line pointed to by
	 * c0_badvaddr because after return from this exception handler the
	 * load / store will be re-executed.
	 */
handle_vced:
	MFC0	k0, CP0_BADVADDR
	li	k1, -4					# Is this ...
	and	k0, k1					# ... really needed?
	mtc0	zero, CP0_TAGLO
	cache	Index_Store_Tag_D, (k0)
	cache	Hit_Writeback_Inv_SD, (k0)
#ifdef CONFIG_PROC_FS
	PTR_LA	k0, vced_count
	lw	k1, (k0)
	addiu	k1, 1
	sw	k1, (k0)
#endif
	eret

handle_vcei:
	MFC0	k0, CP0_BADVADDR
	cache	Hit_Writeback_Inv_SD, (k0)		# also cleans pi
#ifdef CONFIG_PROC_FS
	PTR_LA	k0, vcei_count
	lw	k1, (k0)
	addiu	k1, 1
	sw	k1, (k0)
#endif
	eret
	.set	pop
	END(except_vec3_r4000)

	__FINIT

	.section .cpuidle.text,"ax"
	/* Align to 32 bytes for the maximum idle interrupt region size. */
	.align	5
LEAF(r4k_wait)
	/* Keep the ISA bit clear for calculations on local labels here. */
0:	.fill 	0
	/* Start of idle interrupt region. */
	local_irq_enable
	/*
	 * If an interrupt lands here, before going idle on the next
	 * instruction, we must *NOT* go idle since the interrupt could
	 * have set TIF_NEED_RESCHED or caused a timer to need resched.
	 * Fall through -- see skipover_handler below -- and have the
	 * idle loop take care of things.
	 */
1:	.fill	0
	/* The R2 EI/EHB sequence takes 8 bytes, otherwise pad up.  */
	.if		1b - 0b > 32
	.error	"overlong idle interrupt region"
	.elseif	1b - 0b > 8
	.align	4
	.endif
2:	.fill	0
	.equ	r4k_wait_idle_size, 2b - 0b
	/* End of idle interrupt region; size has to be a power of 2. */
	.set	MIPS_ISA_ARCH_LEVEL_RAW
r4k_wait_insn:
	wait
r4k_wait_exit:
	.set	mips0
	local_irq_disable
	jr	ra
	END(r4k_wait)
	.previous

	.macro	BUILD_SKIPOVER_PROLOGUE handler
	FEXPORT(skipover_\handler)
	.set	push
	.set	noat
	MFC0	k0, CP0_EPC
	/* Subtract/add 2 to let the ISA bit propagate through the mask.  */
	PTR_LA	k1, r4k_wait_insn - 2
	ori 	k0, r4k_wait_idle_size - 2
	.set	noreorder
	bne	k0, k1, \handler
	PTR_ADDIU 	k0, r4k_wait_exit - r4k_wait_insn + 2
	.set	reorder
	MTC0	k0, CP0_EPC
	.set pop
	.endm

	.align	5
BUILD_SKIPOVER_PROLOGUE handle_int
NESTED(handle_int, PT_SIZE, sp)
	.cfi_signal_frame
#ifdef CONFIG_TRACE_IRQFLAGS
	/*
	 * Check to see if the interrupted code has just disabled
	 * interrupts and ignore this interrupt for now if so.
	 *
	 * local_irq_disable() disables interrupts and then calls
	 * trace_hardirqs_off() to track the state. If an interrupt is taken
	 * after interrupts are disabled but before the state is updated
	 * it will appear to restore_all that it is incorrectly returning with
	 * interrupts disabled
	 */
	.set	push
	.set	noat
	mfc0	k0, CP0_STATUS
#if defined(CONFIG_CPU_R3000)
	and	k0, ST0_IEP
	bnez	k0, 1f

	mfc0	k0, CP0_EPC
	.set	noreorder
	j	k0
	 rfe
#else
	and	k0, ST0_IE
	bnez	k0, 1f

	eret
#endif
1:
	.set pop
#endif
	SAVE_ALL docfi=1
	CLI
	TRACE_IRQS_OFF

	LONG_L	s0, TI_REGS($28)
	LONG_S	sp, TI_REGS($28)

	/*
	 * SAVE_ALL ensures we are using a valid kernel stack for the thread.
	 * Check if we are already using the IRQ stack.
	 */
	move	s1, sp # Preserve the sp

	/* Get IRQ stack for this CPU */
	ASM_CPUID_MFC0	k0, ASM_SMP_CPUID_REG
#if defined(CONFIG_32BIT) || defined(KBUILD_64BIT_SYM32)
	lui	k1, %hi(irq_stack)
#else
	lui	k1, %highest(irq_stack)
	daddiu	k1, %higher(irq_stack)
	dsll	k1, 16
	daddiu	k1, %hi(irq_stack)
	dsll	k1, 16
#endif
	LONG_SRL	k0, SMP_CPUID_PTRSHIFT
	LONG_ADDU	k1, k0
	LONG_L	t0, %lo(irq_stack)(k1)

	# Check if already on IRQ stack
	PTR_LI	t1, ~(_THREAD_SIZE-1)
	and	t1, t1, sp
	beq	t0, t1, 2f

	/* Switch to IRQ stack */
	li	t1, _IRQ_STACK_START
	PTR_ADD sp, t0, t1

	/* Save task's sp on IRQ stack so that unwinding can follow it */
	LONG_S	s1, 0(sp)
2:
	jal	plat_irq_dispatch

	/* Restore sp */
	move	sp, s1

	j	ret_from_irq
#ifdef CONFIG_CPU_MICROMIPS
	nop
#endif
	END(handle_int)

	__INIT

/*
 * Special interrupt vector for MIPS64 ISA & embedded MIPS processors.
 * This is a dedicated interrupt exception vector which reduces the
 * interrupt processing overhead.  The jump instruction will be replaced
 * at the initialization time.
 *
 * Be careful when changing this, it has to be at most 128 bytes
 * to fit into space reserved for the exception handler.
 */
NESTED(except_vec4, 0, sp)
1:	j	1b			/* Dummy, will be replaced */
	END(except_vec4)

/*
 * EJTAG debug exception handler.
 * The EJTAG debug exception entry point is 0xbfc00480, which
 * normally is in the boot PROM, so the boot PROM must do an
 * unconditional jump to this vector.
 */
NESTED(except_vec_ejtag_debug, 0, sp)
	j	ejtag_debug_handler
#ifdef CONFIG_CPU_MICROMIPS
	 nop
#endif
	END(except_vec_ejtag_debug)

	__FINIT

/*
 * Vectored interrupt handler.
 * This prototype is copied to ebase + n*IntCtl.VS and patched
 * to invoke the handler
 */
BUILD_SKIPOVER_PROLOGUE except_vec_vi
NESTED(except_vec_vi, 0, sp)
	SAVE_SOME docfi=1
	SAVE_AT docfi=1
	.set	push
	.set	noreorder
	PTR_LA	v1, except_vec_vi_handler
	jr	v1
FEXPORT(except_vec_vi_ori)
	 ori	v0, zero, 0		/* Offset in vi_handlers[] */
	.set	pop
	END(except_vec_vi)
EXPORT(except_vec_vi_end)

/*
 * Common Vectored Interrupt code
 * Complete the register saves and invoke the handler, $v0 holds
 * offset into vi_handlers[]
 */
NESTED(except_vec_vi_handler, 0, sp)
	SAVE_TEMP
	SAVE_STATIC
	CLI
#ifdef CONFIG_TRACE_IRQFLAGS
	move	s0, v0
	TRACE_IRQS_OFF
	move	v0, s0
#endif

	LONG_L	s0, TI_REGS($28)
	LONG_S	sp, TI_REGS($28)

	/*
	 * SAVE_ALL ensures we are using a valid kernel stack for the thread.
	 * Check if we are already using the IRQ stack.
	 */
	move	s1, sp # Preserve the sp

	/* Get IRQ stack for this CPU */
	ASM_CPUID_MFC0	k0, ASM_SMP_CPUID_REG
#if defined(CONFIG_32BIT) || defined(KBUILD_64BIT_SYM32)
	lui	k1, %hi(irq_stack)
#else
	lui	k1, %highest(irq_stack)
	daddiu	k1, %higher(irq_stack)
	dsll	k1, 16
	daddiu	k1, %hi(irq_stack)
	dsll	k1, 16
#endif
	LONG_SRL	k0, SMP_CPUID_PTRSHIFT
	LONG_ADDU	k1, k0
	LONG_L	t0, %lo(irq_stack)(k1)

	# Check if already on IRQ stack
	PTR_LI	t1, ~(_THREAD_SIZE-1)
	and	t1, t1, sp
	beq	t0, t1, 2f

	/* Switch to IRQ stack */
	li	t1, _IRQ_STACK_START
	PTR_ADD sp, t0, t1

	/* Save task's sp on IRQ stack so that unwinding can follow it */
	LONG_S	s1, 0(sp)
2:
	PTR_L	v0, vi_handlers(v0)
	jalr	v0

	/* Restore sp */
	move	sp, s1

	j	ret_from_irq
	END(except_vec_vi_handler)

/*
 * EJTAG debug exception handler.
 */
NESTED(ejtag_debug_handler, PT_SIZE, sp)
	.set	push
	.set	noat
	MTC0	k0, CP0_DESAVE
	mfc0	k0, CP0_DEBUG

	andi	k0, k0, MIPS_DEBUG_DBP	# Check for SDBBP.
	beqz	k0, ejtag_return

#ifdef CONFIG_SMP
1:	PTR_LA	k0, ejtag_debug_buffer_spinlock
	__SYNC(full, loongson3_war)
2:	ll	k0, 0(k0)
	bnez	k0, 2b
	PTR_LA	k0, ejtag_debug_buffer_spinlock
	sc	k0, 0(k0)
	beqz	k0, 1b
# ifdef CONFIG_WEAK_REORDERING_BEYOND_LLSC
	sync
# endif

	PTR_LA	k0, ejtag_debug_buffer
	LONG_S	k1, 0(k0)

	ASM_CPUID_MFC0 k1, ASM_SMP_CPUID_REG
	PTR_SRL	k1, SMP_CPUID_PTRSHIFT
	PTR_SLL	k1, LONGLOG
	PTR_LA	k0, ejtag_debug_buffer_per_cpu
	PTR_ADDU k0, k1

	PTR_LA	k1, ejtag_debug_buffer
	LONG_L	k1, 0(k1)
	LONG_S	k1, 0(k0)

	PTR_LA	k0, ejtag_debug_buffer_spinlock
	sw	zero, 0(k0)
#else
	PTR_LA	k0, ejtag_debug_buffer
	LONG_S	k1, 0(k0)
#endif

	SAVE_ALL
	move	a0, sp
	jal	ejtag_exception_handler
	RESTORE_ALL

#ifdef CONFIG_SMP
	ASM_CPUID_MFC0 k1, ASM_SMP_CPUID_REG
	PTR_SRL	k1, SMP_CPUID_PTRSHIFT
	PTR_SLL	k1, LONGLOG
	PTR_LA	k0, ejtag_debug_buffer_per_cpu
	PTR_ADDU k0, k1
	LONG_L	k1, 0(k0)
#else
	PTR_LA	k0, ejtag_debug_buffer
	LONG_L	k1, 0(k0)
#endif

ejtag_return:
	back_to_back_c0_hazard
	MFC0	k0, CP0_DESAVE
	.set	mips32
	deret
	.set	pop
	END(ejtag_debug_handler)

/*
 * This buffer is reserved for the use of the EJTAG debug
 * handler.
 */
	.data
EXPORT(ejtag_debug_buffer)
	.fill	LONGSIZE
#ifdef CONFIG_SMP
EXPORT(ejtag_debug_buffer_spinlock)
	.fill	LONGSIZE
EXPORT(ejtag_debug_buffer_per_cpu)
	.fill	LONGSIZE * NR_CPUS
#endif
	.previous

	__INIT

/*
 * NMI debug exception handler for MIPS reference boards.
 * The NMI debug exception entry point is 0xbfc00000, which
 * normally is in the boot PROM, so the boot PROM must do a
 * unconditional jump to this vector.
 */
NESTED(except_vec_nmi, 0, sp)
	j	nmi_handler
#ifdef CONFIG_CPU_MICROMIPS
	 nop
#endif
	END(except_vec_nmi)

	__FINIT

NESTED(nmi_handler, PT_SIZE, sp)
	.cfi_signal_frame
	.set	push
	.set	noat
	/*
	 * Clear ERL - restore segment mapping
	 * Clear BEV - required for page fault exception handler to work
	 */
	mfc0	k0, CP0_STATUS
	ori	k0, k0, ST0_EXL
	li	k1, ~(ST0_BEV | ST0_ERL)
	and	k0, k0, k1
	mtc0	k0, CP0_STATUS
	_ehb
	SAVE_ALL
	move	a0, sp
	jal	nmi_exception_handler
	/* nmi_exception_handler never returns */
	.set	pop
	END(nmi_handler)

	.macro	__build_clear_none
	.endm

	.macro	__build_clear_sti
	TRACE_IRQS_ON
	STI
	.endm

	.macro	__build_clear_cli
	CLI
	TRACE_IRQS_OFF
	.endm

	.macro	__build_clear_fpe
	CLI
	TRACE_IRQS_OFF
	.set	push
	/* gas fails to assemble cfc1 for some archs (octeon).*/ \
	.set	mips1
	.set	hardfloat
	cfc1	a1, fcr31
	.set	pop
	.endm

	.macro	__build_clear_msa_fpe
	CLI
	TRACE_IRQS_OFF
	_cfcmsa	a1, MSA_CSR
	.endm

	.macro	__build_clear_ade
	MFC0	t0, CP0_BADVADDR
	PTR_S	t0, PT_BVADDR(sp)
	KMODE
	.endm

	.macro __build_clear_gsexc
	.set	push
	/*
	 * We need to specify a selector to access the CP0.Diag1 (GSCause)
	 * register. All GSExc-equipped processors have MIPS32.
	 */
	.set	mips32
	mfc0	a1, CP0_DIAGNOSTIC1
	.set	pop
	TRACE_IRQS_ON
	STI
	.endm

	.macro	__BUILD_silent exception
	.endm

	/* Gas tries to parse the ASM_PRINT argument as a string containing
	   string escapes and emits bogus warnings if it believes to
	   recognize an unknown escape code.  So make the arguments
	   start with an n and gas will believe \n is ok ...  */
	.macro	__BUILD_verbose nexception
	LONG_L	a1, PT_EPC(sp)
#ifdef CONFIG_32BIT
	ASM_PRINT("Got \nexception at %08lx\012")
#endif
#ifdef CONFIG_64BIT
	ASM_PRINT("Got \nexception at %016lx\012")
#endif
	.endm

	.macro	__BUILD_count exception
	LONG_L	t0,exception_count_\exception
	LONG_ADDIU	t0, 1
	LONG_S	t0,exception_count_\exception
	.comm	exception_count\exception, 8, 8
	.endm

	.macro	__BUILD_HANDLER exception handler clear verbose ext
	.align	5
	NESTED(handle_\exception, PT_SIZE, sp)
	.cfi_signal_frame
	.set	noat
	SAVE_ALL
	FEXPORT(handle_\exception\ext)
	__build_clear_\clear
	.set	at
	__BUILD_\verbose \exception
	move	a0, sp
	jal	do_\handler
	j	ret_from_exception
	END(handle_\exception)
	.endm

	.macro	BUILD_HANDLER exception handler clear verbose
	__BUILD_HANDLER \exception \handler \clear \verbose _int
	.endm

	BUILD_HANDLER adel ade ade silent		/* #4  */
	BUILD_HANDLER ades ade ade silent		/* #5  */
	BUILD_HANDLER ibe be cli silent			/* #6  */
	BUILD_HANDLER dbe be cli silent			/* #7  */
	BUILD_HANDLER bp bp sti silent			/* #9  */
	BUILD_HANDLER ri ri sti silent			/* #10 */
	BUILD_HANDLER cpu cpu sti silent		/* #11 */
	BUILD_HANDLER ov ov sti silent			/* #12 */
	BUILD_HANDLER tr tr sti silent			/* #13 */
	BUILD_HANDLER msa_fpe msa_fpe msa_fpe silent	/* #14 */
#ifdef CONFIG_MIPS_FP_SUPPORT
	BUILD_HANDLER fpe fpe fpe silent		/* #15 */
#endif
	BUILD_HANDLER ftlb ftlb none silent		/* #16 */
	BUILD_HANDLER gsexc gsexc gsexc silent		/* #16 */
	BUILD_HANDLER msa msa sti silent		/* #21 */
	BUILD_HANDLER mdmx mdmx sti silent		/* #22 */
#ifdef	CONFIG_HARDWARE_WATCHPOINTS
	/*
	 * For watch, interrupts will be enabled after the watch
	 * registers are read.
	 */
	BUILD_HANDLER watch watch cli silent		/* #23 */
#else
	BUILD_HANDLER watch watch sti verbose		/* #23 */
#endif
	BUILD_HANDLER mcheck mcheck cli verbose		/* #24 */
	BUILD_HANDLER mt mt sti silent			/* #25 */
	BUILD_HANDLER dsp dsp sti silent		/* #26 */
	BUILD_HANDLER reserved reserved sti verbose	/* others */

	.align	5
	LEAF(handle_ri_rdhwr_tlbp)
	.set	push
	.set	noat
	.set	noreorder
	/* check if TLB contains a entry for EPC */
	MFC0	k1, CP0_ENTRYHI
	andi	k1, MIPS_ENTRYHI_ASID | MIPS_ENTRYHI_ASIDX
	MFC0	k0, CP0_EPC
	PTR_SRL	k0, _PAGE_SHIFT + 1
	PTR_SLL	k0, _PAGE_SHIFT + 1
	or	k1, k0
	MTC0	k1, CP0_ENTRYHI
	mtc0_tlbw_hazard
	tlbp
	tlb_probe_hazard
	mfc0	k1, CP0_INDEX
	.set	pop
	bltz	k1, handle_ri	/* slow path */
	/* fall thru */
	END(handle_ri_rdhwr_tlbp)

	LEAF(handle_ri_rdhwr)
	.set	push
	.set	noat
	.set	noreorder
	/* MIPS32:    0x7c03e83b: rdhwr v1,$29 */
	/* microMIPS: 0x007d6b3c: rdhwr v1,$29 */
	MFC0	k1, CP0_EPC
#if defined(CONFIG_CPU_MICROMIPS) || defined(CONFIG_CPU_MIPS32_R2) || defined(CONFIG_CPU_MIPS64_R2)
	and	k0, k1, 1
	beqz	k0, 1f
	 xor	k1, k0
	lhu	k0, (k1)
	lhu	k1, 2(k1)
	ins	k1, k0, 16, 16
	lui	k0, 0x007d
	b	docheck
	 ori	k0, 0x6b3c
1:
	lui	k0, 0x7c03
	lw	k1, (k1)
	ori	k0, 0xe83b
#else
	andi	k0, k1, 1
	bnez	k0, handle_ri
	 lui	k0, 0x7c03
	lw	k1, (k1)
	ori	k0, 0xe83b
#endif
	.set	reorder
docheck:
	bne	k0, k1, handle_ri	/* if not ours */

isrdhwr:
	/* The insn is rdhwr.  No need to check CAUSE.BD here. */
	get_saved_sp	/* k1 := current_thread_info */
	.set	noreorder
	MFC0	k0, CP0_EPC
#if defined(CONFIG_CPU_R3000)
	ori	k1, _THREAD_MASK
	xori	k1, _THREAD_MASK
	LONG_L	v1, TI_TP_VALUE(k1)
	LONG_ADDIU	k0, 4
	jr	k0
	 rfe
#else
#ifndef CONFIG_CPU_DADDI_WORKAROUNDS
	LONG_ADDIU	k0, 4		/* stall on $k0 */
#else
	.set	at=v1
	LONG_ADDIU	k0, 4
	.set	noat
#endif
	MTC0	k0, CP0_EPC
	/* I hope three instructions between MTC0 and ERET are enough... */
	ori	k1, _THREAD_MASK
	xori	k1, _THREAD_MASK
	LONG_L	v1, TI_TP_VALUE(k1)
	.set	push
	.set	arch=r4000
	eret
	.set	pop
#endif
	.set	pop
	END(handle_ri_rdhwr)

#ifdef CONFIG_CPU_R4X00_BUGS64
/* A temporary overflow handler used by check_daddi(). */

	__INIT

	BUILD_HANDLER  daddi_ov daddi_ov none silent	/* #12 */
#endif
