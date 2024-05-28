#include "kernel.h"
#include "arm/sysregs.h"

    // 1st argument x0: type is the interrupt type
    // 2nd argument x1: is the Exception Syndrome Register. 
    // It contains detailed information about what caused the exception
    // 3rd argument x2: is the elr_el1 register which contains the address of the 
    // last instruction executed
	.macro handle_invalid_entry type
	exception_handler_entry
	mov	x0, #\type
	mrs	x1, esr_el1
	mrs	x2, elr_el1
	bl	show_invalid_entry_message
	b	err_hang
	.endm

    // jump to a label that is provided for the 
    // macro as label argument. We need .align 7 instruction 
    // because all exception vectors should be located at offset 0x80 bytes one from another.
	.macro	ventry label
	.align	7
	b	\label
	.endm

    // Stores registers x0 - x30 to the stack.
	.macro	exception_handler_entry
	sub	sp, sp, #S_FRAME_SIZE
	stp	x0, x1, [sp, #16 * 0]
	stp	x2, x3, [sp, #16 * 1]
	stp	x4, x5, [sp, #16 * 2]
	stp	x6, x7, [sp, #16 * 3]
	stp	x8, x9, [sp, #16 * 4]
	stp	x10, x11, [sp, #16 * 5]
	stp	x12, x13, [sp, #16 * 6]
	stp	x14, x15, [sp, #16 * 7]
	stp	x16, x17, [sp, #16 * 8]
	stp	x18, x19, [sp, #16 * 9]
	stp	x20, x21, [sp, #16 * 10]
	stp	x22, x23, [sp, #16 * 11]
	stp	x24, x25, [sp, #16 * 12]
	stp	x26, x27, [sp, #16 * 13]
	stp	x28, x29, [sp, #16 * 14]
	str	x30, [sp, #16 * 15] 
	.endm

    // Restores processor state by copying back the 
    // values of x0 - x30 registers. It also executes 
    // eret instruction, which returns us back to normal execution flow.
	.macro	exception_handler_exit
	ldp	x0, x1, [sp, #16 * 0]
	ldp	x2, x3, [sp, #16 * 1]
	ldp	x4, x5, [sp, #16 * 2]
	ldp	x6, x7, [sp, #16 * 3]
	ldp	x8, x9, [sp, #16 * 4]
	ldp	x10, x11, [sp, #16 * 5]
	ldp	x12, x13, [sp, #16 * 6]
	ldp	x14, x15, [sp, #16 * 7]
	ldp	x16, x17, [sp, #16 * 8]
	ldp	x18, x19, [sp, #16 * 9]
	ldp	x20, x21, [sp, #16 * 10]
	ldp	x22, x23, [sp, #16 * 11]
	ldp	x24, x25, [sp, #16 * 12]
	ldp	x26, x27, [sp, #16 * 13]
	ldp	x28, x29, [sp, #16 * 14]
	ldr	x30, [sp, #16 * 15] 
	add	sp, sp, #S_FRAME_SIZE		
	eret
	.endm

.globl delay
delay: //
	subs x0, x0, #1
	bne delay
	ret

.section ".text.boot" 
.global _start  
_start: //
	// Loader starts us:
	// At 0x80000
	// In EL2
	// Cores 1-3 are suspended
	ldr x0, =__EL2_stack_core0
	ldr x1, =__EL1_stack_core0
	ldr x2, =__EL0_stack_core0
	// msr	sp_el2, x0	    
	mov sp, x0 		// EL2 stack set
	msr	sp_el1, x1	// EL1 stack set
	msr	sp_el0, x2	// EL0 stack set

	/* Setup vector table for EL2 */
	ldr x0, =vector_table						
    msr vbar_el2, x0

	/* Setup vector table for EL1 */
	ldr x0, =vector_table						
    msr vbar_el1, x0

	/* Initialize Generic Timers */
	mrs	x0, cnthctl_el2
	/* 
	* Bit 1 == enables/disables traps for EL0 accesses to the 
	* frequency register and virtual offset register to EL2.
	*	0b0 == traps are enabled
	*	0b1 == traps are disabled
	*
	* Bit 0 == enables/disables traps for EL0 accesses to the 
	* frequency register and physical counter register to EL2.
	*	0b0 == traps are enabled
	*	0b1 == traps are disabled
	*/
	orr	x0, x0, #0x3	/* Enable EL1 access to timers */
	msr	cnthctl_el2, x0
	/* 
	* Holds the 64-bit virtual offset. This is the offset 
	* between the physical count value visible in CNTPCT_EL0 
	* and the virtual count value visible in CNTVCT_EL0. 
	*/
	msr	cntvoff_el2, xzr

	/* Initilize MPID/MPIDR registers */
	mrs	x0, midr_el1
	mrs	x1, mpidr_el1
	msr	vpidr_el2, x0
	msr	vmpidr_el2, x1

	/* Disable coprocessor traps */
	/*
	* cptr_el2: Controls trapping to EL2 of accesses to CPACR, CPACR_EL1, trace, 
	* Activity Monitor, SVE, and Advanced SIMD and floating-point functionality. 
	*
	* Bit 31 == Enables/disables traps for accesses to CPACR_EL1 from EL1 to EL2
	*	0b0 == disables traps
	*	0b1 == enables traps
	*
	* Bit 30 == Enables/disables Traps EL1 and EL0 accesses to all Activity Monitor registers to EL2
	*	0b0 == disables traps
	*	0b1 == enables traps
	*
	* Bit 28 == Traps System register accesses to all implemented trace registers from both Execution states to EL2
	*	0b0 == disables traps
	*	0b1 == enables traps
	*
	* Bits 21:20 == Traps execution at EL2, EL1, and EL0 of instructions that 
	* access the Advanced SIMD and floating-point registers from both Execution states to EL2
	* 	0b11 == Disables traps 
	* 
	* Bits 17:16 == Traps execution at EL2, EL1, and EL0 of SVE 
	* instructions, and instructions that directly access the ZCR_EL1 or ZCR_EL2 System registers to EL2
	*	0b11 == Disables traps
	*/
	mov	x0, #0x33FF 		/* NOTE: Leave SIMD traps enabled for now */ 
	msr	cptr_el2, x0
	/*
	* hstr_el2: Controls trapping to EL2 from EL1
	*
	* Bits 15, 13:5, 3:0 == control whether EL0 and EL1 accesses, using MCR, MRC, MCRR, 
	* and MRRC instructions, to the System registers in the coproc == 0b1111 encoding space, are trapped to EL2
	*	0b0 == Disables traps
	*/
	msr	hstr_el2, xzr		/* Disable coprocessor traps to EL2 */
	/*
	* cpacr_el1: Controls access to trace, SVE, and Advanced SIMD and floating-point functionality
	*
	* Bit 28 == Traps EL0 and EL1 System register accesses to all implemented 
	* trace registers from both Execution states to EL1, or to EL2
	*	0b0 == Disables traps
	*
	* Bits 21:20 == Traps execution at EL1 and EL0 of instructions that 
	* access the Advanced SIMD and floating-point registers from both Execution states to EL1
	*	0b00 == This control causes execution of these instructions at EL1 and EL0 to be trapped.
	*	0b01 == This control causes execution of these instructions at 
	*		EL0 to be trapped, but does not cause execution of any instructions at EL1 to be trapped.
	* 	0b10 == This control causes execution of these instructions at EL1 and EL0 to be trapped.
	* 	0b11 == This control does not cause execution of any instructions to be trapped.
	*
	* Bits 17:16 == Traps execution at EL1 and EL0 of SVE instructions and instructions that directly 
	* access the ZCR_EL1 System register to EL1, or to EL2 when EL2 is implemented and 
	* enabled in the current Security state and HCR_EL2.TGE is 1.
	*	0b00 == This control causes execution of these instructions at EL1 and EL0 to be trapped.
	*	0b01 == This control causes execution of these instructions at 
	*		EL0 to be trapped, but does not cause execution of any instructions at EL1 to be trapped.
	* 	0b10 == This control causes execution of these instructions at EL1 and EL0 to be trapped.
	* 	0b11 == This control does not cause execution of any instructions to be trapped.
	*/
	mov x0, #0
	orr x0, x0, #3 << 20 // don't trap SIMD/FP registers
	orr x0, x0, #3 << 16 // don't trap SVE instructions
	msr	cpacr_el1, x0

	// Hypervisor Configuration Register for EL2
	ldr	x0, =HCR_EL2_VALUE
	msr	hcr_el2, x0

	// Disable MMU for EL1 & EL0
	// ldr	x0, =SCTLR_EL1_VALUE
	// msr	sctlr_el1, x0
	mov	 x0, #0x0800 // bit 11
	movk x0, #0x30D0, lsl #16 // bits 20, 22, 23, 28, 29
	msr	sctlr_el1, x0

	// Disable (Mask) DAIF Interrupts
	// Set on exception to use EL1 stack NOT EL0 stack
	ldr x0, =SPSR_EL2_VALUE
    msr spsr_el2, x0

	// Change from EL2 to EL1
	adr	x0, el1_entry		
	msr	elr_el2, x0
	eret

el1_entry: //

	bss_clear: //
         ldr     x1, =__bss_start
         ldr     x2, =__bss_size
    bss_clear_inner:  //
         str     xzr, [x1], #8
         subs    x2, x2, #8
         b.gt    bss_clear_inner

	bl kernel_main
	b err_hang

/*
 * Exception vectors.
 */
.align	11
.globl vector_table 
vector_table: //
	ventry	sync_invalid_el1t	 // Synchronous EL1t
	ventry	irq_invalid_el1t	 // IRQ EL1t
	ventry	fiq_invalid_el1t	 // FIQ EL1t
	ventry	error_invalid_el1t	 // Error EL1t

	ventry	sync_invalid_el1h	 // Synchronous EL1h
	ventry	el1_irq			     // IRQ EL1h
	ventry	fiq_invalid_el1h	 // FIQ EL1h
	ventry	error_invalid_el1h	 // Error EL1h

	ventry	sync_invalid_el0_64	 // Synchronous 64-bit EL0
	ventry	irq_invalid_el0_64	 // IRQ 64-bit EL0
	ventry	fiq_invalid_el0_64	 // FIQ 64-bit EL0
	ventry	error_invalid_el0_64 // Error 64-bit EL0

	ventry	sync_invalid_el0_32	 // Synchronous 32-bit EL0
	ventry	irq_invalid_el0_32	 // IRQ 32-bit EL0
	ventry	fiq_invalid_el0_32	 // FIQ 32-bit EL0
	ventry	error_invalid_el0_32 // Error 32-bit EL0

sync_invalid_el1t:
	handle_invalid_entry  SYNC_INVALID_EL1t

irq_invalid_el1t:
	handle_invalid_entry  IRQ_INVALID_EL1t

fiq_invalid_el1t:
	handle_invalid_entry  FIQ_INVALID_EL1t

error_invalid_el1t:
	handle_invalid_entry  ERROR_INVALID_EL1t

sync_invalid_el1h:
	handle_invalid_entry  SYNC_INVALID_EL1h

fiq_invalid_el1h:
	handle_invalid_entry  FIQ_INVALID_EL1h

error_invalid_el1h:
	handle_invalid_entry  ERROR_INVALID_EL1h

sync_invalid_el0_64:
	handle_invalid_entry  SYNC_INVALID_EL0_64

irq_invalid_el0_64:
	handle_invalid_entry  IRQ_INVALID_EL0_64

fiq_invalid_el0_64:
	handle_invalid_entry  FIQ_INVALID_EL0_64

error_invalid_el0_64:
	handle_invalid_entry  ERROR_INVALID_EL0_64

sync_invalid_el0_32:
	handle_invalid_entry  SYNC_INVALID_EL0_32

irq_invalid_el0_32:
	handle_invalid_entry  IRQ_INVALID_EL0_32

fiq_invalid_el0_32:
	handle_invalid_entry  FIQ_INVALID_EL0_32

error_invalid_el0_32:
	handle_invalid_entry  ERROR_INVALID_EL0_32

el1_irq:
	exception_handler_entry 
	bl	handle_irq
	exception_handler_exit 

.globl err_hang
err_hang: //
	b err_hang