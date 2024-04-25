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


.globl irq_vector_init
irq_vector_init: //
	adr	x0, vectors // load VBAR_EL1 with virtual
	msr	vbar_el1, x0 // vector table address
	ret

// responsible for unmasking interrupts
// Unmasks the I bit for IRQs, hence #2
.globl enable_irq
enable_irq: //
	msr daifclr, #2 
	ret

// responsible for masking interrupts
// Masks the I bit for IRQs, hence #2
.globl disable_irq
disable_irq: //
	msr	daifset, #2
	ret

.globl get_exception_lvl
get_exception_lvl: //
	mrs x0, CurrentEL 
	// Use mrs instruction to read the value from CurrentEL system register into x0 register. 
	// shift this value 2 bits to the right 
	// (1st 2 bits in the CurrentEL register are reserved and always have value 0)
    lsr x0, x0, #2
	// in the register x0 there is an integer number indicating current exception level.
    ret

.globl put32
put32: //
	str w1, [x0] ; // store the value in w1 (32 bit reg) at the location in x0 (64 bit reg)
	ret

.globl get32
get32: //
	ldr w0, [x0] ; // load value at address in x0 into w0
	ret

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
	mov	sp, __EL2_stack_core0	    // EL2 stack set 
	msr	sp_el1, __EL1_stack_core0	// EL1 stack set
	msr	sp_el0, __EL0_stack_core0	// EL0 stack set
	// Disable MMU for EL1 & EL0
	ldr	x0, =SCTLR_EL1_VALUE
	msr	sctlr_el1, x0
	// Hypervisor Configuration Register for EL2
	ldr	x0, =HCR_EL2_VALUE
	msr	hcr_el2, x0
	// Disable (Mask) DAIF Interrupts
	// Set on exception to use EL1 stack NOT EL0 stack
	ldr x0, =SPSR_EL2_VALUE
    msr spsr_el2, x0
	// Change from EL2 to EL1
	adr	x0, el1_entry		
	msr	elr_el2, x0
	eret

el1_entry: //

	ldr x0, =vector_table						
    msr vbar_el1, x0

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