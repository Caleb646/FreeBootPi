.globl irq_vector_init
irq_vector_init:
	adr	x0, vectors // load VBAR_EL1 with virtual
	msr	vbar_el1, x0 // vector table address
	ret

; // responsible for unmasking interrupts
; // Unmasks the I bit for IRQs, hence #2
.globl enable_irq
enable_irq:
	msr daifclr, #2 
	ret

; // responsible for masking interrupts
; // Masks the I bit for IRQs, hence #2
.globl disable_irq
disable_irq:
	msr	daifset, #2
	ret