
.globl get_exception_lvl
get_exception_lvl:
	mrs x0, CurrentEL ; // Use mrs instruction to read the value from CurrentEL system register into x0 register. 
	; // shift this value 2 bits to the right 
	; // (1st 2 bits in the CurrentEL register are reserved and always have value 0)
    lsr x0, x0, #2
	; // in the register x0 there is an integer number indicating current exception level.
    ret

.globl put32
put32:
	str w1, [x0] ; // store the value in w1 (32 bit reg) at the location in x0 (64 bit reg)
	ret

.globl get32
get32:
	ldr w0, [x0] ; // load address in x0 (64 bit reg) and store in w0 (32 bit reg)
	ret

.globl delay
delay:
	subs x0, x0, #1
	bne delay
	ret