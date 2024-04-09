.globl put32
put32:
	str w1, [x0] // store the value in w1 (32 bit reg) at the location in x0 (64 bit reg)
	ret

.globl get32
get32:
	ldr w0, [x0] // load address in x0 (64 bit reg) and store in w0 (32 bit reg)
	ret

.globl delay
delay:
	subs x0, x0, #1
	bne delay
	ret