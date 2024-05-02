#ifndef	_BASE_H
#define	_BASE_H

typedef unsigned char s8;
typedef unsigned char u8;
typedef unsigned int u32;
typedef long unsigned int u64;

typedef signed int s32;
typedef long signed int s64;

typedef void (*irq_handler_t)(u32 irq_id);

#define REG_PTR32(reg_addr) *((u32 volatile *)reg_addr)
#define REG_PTR64(reg_addr) *((u64 volatile *)reg_addr)

// Raspberry Pi 4b
#define PBASE 0x0FC000000 // When in "Low Peripheral Mode"
// #define PBASE 0x47C000000
#define ARM_NUM_CORES 4

#ifndef __ASSEMBLER__

void put32(u64 addr, u32 val)
{
    REG_PTR32(addr) = val;
}

u32 get32(u64 addr)
{
    return REG_PTR32(addr);
}

u64 get_arm_core_id(void)
{
	u64 core_id;
	asm volatile ("mrs %0, mpidr_el1" : "=r" (core_id));
	return core_id & 0x3;
}

u64 get_arm_exception_lvl(void)
{
	u64 lvl;
	asm volatile(
		"mrs x0, CurrentEL\n" 
    	"lsr x0, x0, #2\n"
		"str x0, [%0]\n"
		: "=r" (lvl)
		);
	return lvl;
}

#endif // __ASSEMBLER__

#endif  /*_ACTYPES_H */