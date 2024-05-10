#include "base.h"

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

void memset(void* src, u8 value, size_t nbytes)
{
	u8* ptr = (u8*)src;
	while(nbytes-- > 0)
	{
		*ptr++ = value;
	}
}