#include "base.h"
#include "printf.h"
#include "kernel.h"
#include "peripherals/timer.h"
#include "irq.h"
#include "peripherals/uart.h"
#include "arm/sysregs.h"



static u32 volatile s_ncritical_lvl[ARM_NUM_CORES];
static u64 volatile s_int_flags[ARM_NUM_CORES][MAX_NESTED_INTERRUPTS];
/*
* \brief	Returns ARM core interrupt status
* \return	
* 0 is not masked (enabled) & 1 is masked (disabled)
* Bit 9 = Watchpoint, Breakpoint, and Software Step exceptions 
* Bit 8 = SError exception
* Bit 7 = IRQ mask
* Bit 6 = FIQ mask
*/
u64 get_daif_flags(void)
{
	u64 flags;
	asm volatile ("mrs %0, daif" : "=r" (flags));
	return flags;
}

void set_daif_flags(u64 flags)
{
	asm volatile ("msr daif, %0" :: "r" (flags));
}

s32 enter_critical(u32 target_lvl)
{
	u64 flags = get_daif_flags();
	u64 core_id = get_arm_core_id();
	if(s_ncritical_lvl[core_id] >= MAX_NESTED_INTERRUPTS)
	{
		return 0;
	}
	// Save DAIF flags to be restored after leaving interrupt
	s_int_flags[core_id][s_ncritical_lvl[core_id]++] = flags;
	DISABLE_IRQ_FIQ();

	/*
	* Ensure all load & store operations are visible to this core
	* before entering the critical section
	*/
	DATA_MEMORY_BARRIER_NOSHARE_ANY();
	return 1;
}

s32 leave_critical(void)
{
	/* 
	* Ensure all load & store operations above the leave_critical call 
	* are visible to this core before exiting the critical section
	*/
	DATA_MEMORY_BARRIER_NOSHARE_ANY();

	u64 core_id = get_arm_core_id();
	if(s_ncritical_lvl[core_id] <= 0)
	{
		return 0;
	}
	// restore saved DAIF flags
	u64 flags = s_int_flags[core_id][--s_ncritical_lvl[core_id]];
	set_daif_flags(flags);
	return 1;
}

const char* entry_error_messages[] = {
	"SYNC_INVALID_EL1t",
	"IRQ_INVALID_EL1t",		
	"FIQ_INVALID_EL1t",		
	"ERROR_INVALID_EL1t",		

	"SYNC_INVALID_EL1h",		
	"IRQ_INVALID_EL1h",		
	"FIQ_INVALID_EL1h",		
	"ERROR_INVALID_EL1h",		

	"SYNC_INVALID_EL0_64",		
	"IRQ_INVALID_EL0_64",		
	"FIQ_INVALID_EL0_64",		
	"ERROR_INVALID_EL0_64",	

	"SYNC_INVALID_EL0_32",		
	"IRQ_INVALID_EL0_32",		
	"FIQ_INVALID_EL0_32",		
	"ERROR_INVALID_EL0_32"	
};

void show_invalid_entry_message(s32 type, u32 esr, u32 address) 
{
	LOG_ERROR("%s, ESR: %x, address, %x", entry_error_messages[type], esr, address);
}

void kernel_main(void)
{
	init_printf(0, putc);
	irq_init();
	LOG_DEBUG("Hello from Kernel\r\n");
    while (1) 
	{
        LOG_DEBUG("Hello from Kernel\r\n");
        delay(500);
    }
}
