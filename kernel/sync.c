#include "sync.h"
#include "irq.h"


static u32 volatile s_ncritical_lvl[ARM_NUM_CORES];
static u64 volatile s_int_flags[ARM_NUM_CORES][MAX_NESTED_INTERRUPTS];

s32 enter_critical(u32 target_lvl)
{
	u64 flags = get_daif_flags();
	u64 core_id = get_arm_core_id();
	if(s_ncritical_lvl[core_id] >= MAX_NESTED_INTERRUPTS)
	{
		return 0;
	}
    DISABLE_IRQ_FIQ();
	// Save DAIF flags to be restored after leaving interrupt
	s_int_flags[core_id][s_ncritical_lvl[core_id]++] = flags;
	
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