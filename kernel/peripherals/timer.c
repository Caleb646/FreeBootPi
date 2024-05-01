#include "peripherals/timer.h"
#include "printf.h"
#include "irq.h"

extern timer_s timer = {};

u32 get_arm_local_timer_freq(void)
{
    /* Bits [31:0] --- System counter clock frequency in Hz. */
    u64 freq;
	asm volatile ("mrs %0, CNTFRQ_EL0" : "=r" (freq));
    freq &= 0xFFFFFFFF;
    if(freq == 0)
    {
        LOG_ERROR("System Clock Frequency is NOT set");
    }
    return freq;
}


s32 init_timer(timer_s* t)
{
    
    return 1;
}