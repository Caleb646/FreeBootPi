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

void sys_timer_irq_handler(u32 irq_id)
{
    LOG_INFO("Received Timer IRQ [%u]", irq_id);
}


s32 timer_init(timer_s* t)
{
    gic_enable_interrupt(VC_GIC_SYSTEM_TIMER_IRQ_3, &sys_timer_irq_handler);
    return 1;
}