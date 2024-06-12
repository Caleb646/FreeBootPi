#include "timer.h"
#include "irq.h"
#include "sync.h"

static timer_s volatile timer_ = { 0 };

u32 get_arm_local_timer_freq (void) {
    /* Bits [31:0] --- System counter clock frequency in Hz. */
    u64 freq;
    asm volatile("mrs %0, CNTFRQ_EL0" : "=r"(freq));
    freq &= 0xFFFFFFFF;
    if (freq == 0) {
        LOG_ERROR ("System Clock Frequency is NOT set");
    }
    return freq;
}

static void sys_timer_irq_handler (u32 irq_id, void* context) {
    // LOG_INFO("Received Timer IRQ [%u]", timer.cur_ticks + 1);
    /* Update compare register */
    write32 (
    VC_SYSTEM_TIMER_IRQID_TO_CO_REG (irq_id),
    read32 (VC_SYSTEM_TIMER_CLO) + timer_.irq_interval);

    /* Acknowledge timer status */
    write32 (VC_SYSTEM_TIMER_CS, read32 (VC_SYSTEM_TIMER_CS) | VC_SYSTEM_TIMER_IRQID_TO_MBIT (irq_id));

    if (enter_critical (eCRITICAL_SECTION_TARGET_DISABLE_IRQ_FIQ)) {
        if ((++timer_.cur_ticks % TIMER_UPDATES_PER_SECOND) == 0) {
            ++timer_.cur_seconds;
        }
        leave_critical ();
    }
}

s32 timer_init (u32 timer_irq_id) {
    if (timer_irq_id != VC_GIC_SYSTEM_TIMER_IRQ_1 && timer_irq_id != VC_GIC_SYSTEM_TIMER_IRQ_3) {
        LOG_ERROR ("Only System Timers 1 (97) and 3 (99) are free. Not [%u]", timer_irq_id);
        return 0;
    }
    /* Initialize Timer Struct */
    timer_.cur_ticks    = 0;
    timer_.cur_seconds  = 0;
    timer_.irq_interval = VC_SYSTEM_CLOCK_HZ / TIMER_UPDATES_PER_SECOND;
    timer_.irq_id       = timer_irq_id;

    /* Enable Timer Interrupt on GIC and Set Handler */
    gic_enable_interrupt (timer_irq_id, &sys_timer_irq_handler, NULLPTR);

    /* Put new updated compare value in timer compare register */
    write32 (
    VC_SYSTEM_TIMER_IRQID_TO_CO_REG (timer_irq_id),
    read32 (VC_SYSTEM_TIMER_CLO) + timer_.irq_interval);
    return 1;
}

u64 get_sys_time_s (void) {
    return timer_.cur_seconds;
}

u64 get_sys_time_ms (void) {
    return get_sys_time_s () * 1000;
}

void wait_us (u32 us) {
    u32 cticks = read32 (VC_SYSTEM_TIMER_CLO);
    u32 nticks = cticks + ((VC_SYSTEM_CLOCK_HZ / 1000000) * us);
overflow:
    /* If the current counter plus the necessary ticks needed overflows */
    if (nticks < cticks) {
        /* Wait until CLO overflows */
        while (nticks < read32 (VC_SYSTEM_TIMER_CLO)) {
        }
        /* Wait until CLO is greater than nticks*/
        while (nticks > read32 (VC_SYSTEM_TIMER_CLO)) {
        }
    }
    /* If close to overflowing go ahead and overflow */
    else if ((nticks + 100000) < cticks) {
        nticks += 100000;
        goto overflow;
    } else {
        while (read32 (VC_SYSTEM_TIMER_CLO) < nticks) {
        }
    }
}

void wait_ms (u32 ms) {
    wait_us (ms * 1000);
}

void wait_s (u32 s) {
    wait_ms (s * 1000);
}