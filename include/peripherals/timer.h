#ifndef	_P_TIMER_H
#define	_P_TIMER_H

#include "base.h"

#define VC_SYSTEM_TIMER_BASE        (PBASE + 0x2003000)
// CS --- This register is used to record and clear timer channel comparator matches. The system timer match bits are routed
// to the interrupt controller where they can generate an interrupt.
#define VC_SYSTEM_TIMER_CS          (VC_SYSTEM_TIMER_BASE + 0x00) // 32 bits -- System Timer Control/Status
// The M0-3 fields contain the free-running counter match status. Write a one to the relevant bit to clear the match
// detect status bit and the corresponding interrupt request line.
#define VC_SYSTEM_TIMER_CS_M0_BIT   (1 << 0)
#define VC_SYSTEM_TIMER_CS_M1_BIT   (1 << 1)
#define VC_SYSTEM_TIMER_CS_M2_BIT   (1 << 2)
#define VC_SYSTEM_TIMER_CS_M3_BIT   (1 << 3)
#define VC_SYSTEM_TIMER_CLO         (VC_SYSTEM_TIMER_BASE + 0x04) // 32 bits -- System Timer Counter Lower 32 bits
#define VC_SYSTEM_TIMER_CHI         (VC_SYSTEM_TIMER_BASE + 0x08) // 32 bits -- System Timer Counter Higher 32 bits
// C0, C1, C2, and C3 --- The system timer compare registers hold the compare value for each of the four timer channels. 
// Whenever the lower 32-bits of the free-running counter matches one of the compare values 
// the corresponding bit in the system timer control/status register is set.
// NOTE: Timers 0 & 2 are reserved for the GPU.
// 1 & 3 are free.
#define VC_SYSTEM_TIMER_C0          (VC_SYSTEM_TIMER_BASE + 0x0C) // 32 bits -- System Timer Compare 0
#define VC_SYSTEM_TIMER_C1          (VC_SYSTEM_TIMER_BASE + 0x10) // 32 bits -- System Timer Compare 1
#define VC_SYSTEM_TIMER_C2          (VC_SYSTEM_TIMER_BASE + 0x14) // 32 bits -- System Timer Compare 2
#define VC_SYSTEM_TIMER_C3          (VC_SYSTEM_TIMER_BASE + 0x18) // 32 bits -- System Timer Compare 3

/* 96 through 99 */
#define VC_SYSTEM_TIMER_IRQID_TO_CO_REG(irq_id) (VC_SYSTEM_TIMER_C0 + (irq_id - VC_GIC_BASE_IRQ) * sizeof(u32))
/* Bits 0, 1, 2, 3 are for acking timers 0, 1, 2, 3 */
#define VC_SYSTEM_TIMER_IRQID_TO_MBIT(irq_id) (1 << (irq_id - VC_GIC_BASE_IRQ))
#define VC_SYSTEM_CLOCK_HZ          1000000
#define TICKS_PER_SECOND            100

#define SECONDS_TO_MS(seconds)      (seconds * 1000)
#define MS_TO_SECONDS(ms)           (ms / 1000)
#define SECONDS_TO_US(seconds)      (seconds * 1000000)
#define US_TO_SECONDS(us)           (us / 1000000)

typedef struct timer_s
{
    u64 cur_ticks;
    u64 cur_seconds;
    u32 irq_interval;
    u32 irq_id;
} timer_s;

//extern timer_s timer;

s32 timer_init(u32);
u64 get_sys_time_ms(void);
void wait_ms(u32);

void sys_timer_irq_handler(u32 irq_id);

u32 get_arm_local_timer_freq(void);

#endif  /*_P_TIMER_H */