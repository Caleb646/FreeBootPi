#ifndef	_P_TIMER_H
#define	_P_TIMER_H

#include "base.h"

#define VC_SYSTEM_TIMER_BASE        (PBASE + 0x2003000)
// CS --- This register is used to record and clear timer channel comparator matches. The system timer match bits are routed
// to the interrupt controller where they can generate an interrupt.
#define VC_SYSTEM_TIMER_CS          (TIMER_BASE + 0x00) // 32 bits -- System Timer Control/Status
// The M0-3 fields contain the free-running counter match status. Write a one to the relevant bit to clear the match
// detect status bit and the corresponding interrupt request line.
#define VC_SYSTEM_TIMER_CS_M0_BIT   (1 << 0)
#define VC_SYSTEM_TIMER_CS_M1_BIT   (1 << 1)
#define VC_SYSTEM_TIMER_CS_M2_BIT   (1 << 2)
#define VC_SYSTEM_TIMER_CS_M3_BIT   (1 << 3)
#define VC_SYSTEM_TIMER_CLO         (TIMER_BASE + 0x04) // 32 bits -- System Timer Counter Lower 32 bits
#define VC_SYSTEM_TIMER_CHI         (TIMER_BASE + 0x08) // 32 bits -- System Timer Counter Higher 32 bits
// C0, C1, C2, and C3 --- The system timer compare registers hold the compare value for each of the four timer channels. 
// Whenever the lower 32-bits of the free-running counter matches one of the compare values 
// the corresponding bit in the system timer control/status register is set.
#define VC_SYSTEM_TIMER_C0          (TIMER_BASE + 0x0C) // 32 bits -- System Timer Compare 0
#define VC_SYSTEM_TIMER_C1          (TIMER_BASE + 0x10) // 32 bits -- System Timer Compare 1
#define VC_SYSTEM_TIMER_C2          (TIMER_BASE + 0x14) // 32 bits -- System Timer Compare 2
#define VC_SYSTEM_TIMER_C3          (TIMER_BASE + 0x18) // 32 bits -- System Timer Compare 3

#define TICKS_PER_SECOND            100

typedef struct timer_s
{

} timer_s;

extern timer_s timer;

s32 init_timer(timer_s*);
u64 get_sys_time_s(void);

void sys_timer_update_irq_handler(u32 irq_id);

u32 get_arm_local_timer_freq(void);

#endif  /*_P_TIMER_H */