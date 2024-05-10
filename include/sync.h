#ifndef	_SYNC_H
#define	_SYNC_H

#include "base.h"

/********************** ARM Sync ***************************/
/* Operation only to the outer shareable domain and waits only for stores to complete*/
#define DATA_MEMORY_BARRIER_OUTER_STORES()         asm volatile("dmb ISHST" ::: "memory")
/* ISH --- Operation only to the Inner Shareable domain (i.e. visible to all or some cores) */
#define DATA_MEMORY_BARRIER_INNER_ANY()         asm volatile("dmb ISH" ::: "memory")
/* NSH --- Operation only out to the point of unification (i.e. visible to this core)  */
#define DATA_MEMORY_BARRIER_NOSHARE_ANY()       asm volatile("dmb NSH" ::: "memory")

#define IRQ_DISABLED_FIQ_DISABLED_TARGET        0x0
#define IRQ_DISABLED_FIQ_ENABLED_TARGET         0x1



#ifndef __ASSEMBLER__

s32 enter_critical(u32 target_lvl);
s32 leave_critical(void);

#endif // __ASSEMBLER__

#endif  /*_SYNC_H */