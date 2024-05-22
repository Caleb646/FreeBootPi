#ifndef _SYNC_H
#define _SYNC_H

#include "base.h"

#define IRQ_DISABLED_FIQ_DISABLED_TARGET 0x0
#define IRQ_DISABLED_FIQ_ENABLED_TARGET  0x1


#ifndef __ASSEMBLER__

/* NSH --- Operation only out to the point of unification (i.e. visible to this core)  */
#define DATA_MEMORY_BARRIER_NOSHARE_ANY() asm volatile("dmb NSH" ::: "memory")
/* ISH --- Operation only to the Inner Shareable domain (i.e. visible to all or some cores) */
#define DATA_MEMORY_BARRIER_INNER_ANY()   asm volatile("dmb ISH" ::: "memory")
/* Operation only to the outer shareable domain and waits only for stores to complete*/
#define DATA_MEMORY_BARRIER_OUTER_STORES() \
    asm volatile("dmb ISHST" ::: "memory")

#define DATA_MEMORY_BARRIER_FS_STORES() asm volatile("dmb ST" ::: "memory")
#define DATA_MEMORY_BARRIER_FS_LOADS()  asm volatile("dmb LD" ::: "memory")

#define DATA_SYNC_BARRIER_FS_ANY()      asm volatile("dsb SY" ::: "memory")
#define DATA_SYNC_BARRIER_FS_STORES()   asm volatile("dsb ST" ::: "memory")
#define DATA_SYNC_BARRIER_FS_LOADS()    asm volatile("dsb LD" ::: "memory")

#define ISB()                           asm volatile("isb" ::: "memory")


void cache_clean (void);
void cache_invalidate (void);
void cache_clean_invalidate (void);

s32 enter_critical (u32 target_lvl);
s32 leave_critical (void);

#endif // __ASSEMBLER__

#endif /*_SYNC_H */