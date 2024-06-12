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
#define DATA_MEMORY_BARRIER_INNER_STORES() \
    asm volatile("dmb ISHST" ::: "memory")
/* Operation only to the outer shareable domain and waits only for stores to complete*/
#define DATA_MEMORY_BARRIER_OUTER_STORES() \
    asm volatile("dmb OSHST" ::: "memory")

#define DATA_MEMORY_BARRIER_FS_ANY()    asm volatile("dmb SY" ::: "memory")
#define DATA_MEMORY_BARRIER_FS_STORES() asm volatile("dmb ST" ::: "memory")
#define DATA_MEMORY_BARRIER_FS_LOADS()  asm volatile("dmb LD" ::: "memory")

#define DATA_SYNC_BARRIER_FS_ANY()      asm volatile("dsb SY" ::: "memory")
#define DATA_SYNC_BARRIER_FS_STORES()   asm volatile("dsb ST" ::: "memory")
#define DATA_SYNC_BARRIER_FS_LOADS()    asm volatile("dsb LD" ::: "memory")

#define ISB()                           asm volatile("isb" ::: "memory")


void cache_clean (void);
void cache_invalidate (void);
void cache_clean_invalidate (void);

void clean_data_cache_vaddr (uintptr_t vaddr, u64 size);
void invalidate_data_cache_vaddr (uintptr_t vaddr, u64 size);
void clean_invalidate_data_cache_vaddr (uintptr_t vaddr, u64 size);

typedef enum critical_section_target_t {
    eCRITICAL_SECTION_TARGET_DISABLE_IRQ,
    eCRITICAL_SECTION_TARGET_DISABLE_IRQ_FIQ
} critical_section_target_t;
bool enter_critical (critical_section_target_t target_lvl);
void leave_critical (void);

#endif // __ASSEMBLER__

#endif /*_SYNC_H */