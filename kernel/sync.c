#include "sync.h"
#include "irq.h"

#define L1_DCACHE_SETS        256
#define L1_DCACHE_WAYS        2
/*
 * https://developer.arm.com/documentation/ddi0601/2020-12/AArch64-Instructions/DC-ISW--Data-or-unified-Cache-line-Invalidate-by-Set-Way
 *
 * Bits [63:32] are reserved
 * Bits [31:32 - A] are the way index to operate on
 *	A = 32 - Log2(ASSOCIATIVITY) where ASSOCIATIVITY is the number of
 *	ways in the cache
 * Bits [B - 1:L] are the set index to operate on
 *	L = Log2(cache line length in bytes)
 *	B = (L + S)
 *		S = Log2(number of sets in the cache)
 * Bits [L - 1:4] are reserved
 * Bits [3:1] determine the cache lvl. 0 = L1, 1 = L2, 2 = L3....
 * Bit  [0] is reserved
 */
#define L1_SW_WAY_SHIFT       31 // A = 32 - Log2(L1 number of ways in the cache)
#define L1_DCACHE_LINE_LENGTH 64
#define L1_SW_SET_SHIFT       6 // L = Log2(L1 cache line length in bytes)
#define L1_LEVEL              (0 << 1)

#define L2_CACHE_SETS         1024
#define L2_CACHE_WAYS         16
#define L2_SW_WAY_SHIFT       28 // A = 32 - Log2(L2 number of ways in the cache)
#define L2_CACHE_LINE_LENGTH  64
#define L2_SW_SET_SHIFT       6 // L = Log2(L2 cache line length in bytes)
#define L2_LEVEL              (1 << 1)


void cache_invalidate (void) {
    for (u32 set_idx = 0; set_idx < L1_DCACHE_SETS; ++set_idx) {
        for (u32 way_idx = 0; way_idx < L1_DCACHE_WAYS; ++way_idx) {
            u64 set_way_idx = (set_idx << L1_SW_SET_SHIFT) |
                              (way_idx << L1_SW_WAY_SHIFT) | L1_LEVEL;
            /*
             * isw = Invalidate by Set/Way
             */
            asm volatile("dc isw, %0" ::"r"(set_way_idx) : "memory");
        }
    }

    for (u32 set_idx = 0; set_idx < L2_CACHE_SETS; ++set_idx) {
        for (u32 way_idx = 0; way_idx < L2_CACHE_WAYS; ++way_idx) {
            u64 set_way_idx = (set_idx << L2_SW_SET_SHIFT) |
                              (way_idx << L2_SW_WAY_SHIFT) | L2_LEVEL;

            asm volatile("dc isw, %0" ::"r"(set_way_idx) : "memory");
        }
    }

    DATA_SYNC_BARRIER_FS_ANY ();
}

void cache_clean (void) {
    for (u32 set_idx = 0; set_idx < L1_DCACHE_SETS; ++set_idx) {
        for (u32 way_idx = 0; way_idx < L1_DCACHE_WAYS; ++way_idx) {
            u64 set_way_idx = (set_idx << L1_SW_SET_SHIFT) |
                              (way_idx << L1_SW_WAY_SHIFT) | L1_LEVEL;
            /*
             * csw = Push to main memory by Set/Way
             */
            asm volatile("dc csw, %0" ::"r"(set_way_idx) : "memory");
        }
    }

    for (u32 set_idx = 0; set_idx < L2_CACHE_SETS; ++set_idx) {
        for (u32 way_idx = 0; way_idx < L2_CACHE_WAYS; ++way_idx) {
            u64 set_way_idx = (set_idx << L2_SW_SET_SHIFT) |
                              (way_idx << L2_SW_WAY_SHIFT) | L2_LEVEL;

            asm volatile("dc csw, %0" ::"r"(set_way_idx) : "memory");
        }
    }

    DATA_SYNC_BARRIER_FS_ANY ();
}

void cache_clean_invalidate (void) {
    for (u32 set_idx = 0; set_idx < L1_DCACHE_SETS; ++set_idx) {
        for (u32 way_idx = 0; way_idx < L1_DCACHE_WAYS; ++way_idx) {
            u64 set_way_idx = (set_idx << L1_SW_SET_SHIFT) |
                              (way_idx << L1_SW_WAY_SHIFT) | L1_LEVEL;
            /*
             * cisw = Clean and invalidate by Set/Way
             */
            asm volatile("dc cisw, %0" ::"r"(set_way_idx) : "memory");
        }
    }

    for (u32 set_idx = 0; set_idx < L2_CACHE_SETS; ++set_idx) {
        for (u32 way_idx = 0; way_idx < L2_CACHE_WAYS; ++way_idx) {
            u64 set_way_idx = (set_idx << L2_SW_SET_SHIFT) |
                              (way_idx << L2_SW_WAY_SHIFT) | L2_LEVEL;

            asm volatile("dc cisw, %0" ::"r"(set_way_idx) : "memory");
        }
    }

    DATA_SYNC_BARRIER_FS_ANY ();
}

void clean_data_cache_vaddr (uintptr_t vaddr, u64 size) {
    while (1) {
        asm volatile("dc cvac, %0" ::"r"(vaddr) : "memory");
        if (size <= L2_CACHE_LINE_LENGTH) {
            break;
        }
        vaddr += L2_CACHE_LINE_LENGTH;
        size -= L2_CACHE_LINE_LENGTH;
    }
    DATA_SYNC_BARRIER_FS_ANY ();
}

void invalidate_data_cache_vaddr (uintptr_t vaddr, u64 size) {
    while (1) {
        asm volatile("dc ivac, %0" ::"r"(vaddr) : "memory");
        if (size <= L2_CACHE_LINE_LENGTH) {
            break;
        }
        vaddr += L2_CACHE_LINE_LENGTH;
        size -= L2_CACHE_LINE_LENGTH;
    }
    DATA_SYNC_BARRIER_FS_ANY ();
}

void clean_invalidate_data_cache_vaddr (uintptr_t vaddr, u64 size) {
    while (1) {
        asm volatile("dc civac, %0" ::"r"(vaddr) : "memory");
        if (size <= L2_CACHE_LINE_LENGTH) {
            break;
        }
        vaddr += L2_CACHE_LINE_LENGTH;
        size -= L2_CACHE_LINE_LENGTH;
    }
    DATA_SYNC_BARRIER_FS_ANY ();
}

static u32 volatile s_ncritical_lvl[ARM_NUM_CORES];
static u64 volatile s_int_flags[ARM_NUM_CORES][MAX_NESTED_INTERRUPTS];

s32 enter_critical (u32 target_lvl) {
    u64 flags   = get_daif_flags ();
    u64 core_id = get_arm_core_id ();
    if (s_ncritical_lvl[core_id] >= MAX_NESTED_INTERRUPTS) {
        return 0;
    }
    DISABLE_IRQ_FIQ ();
    // Save DAIF flags to be restored after leaving interrupt
    s_int_flags[core_id][s_ncritical_lvl[core_id]++] = flags;

    /*
     * Ensure all load & store operations are visible to this core
     * before entering the critical section
     */
    DATA_MEMORY_BARRIER_NOSHARE_ANY ();
    return 1;
}

s32 leave_critical (void) {
    /*
     * Ensure all load & store operations above the leave_critical call
     * are visible to this core before exiting the critical section
     */
    DATA_MEMORY_BARRIER_NOSHARE_ANY ();

    u64 core_id = get_arm_core_id ();
    if (s_ncritical_lvl[core_id] <= 0) {
        return 0;
    }
    // restore saved DAIF flags
    u64 flags = s_int_flags[core_id][--s_ncritical_lvl[core_id]];
    set_daif_flags (flags);
    return 1;
}