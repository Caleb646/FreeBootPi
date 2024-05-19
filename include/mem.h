#ifndef _MEM_H
#define _MEM_H

#include "base.h"

#ifdef ON_HOST_TESTING
#define TEST_HEAP_SIZE 512
extern u8 low_heap[TEST_HEAP_SIZE];
extern u8 mid_heap[TEST_HEAP_SIZE];
extern u8 high_heap[TEST_HEAP_SIZE];
#define ARM_DRAM_LOW_MEM_START  &(low_heap[0])
#define ARM_DRAM_LOW_MEM_END    &(low_heap[TEST_HEAP_SIZE])
#define ARM_DRAM_MID_MEM_START  &(mid_heap[0])
#define ARM_DRAM_MID_MEM_END    &(mid_heap[TEST_HEAP_SIZE])
#define ARM_DRAM_HIGH_MEM_START &(high_heap[0])
#define ARM_DRAM_HIGH_MEM_END   &(high_heap[TEST_HEAP_SIZE])
#else
/*
 * Full 35-bit address map
 *   Seen by both "large address" masters (e.g. the DMA4 engines) and the ARM CPU.
 *   Two VPU L2 cache aliases (one allocating, one non-allocating) which cache (only) the first 1GB of SDRAM.
 *
 * Video Core and ARM DRAM Address Spaces in Low Peripheral Mode
 * for Raspberry Pi 4b with 8 GBs.
 * Layers in order from lowest to highest address:
 *   ARM DRAM LOW
 *   VC DRAM
 *   ARM DRAM MID
 *   ARM DRAM HIGH
 */
#define VC_DRAM_MEM_END         0x040000000
/* Assume VC DRAM is no larger than 256 MBs (0xF424000) */
#define VC_DRAM_MEM_START       (VC_DRAM_MEM_END - 0xF424000)
/*
 * Low DRAM for ARM is where the Kernel starts (0x80000)
 * Below this is where armstub put spin table for the secondary cores.
 */
#define ARM_DRAM_LOW_MEM_START  0x01000
#define ARM_DRAM_LOW_MEM_END    KERNEL_LOAD_ADDR
#define ARM_DRAM_MID_MEM_START  0x040000000 /* Top of SDRAM for the Video Core */
#define ARM_DRAM_MID_MEM_END    0x0FC000000 /* Start of the Main Peripherals */
/*
 * High mem requires a 35 bit addresses
 */
#define ARM_DRAM_HIGH_MEM_START 0x100000000
#define ARM_DRAM_HIGH_MEM_END   0x400000000 /* 1.6 GBs */

#endif

/* Size of cache line */
#define MALLOC_ADDR_ALIGNMENT      64
#define MALLOC_ADDR_ALIGNMENT_MASK (~(MALLOC_ADDR_ALIGNMENT - 1))

#define MEM_NUM_HEAP_SECTIONS      3

#ifndef __ASSEMBLER__

typedef enum heap_id_t {
    MEM_LOW_HEAP_ID      = 0,
    MEM_MID_HEAP_ID      = 1,
    MEM_HIGH_HEAP_ID     = 2,
    MEM_STANDARD_HEAP_ID = MEM_MID_HEAP_ID
} heap_id_t;

/* Do NOT adjust order of node_s members*/
typedef struct node_s {
    size_t sz;
    struct node_s* next;
} node_s;

typedef struct heap_s {
    u8 const* start;
    u8 const* end;
    u8* cur_pos;
    u64 size;
    node_s* free_list_head;
} heap_s;

GCC_NODISCARD void* align_allocate (size_t sz, size_t alignment);
GCC_NODISCARD void* calign_allocate (heap_id_t heap_id, size_t sz, size_t alignment);
GCC_NODISCARD void* align_allocate_set (size_t sz, u8 value, size_t alignment);
GCC_NODISCARD void* allocate (size_t sz);

void delete (void* ptr);
s32 mem_init (heap_s*);


#endif // __ASSEMBLER__

#endif /*_MEM_H */