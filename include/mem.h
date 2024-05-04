#ifndef	_MEM_H
#define	_MEM_H

#include "base.h"

/*
* Full 35-bit address map
*   Seen by both "large address" masters (e.g. the DMA4 engines) and the ARM CPU.
*   Two L2 cache aliases (one allocating, one non-allocating) which cache (only) the first 1GB of SDRAM.
*/

/* 
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
*/      
#define ARM_DRAM_LOW_MEM_START  0x0
#define ARM_DRAM_LOW_MEM_END    VC_DRAM_MEM_START
#define ARM_DRAM_MID_MEM_START  0x040000000 /* Top of SDRAM for the Video Core */
#define ARM_DRAM_MID_MEM_END    0x0FC000000 /* Start of the Main Peripherals */
#define ARM_DRAM_HIGH_MEM_START 0x100000000
#define ARM_DRAM_HIGH_MEM_END   0x400000000 /* 1.6 GBs */

#define MALLOC_ADDR_ALIGNMENT        64 /* Size of cache line */
#define MALLOC_ADDR_ALIGNMENT_MASK   (~(MALLOC_ADDR_ALIGNMENT - 1)) /* Size of cache line */

#ifndef __ASSEMBLER__

/* Do NOT adjust order of node_s members*/
typedef struct node_s
{
    size_t sz;
    struct node_s* next;
} node_s;

typedef struct heap_s
{
    u8 const* start;
    u8 const* end;
    u8* cur_pos;
    u64 size;
    node_s* free_list_head;
} heap_s;

void* malloc(size_t sz);
void free(void* ptr);
s32 mem_init(heap_s*);


#endif // __ASSEMBLER__

#endif  /*_MEM_H */