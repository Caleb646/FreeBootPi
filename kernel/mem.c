#include "mem.h"
#ifdef ON_HOST_TESTING
extern u8 low_heap[TEST_HEAP_SIZE];
extern u8 mid_heap[TEST_HEAP_SIZE];
extern u8 high_heap[TEST_HEAP_SIZE];
#else
#include "printf.h"
#include "sync.h"
#endif

static heap_s heaps[MEM_NUM_HEAP_SECTIONS];

static s32 find_heap (void* ptr) {
    uintptr_t ptr_addr = (uintptr_t)ptr;
    for (u32 i = 0; i < MEM_NUM_HEAP_SECTIONS; ++i) {
        heap_s heap = heaps[i];
        if (ptr_addr >= (uintptr_t)heap.start && ptr_addr <= (uintptr_t)heap.end) {
            return i;
        }
    }
    return -1;
}

GCC_NODISCARD static void* align_allocate_ (heap_s* heap, size_t sz, size_t alignment) {
    // Interrupt but NOT thread safe
    s32 status = enter_critical (IRQ_DISABLED_FIQ_DISABLED_TARGET);
    if (status == -1) {
        return NULLPTR;
    }
    node_s* prev  = NULLPTR;
    node_s** cur  = &(heap->free_list_head);
    node_s* alloc = NULLPTR;
    while ((*cur) != NULLPTR) {
        /*
         * TODO: A very large previous allocation in the free list could be used to allocate a small object.
         * Should search for smallest allocation in free list that meets the requirements.
         */
        if ((*cur)->sz >= sz) {
            if ((*cur)->next != NULLPTR && prev != NULLPTR) {
                prev->next = (*cur)->next;
            }
            alloc     = (node_s*)(*cur);
            alloc->sz = (*cur)->sz;
            *cur      = NULLPTR;
            break;
        }
        prev   = (*cur);
        (*cur) = (*cur)->next;
    }
    if (alloc == NULLPTR &&
        ((uintptr_t)heap->end - (uintptr_t)heap->cur_pos) < (sz + sizeof (size_t))) {
        /* Aligning address using modulo */
        // u32 alignment = ((uintptr_t)(heap_current + sizeof(size_t))) % MALLOC_ADDR_ALIGNMENT;
        // heap_current += MALLOC_ADDR_ALIGNMENT - alignment;

        /*
         * Align returned address (&alloc->next) by MALLOC_ADDR_ALIGNMENT (8,
         * 16, 64...). Depending on the alignment the last several least
         * significant bits will not be used. For example, with alignment 16,
         * the 4 least significant bits will not be used. So a mask of ~0x0F
         * anded with the current memory address + (alignment - 1) +
         * sizeof(size_t) returns the aligned address for alloc->next.
         */
        heap->cur_pos =
        (u8*)(((uintptr_t)heap->cur_pos + (uintptr_t)(alignment - 1) + sizeof (size_t)) &
              (uintptr_t)(~(alignment - 1)));
        alloc     = (node_s*)((uintptr_t)heap->cur_pos - sizeof (size_t));
        alloc->sz = sz;
        heap->cur_pos += sz;
    }

    void* ret = (void*)(&(alloc->next));
    status    = leave_critical ();
    return ret;
}

/*
 * \brief
 * \param sz size of allocation to make in bytes
 * \param alignment if alignment == 8, align allocation address on 8 byte
 * boundary. Must be a power of 2.
 */
GCC_NODISCARD void* align_allocate (size_t sz, size_t alignment) {
    return align_allocate_ (&(heaps[MEM_STANDARD_HEAP_ID]), sz, alignment);
}

/*
 * \brief The raspberry pi 4 in low peripheral mode has 3 seperate heap sections. This function
 * allows for the selection of which heap to make the allocation.
 * \param heap_id The id of the heap the allocation will be created in.
 * \param sz size of allocation to make in bytes
 * \param alignment if alignment == 8, align allocation address on 8 byte boundary.
 *   Must be a power of 2.
 */
GCC_NODISCARD void* calign_allocate (heap_id_t heap_id, size_t sz, size_t alignment) {
    if (heap_id < MEM_NUM_HEAP_SECTIONS) {
        return align_allocate_ (&(heaps[heap_id]), sz, alignment);
    }
    return NULLPTR;
}

GCC_NODISCARD void* align_allocate_set (size_t sz, u8 value, size_t alignment) {
    void* ptr = align_allocate (sz, alignment);
    memset (ptr, value, sz);
    return ptr;
}

GCC_NODISCARD void* allocate (size_t sz) {
    return align_allocate (sz, MALLOC_ADDR_ALIGNMENT);
}

void delete (void* ptr) {
    s32 status = enter_critical (IRQ_DISABLED_FIQ_DISABLED_TARGET);
    if (status == -1) {
        return;
    }

    s32 heap_id = find_heap (ptr);
    if (heap_id == -1) {
        LOG_ERROR ("Failed to delete allocation because corresponding heap "
                   "could not be found");
        return;
    }

    heap_s* heap = &(heaps[heap_id]);

    node_s* node      = (node_s*)(((u8*)ptr) - sizeof (size_t));
    size_t alloc_size = node->sz;
    if (heap->free_list_head == NULLPTR) {
        heap->free_list_head       = (node_s*)node;
        heap->free_list_head->sz   = alloc_size;
        heap->free_list_head->next = NULLPTR;
        return;
    }
    /*
     * TODO: Should join free_list entries together
     */
    node_s* cur = heap->free_list_head;
    while (cur != NULLPTR) {
        if (cur->next == NULLPTR) {
            cur->next       = (node_s*)ptr;
            cur->next->sz   = alloc_size;
            cur->next->next = NULLPTR;
            return;
        }
        cur = cur->next;
    }
    status = leave_critical ();
}


/**************************** Setup MMU *******************************************/
#define MMU_LEVEL2_ENTRIES                    128
#define MMU_LEVEL2_PAGE_SIZE                  0x1000 // 4KB
#define MMU_LEVEL3_ENTRIES                    8192
#define MMU_LEVEL3_PAGE_SIZE                  0x10000 // 64KB

#define MMU_BLOCK_DESC_TYPE                   0x1
#define MMU_TBL_DESC_TYPE                     0x3

/*
 * Granule size is 64KB so m is 16.
 * pg 2566 of arm reference manual
 */
#define MMU_GRANULE_SIZE_M                    16
#define LEVEL2_TBL_ADDR(addr)                 ((addr >> MMU_GRANULE_SIZE_M) & 0xFFFFFFFF)
/*
 * Core sends 64-bit virtual address:
 *   Bits [(m-1):0] of the given virtual address are the bottom [(m-1):0] bits for the translated physical address.
 *   Level 3 Page Entry holds the upper bits [47:m] for the translated physical address.
 *
 *   So the physical address given to the lvl 3 page entry needs to be shifted right m places so
 *   it will be in the correct position when added together to create the translated physical address.
 */
#define LEVEL3_PHYS_ADDR(addr)                ((addr >> MMU_GRANULE_SIZE_M) & 0xFFFFFFFF)

/*
 * mair_el1 Register: https://developer.arm.com/documentation/ddi0601/2024-03/AArch64-Registers/MAIR-EL1--Memory-Attribute-Indirection-Register--EL1-?lang=en
 * Device memory attributes: https://developer.arm.com/documentation/den0024/a/Memory-Ordering/Memory-types/Device-memory
 */
/*
 * NON Gathering, NON Re-ordering, NON Early Write Acknowledgement
 */
#define MAIR_DEVICE_NGNRNE                    (0b00 << 2)
/*
 * NON Gathering, NON Re-ordering, Early Write Acknowledgement
 */
#define MAIR_DEVICE_NGNRE                     (0b01 << 2)
/*
 * NON Gathering, Re-ordering, Early Write Acknowledgement
 */
#define MAIR_DEVICE_NGRE                      (0b10 << 2)
/*
 * Gathering, Re-ordering, Early Write Acknowledgement
 */
#define MAIR_DEVICE_GRE                       (0b11 << 2)
/*
 * https://developer.arm.com/documentation/ddi0601/2024-03/AArch64-Registers/MAIR-EL1--Memory-Attribute-Indirection-Register--EL1-?lang=en
 */
#define MAIR_NORMAL_OUTERW_THROUGH_TRANSIENT  (0b0011 << 4)
#define MAIR_NORMAL_OUTER_NO_CACHE            (0b0100 << 4)
#define MAIR_NORMAL_OUTERW_BACK_TRANSIENT     (0b0111 << 4)
#define MAIR_NORMAL_OUTERW_BACK_NON_TRANSIENT (0b1111 << 4)

#define MAIR_NORMAL_INNERW_BACK_NON_TRANSIENT (0b1111 << 0)

#define MAIR_NORMAL_ATTR_IDX                  0
#define MAIR_DEVICE_ATTR_IDX                  1
#define MAIR_COHERENT_ATTR_IDX                2

// clang-format off
/*
 * pg 2566 of arm reference manual
 */
typedef struct tbl_desc_lvl2 {
    u64 entry_type : 2, 
    ignored : (1 + 15 - 2),
    /*
    * Bits[47:m] are bits[47:m] of the address of the required next-level table, which is:
    *   For a level 1 Table descriptor, the address of a level 2 table.
    *   For a level 2 Table descriptor, the address of a level 3 table
    * Bits[15:0] of the table address are zero.
    */ 
    tbl_addr : (1 + 47 - MMU_GRANULE_SIZE_M),
    ignored2 : (1 + 58 - 48),
    pxn_tbl : (1 + 59 - 59),
    xn_tbl  : (1 + 60 - 60), // XN limit for subsequent levels of lookup
    ap_tbl  : (1 + 62 - 61), // Access permissions limit for subsequent levels of lookup -- pg 2583 arm reference manual
    ns_tbl  : (1 + 63 - 63); // For memory accesses from Secure state, specifies the Security state for subsequent levels of lookup
} tbl_desc_lvl2;

/*
 * pg 2569 of arm reference manual
 */
typedef struct page_desc_lvl3 {
    u64 entry_type  : (1 + 1 - 0),
    attr_idx	    : (1 + 4 - 2),	    // mair_el1
    ns		        : (1 + 5 - 5),	    // Non-secure bit -- set to 0
#define AP_EL1_READ_WRITE   0b00 // el1 read and write
#define AP_ALL_READ_WRITE   0b01 // el1 and el0 read and write
#define AP_El1_READ_ONLY    0b10 // el1 read only
#define AP_ALL_READ_ONLY    0b11 // el1 and el0 read only
    ap		        : (1 + 7 - 6),	    // Data Access Permissions bits -- pg 2581 arm reference manual
#define SH_NORMAL_MEM_NO_SHARE      0b00 // Non-shareable
#define SH_NORMAL_MEM_OUTER_SHARE   0b10 // Outer Shareable
#define SH_NORMAL_MEM_INNER_SHARE   0b11 // Inner Shareable
    sh		        : (1 + 9 - 8),	    // Shareability field -- pg 2600 arm reference manual
    af		        : (1 + 10 - 10),    // The Access flag -- set to 1 pg 2590 arm reference manual
    ng		        : (1 + 11 - 11),    // The not global bit -- set to 0
    ta	            : (1 + 15 - 12),    // TA[51:48] indicates bits[51:48] of the page address.
    tbl_addr        : (1 + 47 - MMU_GRANULE_SIZE_M),
    reserved1	    : (1 + 50 - 48),	// set to 0
    dbm             : (1 + 51 - 51),    // Dirty Bit Modifier
    contiguous	    : (1 + 52 - 52),	// A hint bit indicating that the translation table entry is one of a contiguous set or entries -- set to 0
    pxn		        : (1 + 53 - 53),	// The Privileged execute-never field -- set to 0, 1 for device memory
    uxn		        : (1 + 54 - 54),	// The Execute-never or Unprivileged execute-never field -- set to 1
    ignored0		: 9;	            // set to 0

} page_desc_lvl3;
// clang-format on

uintptr_t create_translation_tbl_lvl3_ (uintptr_t addr) {
}

void translation_tbl_init (void) {
    uintptr_t addr = 0x0;
    tbl_desc_lvl2* table_ptr =
    (tbl_desc_lvl2*)calign_allocate (MEM_LOW_HEAP_ID, MMU_LEVEL2_PAGE_SIZE, MMU_LEVEL2_PAGE_SIZE);
    memset (table_ptr, 0, MMU_LEVEL2_PAGE_SIZE);

    for (uintptr_t entry_idx = 0; entry_idx < MMU_LEVEL2_ENTRIES; ++entry_idx) {
        addr = entry_idx * MMU_LEVEL3_ENTRIES * MMU_LEVEL3_PAGE_SIZE;
    }
}

s32 mem_init (heap_s* hp) {
    if (hp == NULLPTR) {
        //#ifndef ON_HOST_TESTING
        heap_s low              = { .start   = (u8*)ARM_DRAM_LOW_MEM_START,
                                    .end     = (u8*)ARM_DRAM_LOW_MEM_END,
                                    .cur_pos = (u8*)ARM_DRAM_LOW_MEM_START,
                                    .size    = ARM_DRAM_LOW_MEM_END - ARM_DRAM_LOW_MEM_START,
                                    .free_list_head = NULLPTR };
        heap_s mid              = { .start   = (u8*)ARM_DRAM_MID_MEM_START,
                                    .end     = (u8*)ARM_DRAM_MID_MEM_END,
                                    .cur_pos = (u8*)ARM_DRAM_MID_MEM_START,
                                    .size    = ARM_DRAM_MID_MEM_END - ARM_DRAM_MID_MEM_START,
                                    .free_list_head = NULLPTR };
        heap_s high             = { .start   = (u8*)ARM_DRAM_HIGH_MEM_START,
                                    .end     = (u8*)ARM_DRAM_HIGH_MEM_END,
                                    .cur_pos = (u8*)ARM_DRAM_HIGH_MEM_START,
                                    .size = ARM_DRAM_HIGH_MEM_END - ARM_DRAM_HIGH_MEM_START,
                                    .free_list_head = NULLPTR };
        heaps[MEM_LOW_HEAP_ID]  = low;
        heaps[MEM_MID_HEAP_ID]  = mid;
        heaps[MEM_HIGH_HEAP_ID] = high;
        //#endif
    }
    return 1;
}