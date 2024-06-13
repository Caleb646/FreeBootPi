#include "mem.h"
// #ifdef ON_HOST_TESTING
// extern u8 low_heap[TEST_HEAP_SIZE];
// extern u8 mid_heap[TEST_HEAP_SIZE];
// extern u8 high_heap[TEST_HEAP_SIZE];
// #else
#include "arm/sysregs.h"
#include "peripherals/uart.h"
#include "sync.h"
// #endif

static heap_t heaps_[MEM_NUM_HEAP_SECTIONS];

static s32 find_heap_id_ (void const* ptr);
static bool heap_is_valid_ (heap_t const* heap);
static bool heaps_are_valid_ (heap_t const pheaps[1][MEM_NUM_HEAP_SECTIONS]);
// static uintptr_t align_address_ (uintptr_t addr, u64 alignment);
GCC_NODISCARD static void* align_allocate_ (heap_id_t heap_id, size_t sz, size_t alignment);

static s32 find_heap_id_ (void const* ptr) {
    uintptr_t ptr_addr = (uintptr_t)ptr;
    for (u32 i = 0; i < MEM_NUM_HEAP_SECTIONS; ++i) {
        heap_t const* pheap = &heaps_[i];
        if (ptr_addr >= pheap->start && ptr_addr <= pheap->end) {
            return i;
        }
    }
    return -1;
}

static bool heap_is_valid_ (heap_t const* pheap) {
    if (pheap == NULLPTR) {
        return false;
    }
    return pheap->end > pheap->start;
}

static bool heaps_are_valid_ (heap_t const pheaps[1][MEM_NUM_HEAP_SECTIONS]) {
    for (u32 i = 0; i < MEM_NUM_HEAP_SECTIONS; ++i) {
        if (heap_is_valid_ (&pheaps[0][i]) == false) {
            return false;
        }
        uintptr_t is = pheaps[0][i].start;
        uintptr_t ie = pheaps[0][i].end;
        for (u32 j = i + 1; j < MEM_NUM_HEAP_SECTIONS; ++j) {
            uintptr_t js      = pheaps[0][j].start;
            uintptr_t je      = pheaps[0][j].end;
            bool does_overlap = is >= js && is <= je;
            does_overlap |= ie >= js && ie <= je;
            does_overlap |= js >= is && js <= ie;
            does_overlap |= je >= is && je <= ie;
            if (does_overlap) {
                LOG_ERROR (
                "Invalid heap 1st start [0x%X] end [0x%X] -- 2nd start [0x%X] "
                "end "
                "[0x%X]",
                (u32)is, (u32)ie, (u32)js, (u32)je);
                return false;
            }
        }
    }
    return true;
}

// static uintptr_t align_address_ (uintptr_t addr, u64 nbytes) {
//     if ((addr & (nbytes - 1)) == 0) {
//         return addr;
//     }
//     return (addr + (nbytes - 1)) & ~(nbytes - 1);
// }

GCC_NODISCARD static void* align_allocate_ (heap_id_t heap_id, size_t sz, size_t alignment) {
    ASSERT (heap_id >= 0 && heap_id < MEM_NUM_HEAP_SECTIONS, "");
    // Interrupt but NOT thread safe
    if (enter_critical (eCRITICAL_SECTION_TARGET_DISABLE_IRQ_FIQ) == false) {
        return NULLPTR;
    }
    heap_t* pheap = &heaps_[heap_id];
    node_t* prev  = NULLPTR;
    node_t** cur  = &(pheap->pfree_list_head);
    node_t* alloc = NULLPTR;
    while ((*cur) != NULLPTR) {
        /*
         * TODO: A very large previous allocation in the free list could be used to allocate a small object.
         * Should search for smallest allocation in free list that meets the requirements.
         */
        if ((*cur)->sz >= sz) {
            if ((*cur)->pnext != NULLPTR && prev != NULLPTR) {
                prev->pnext = (*cur)->pnext;
            }
            alloc     = (node_t*)(*cur);
            alloc->sz = (*cur)->sz;
            *cur      = NULLPTR;
            break;
        }
        prev   = (*cur);
        (*cur) = (*cur)->pnext;
    }
    size_t heap_mem_left = pheap->end - (uintptr_t)pheap->pcur_pos;
    if (alloc == NULLPTR && heap_mem_left > (sz + sizeof (size_t))) {
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
        pheap->pcur_pos =
        (u8*)(((uintptr_t)pheap->pcur_pos + (alignment - 1) + sizeof (size_t)) & (~(alignment - 1)));
        alloc     = (node_t*)((uintptr_t)pheap->pcur_pos - sizeof (size_t));
        alloc->sz = sz;
        pheap->pcur_pos += sz;
    }

    void* ret = (void*)(&(alloc->pnext));
    leave_critical ();
    return ret;
}

/*
 * \brief
 * \param sz size of allocation to make in bytes
 * \param alignment if alignment == 8, align allocation address on 8 byte
 * boundary. Must be a power of 2.
 */
GCC_NODISCARD void* align_allocate (size_t sz, size_t alignment) {
    return align_allocate_ (eHEAP_ID_MID, sz, alignment);
}

GCC_NODISCARD void* callocate (heap_id_t heap_id, size_t sz) {
    return align_allocate_ (heap_id, sz, MALLOC_ADDR_ALIGNMENT);
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
    return align_allocate_ (heap_id, sz, alignment);
}

GCC_NODISCARD void* align_allocate_set (size_t sz, u8 value, size_t alignment) {
    void* ptr = align_allocate (sz, alignment);
    if (ptr != NULLPTR) {
        memset (ptr, value, sz);
    }
    return ptr;
}

GCC_NODISCARD void* allocate (size_t sz) {
    return align_allocate (sz, MALLOC_ADDR_ALIGNMENT);
}

GCC_NODISCARD void* align_coherent_allocate (size_t sz, size_t alignment) {
    return calign_allocate (eHEAP_ID_COHERENT, sz, alignment);
}

GCC_NODISCARD void* npage_allocate (heap_id_t heap_id, u64 npages) {
    ASSERT (heap_id >= 0 && heap_id < MEM_NUM_HEAP_SECTIONS, "");
    heap_t* pheap = &heaps_[heap_id];
    void* alloc   = NULLPTR;
    if (enter_critical (eCRITICAL_SECTION_TARGET_DISABLE_IRQ_FIQ)) {
        if (pheap->end - (uintptr_t)pheap->pcur_pos < PAGE_SIZE * npages) {
            LOG_ERROR ("Size [0x%X] is too large for heap [%u]", (u32)(PAGE_SIZE * npages), heap_id);
            return NULLPTR;
        }
        alloc = pheap->pcur_pos;
        pheap->pcur_pos += PAGE_SIZE * npages;
        leave_critical ();
    }
    return alloc;
}

GCC_NODISCARD void* npage_coherent_allocate (u64 npages) {
    return npage_allocate (eHEAP_ID_COHERENT, npages);
}

void delete (void* ptr) {
    if (enter_critical (eCRITICAL_SECTION_TARGET_DISABLE_IRQ_FIQ) == false) {
        return;
    }

    s32 heap_id = find_heap_id_ (ptr);
    ASSERT (heap_id != -1, "");
    heap_t* pheap = &(heaps_[heap_id]);

    node_t* node      = (node_t*)(((u8*)ptr) - sizeof (size_t));
    size_t alloc_size = node->sz;
    if (pheap->pfree_list_head == NULLPTR) {
        pheap->pfree_list_head        = (node_t*)node;
        pheap->pfree_list_head->sz    = alloc_size;
        pheap->pfree_list_head->pnext = NULLPTR;
        return;
    }
    /*
     * TODO: Should join near by free_list entries together
     */
    node_t* cur = pheap->pfree_list_head;
    while (cur != NULLPTR) {
        if (cur->pnext == NULLPTR) {
            cur->pnext        = (node_t*)ptr;
            cur->pnext->sz    = alloc_size;
            cur->pnext->pnext = NULLPTR;
            return;
        }
        cur = cur->pnext;
    }
    leave_critical ();
}


/**********************************************************************************/
/**************************** Setup MMU *******************************************/
/**********************************************************************************/


#define MMU_LEVEL2_ENTRIES   128
#define MMU_LEVEL2_PAGE_SIZE 0x1000 // 4KB
#define MMU_LEVEL3_ENTRIES   8192


// #define MMU_BLOCK_DESC_TYPE                   0x1
// #define MMU_TBL_DESC_TYPE                     0x3

/*
 * Granule size is 64KB so m is 16.
 * pg 2566 of arm reference manual
 */
#define MMU_GRANULE_SIZE_M   16
/*
 *  Level 2 Table Entry bits [47:m] == bits [47:m] for the base physical address
 * of the Level 3 Page Table. So the base physical address for Level 3 Page
 * Table needs to have the bottom [(m - 1):0] bits set to 0.
 */
#define LEVEL2_TBL_ADDR(addr) \
    (((uintptr_t)addr >> MMU_GRANULE_SIZE_M) & 0xFFFFFFFFUL)
/*
 * Core sends 64-bit virtual address:
 *   Bits [(m-1):0] of the given virtual address are the bottom [(m-1):0] bits for the translated physical address.
 *   Level 3 Page Entry holds the upper bits [47:m] for the translated physical address.
 *
 *   So the physical address given to the lvl 3 page entry needs to be shifted right m places so
 *   it will be in the correct position when added together to create the translated physical address.
 */
#define LEVEL3_PHYS_ADDR(addr) \
    (((uintptr_t)addr >> MMU_GRANULE_SIZE_M) & 0xFFFFFFFFUL)

/*
 * mair_el1 Register: https://developer.arm.com/documentation/ddi0601/2024-03/AArch64-Registers/MAIR-EL1--Memory-Attribute-Indirection-Register--EL1-?lang=en
 * Device memory attributes: https://developer.arm.com/documentation/den0024/a/Memory-Ordering/Memory-types/Device-memory
 */
/*
 * NON Gathering, NON Re-ordering, NON Early Write Acknowledgement
 */
#define MAIR_DEVICE_NGNRNE                    (0b00UL << 2)
/*
 * NON Gathering, NON Re-ordering, Early Write Acknowledgement
 */
#define MAIR_DEVICE_NGNRE                     (0b01UL << 2)
/*
 * NON Gathering, Re-ordering, Early Write Acknowledgement
 */
#define MAIR_DEVICE_NGRE                      (0b10UL << 2)
/*
 * Gathering, Re-ordering, Early Write Acknowledgement
 */
#define MAIR_DEVICE_GRE                       (0b11UL << 2)
/*
 * https://developer.arm.com/documentation/ddi0601/2024-03/AArch64-Registers/MAIR-EL1--Memory-Attribute-Indirection-Register--EL1-?lang=en
 */
#define MAIR_NORMAL_OUTERW_THROUGH_TRANSIENT  (0b0011UL << 4)
#define MAIR_NORMAL_OUTER_NO_CACHE            (0b0100UL << 4)
#define MAIR_NORMAL_OUTERW_BACK_TRANSIENT     (0b0111UL << 4)
#define MAIR_NORMAL_OUTERW_BACK_NON_TRANSIENT (0b1111UL << 4)

#define MAIR_NORMAL_INNERW_BACK_NON_TRANSIENT (0b1111UL << 0)
#define MAIR_NORMAL_INNER_NO_CACHE            (0b0100UL << 0)

#define MAIR_NORMAL_IDX                       0
#define MAIR_NORMAL_NO_CACHE_IDX              1
#define MAIR_DEVICE_IDX                       2
#define MAIR_DEVICE_COHERENT_IDX              3

#define MAIR_NORMAL \
    (MAIR_NORMAL_OUTERW_BACK_NON_TRANSIENT | MAIR_NORMAL_INNERW_BACK_NON_TRANSIENT)
#define MAIR_NORMAL_NO_CACHE \
    (MAIR_NORMAL_OUTER_NO_CACHE | MAIR_NORMAL_INNER_NO_CACHE)
#define MAIR_DEVICE_COHERENT MAIR_DEVICE_NGNRNE
#define MAIR_DEVICE          MAIR_DEVICE_NGNRE

/*
 * 64 bit TCR Register: https://developer.arm.com/documentation/ddi0601/2024-03/AArch64-Registers/TCR-EL1--Translation-Control-Register--EL1-
 * Intermediate Physical Address Size
 */
#define TCR_IPS_32           (0b000UL << 32) // 32 bits, 4GB.
#define TCR_IPS_36           (0b001UL << 32) // 36 bits, 64GB.
#define TCR_IPS_40           (0b010UL << 32) // 40 bits, 1TB.
#define TCR_IPS_42           (0b011UL << 32)
#define TCR_IPS_44           (0b100UL << 32)
#define TCR_IPS_48           (0b101UL << 32)
#define TCR_IPS_52           (0b110UL << 32)
#define TCR_IPS_MASK         (0b111UL << 32)

#define TCR_TGN_64KB(n)      (0b01UL << ((n * 30) + ((1 - n) * 14)))
#define TCR_TGN_16KB(n)      (0b10UL << ((n * 30) + ((1 - n) * 14)))
#define TCR_TGN_4KB(n)       (0b00UL << ((n * 30) + ((1 - n) * 14)))
#define TCR_TGN_MASK(n)      (0b11UL << ((n * 30) + ((1 - n) * 14)))
#define TCR_SHN_INNER(n)     (0b11UL << ((n * 28) + ((1 - n) * 12)))
#define TCR_SHN_NOSHARE(n)   (0b00UL << ((n * 28) + ((1 - n) * 12)))
#define TCR_SHN_OUTER(n)     (0b10UL << ((n * 28) + ((1 - n) * 12)))
#define TCR_SHN_MASK(n)      (0b11UL << ((n * 28) + ((1 - n) * 12)))
#define TCR_ORGNN_NORMAL_OUTER_NOCACHE(n) \
    (0b00UL << ((n * 26) + ((1 - n) * 10)))
#define TCR_ORGNN_NORMAL_OUTER_WB_CACHE(n) \
    (0b01UL << ((n * 26) + ((1 - n) * 10)))
#define TCR_ORGNN_NORMAL_OUTER_WT_CACHE(n) \
    (0b10UL << ((n * 26) + ((1 - n) * 10)))
#define TCR_ORGNN_NORMAL_OUTER_WB_NOWA(n) \
    (0b11UL << ((n * 26) + ((1 - n) * 10)))
#define TCR_ORGNN_MASK(n)                 (0b11UL << ((n * 26) + ((1 - n) * 10)))
#define TCR_IRGNN_NORMAL_INNER_NOCACHE(n) (0b00UL << ((n * 24) + ((1 - n) * 8)))
#define TCR_IRGNN_NORMAL_INNER_WB_CACHE(n) \
    (0b01UL << ((n * 24) + ((1 - n) * 8)))
#define TCR_IRGNN_NORMAL_INNER_WT_CACHE(n) \
    (0b10UL << ((n * 24) + ((1 - n) * 8)))
#define TCR_IRGNN_NORMAL_INNER_WB_NOWA(n) (0b11UL << ((n * 24) + ((1 - n) * 8)))
#define TCR_IRGNN_MASK(n)                 (0b11UL << ((n * 24) + ((1 - n) * 8)))
/*
 * This bit controls whether a translation table walk is performed on a TLB
 * miss, for an address that is translated using TTBRN_EL1
 */
// #define TCR_EPDN_ALLOW_TBL_WALK(n)        (0b0UL << ((n * 23) + ((1 - n) * 7)))
#define TCR_EPDN_NO_TBL_WALK(n)           (0b1UL << ((n * 23) + ((1 - n) * 7)))
#define TCR_EPDN_MASK(n)                  (0b1UL << ((n * 23) + ((1 - n) * 7)))
/*
 * Selects whether TTBR0_EL1 or TTBR1_EL1 defines the ASID.
 * 0b0 = TTBR0_EL1.ASID defines the ASID.
 * 0b1 = TTBR1_EL1.ASID defines the ASID
 */
#define TCR_A1_TTBR1                      (0b1UL << 22)
#define TCR_A1_MASK                       (0b1UL << 22)
/*
 * The size offset of the memory region addressed by TTBRN_EL1. The region size
 * is 2^(64 - TNSZ) bytes. T1SZ = [21:16] and T0SZ = [5:0]
 */
// x = 64 - log2(4GB)
#define TCR_TNSZ_4GB(n)                   (32UL << ((n * 16) + ((1 - n) * 0)))
// x = 64 - log2(64GB)
#define TCR_TNSZ_64GB(n)                  (28UL << ((n * 16) + ((1 - n) * 0)))
#define TCR_TNSZ_MASK(n)                  (0x3FUL << ((n * 16) + ((1 - n) * 0)))

#define TCR_MASK(n)                                                           \
    (                                                                         \
    TCR_IPS_MASK | TCR_TGN_MASK (n) | TCR_SHN_MASK (n) | TCR_ORGNN_MASK (n) | \
    TCR_IRGNN_MASK (n) | TCR_EPDN_MASK (n) | TCR_AN_MASK (n) | TCR_TNSZ_MASK (n))


// clang-format off
/*
 * pg 2566 of arm reference manual
 */
typedef struct tbl_desc_lvl2 {
    u64 entry_type      : 2, 
    ignored             : (1 + (MMU_GRANULE_SIZE_M - 1) - 2),
    /*
    * Bits[47:m] are bits[47:m] of the address of the required next-level table, which is:
    *   For a level 1 Table descriptor, the address of a level 2 table.
    *   For a level 2 Table descriptor, the address of a level 3 table
    * Bits[15:0] of the table address are zero.
    */ 
    tbl_addr            : (1 + 47 - MMU_GRANULE_SIZE_M),
    ignored2            : (1 + 58 - 48),
    /*
    * This bit is valid only for a stage1 translation that can support two VA ranges and treated RES0 for 
    * stage1 translations that support only one VA range.
    *
    * When PXNTable bit is set to 1, the PXN bit is treated as set to 1 for all subsequent levels of lookup, 
    * irrespective of the actual value set in the descriptors, and the instructions fetched from the 
    * corresponding region cannot be executed in EL1 and higher exception levels.
    */
    pxn_tbl             : (1 + 59 - 59),
    /*
    * If multiple VA ranges are supported in Stage1, this field is called UXNTable and, 
    * determines if instructions that is fetched from the region referred to by subsequent 
    * levels of look up can be executed at EL0.
    * 
    * If only one VA is supported in Stage1, this field is XNTable, and exercises control over 
    * eXecute Never behavior, for the same stage of translation:
    *   If the bit is UXNTable and set to 1, then UXN bit is treated as set in all subsequent levels lookup, 
    *       irrespective of the actual value set in the descriptors.
    *   If the bit is XNTable and set to 1, then XN bit is treated as set in all subsequent levels 
    *       lookup, irrespective of the actual value set in the descriptors.
    *   If set to 0, this bit has no effect.
    */
    xn_tbl              : (1 + 60 - 60), // XN limit for subsequent levels of lookup
    ap_tbl              : (1 + 62 - 61), // Access permissions limit for subsequent levels of lookup -- pg 2583 arm reference manual
    ns_tbl              : (1 + 63 - 63); // For memory accesses from Secure state, specifies the Security state for subsequent levels of lookup
} GCC_PACKED tbl_desc_lvl2;

/*
 * pg 2569 of arm reference manual
 */
typedef struct page_desc_lvl3 {
    u64 entry_type  : (1 + 1 - 0),
    // Start Lower Attributes -- Bits 11 to 2 -- 10 Bits in Size
    attr_idx	    : (1 + 4 - 2),	    // 3 bits
    /*
    * 0b1: Access granted for Secure and Non-Secure execution state.
    * 0b0: Access granted only for Secure execution state.
    */
    ns		        : (1 + 5 - 5),	    // 1 bit -- Non-secure bit -- set to 0
#define AP_EL1_READ_WRITE   0b00UL // el1 read and write
#define AP_ALL_READ_WRITE   0b01UL // el1 and el0 read and write
#define AP_El1_READ_ONLY    0b10UL // el1 read only
#define AP_ALL_READ_ONLY    0b11UL // el1 and el0 read only
    ap		        : (1 + 7 - 6),	    // 2 bits --  Data Access Permissions bits -- pg 2581 arm reference manual
/*
* Specifies the Shareability attributes of the corresponding memory region.
* NOTE: The shareability field is only relevant if the memory is a Normal Cacheable memory type. All Device and Normal
* Non-cacheable memory regions are always treated as Outer Shareable, regardless of the translation table
* shareability attributes
*/
#define SH_NORMAL_MEM_NO_SHARE      0b00UL // Non-shareable
#define SH_NORMAL_MEM_OUTER_SHARE   0b10UL // Outer Shareable
#define SH_NORMAL_MEM_INNER_SHARE   0b11UL // Inner Shareable
    sh		        : (1 + 9 - 8),	    // 2 bits --  Shareability field -- pg 2600 arm reference manual
    af		        : (1 + 10 - 10),    // 1 bit -- The Access flag -- set to 1 pg 2590 arm reference manual
    ng		        : (1 + 11 - 11),    // 1 bit -- The not global bit -- set to 0
    // End Lower Attributes
    ta	            : (1 + 15 - 12),    // 4 bits -- TA[51:48] indicates bits[51:48] of the page address.
    phys_addr       : (1 + 47 - MMU_GRANULE_SIZE_M),
    reserved1	    : (1 + 50 - 48),	// set to 0
    dbm             : (1 + 51 - 51),    // Dirty Bit Modifier
    contiguous	    : (1 + 52 - 52),	// A hint bit indicating that the translation table entry is one of a contiguous set or entries -- set to 0
/*
* The Privileged execute-never field. When the PXN bit is 1, a Permission fault is 
* generated if the processor is executing at 
* EL1 and attempts to execute an instruction fetched from the corresponding memory region. 
* As with the XN bit, when using the Short-descriptor translation table format, the fault is 
* generated only if the access is to memory in the Client domain.
* 
* If 0, execution of code is allowed.
*/
#define PXN_EL1_EXECUTION_ALLOWED   0x0UL // Should be allowed for kernel memory
#define PXN_EL1_NO_EXECUTION        0x1UL // Should be disallowed for device memory
    pxn		        : (1 + 53 - 53),
/*
* The Execute-never or Unprivileged execute-never field. 
* If 0, execution of code is permitted for EL0.
*/
#define UXN_EL0_EXECUTION_ALLOWED   0x0UL
#define UXN_EL0_NO_EXECUTION        0x1UL
    uxn		        : (1 + 54 - 54),
    ignored0		: 9;	            // set to 0

} GCC_PACKED page_desc_lvl3;

STATIC_ASSERT((sizeof(tbl_desc_lvl2) == 8));
STATIC_ASSERT((sizeof(page_desc_lvl3) == 8));

GCC_NODISCARD static void* vtable_page_allocate (void) {
    heap_t* pheap = &heaps_[eHEAP_ID_MID];
    if(pheap->end - (uintptr_t)pheap->pcur_pos < VTABLE_PAGE_SIZE) {
        return NULLPTR;
    }
    void* alloc   = pheap->pcur_pos;
    pheap->pcur_pos += VTABLE_PAGE_SIZE;
    return alloc;
}

// clang-format on

static void* create_translation_tbl_lvl3_ (uintptr_t addr) {
    page_desc_lvl3* tbl = vtable_page_allocate ();
    memset (tbl, 0, VTABLE_PAGE_SIZE);
    // LOG_INFO ("MMU Level 3 Page Table with Addr [0x%X]", (uintptr_t)tbl);
    /*
     * With Granule Size 64KB addresses to level 3 tables have
     * 1 + 47 - 16 = 32 bits to represent a physical address to their table.
     * 1111 1111 1111 1111 1111 1111 1111 1111 0000 0000 0000 0000
     */
    ASSERT ((tbl != NULLPTR), "MMU Failed to allocate level 3 Page Table");
    ASSERT (ADDR_IS_ALIGNED (tbl, 16), "MMU Level 3 Page table address is NOT aligned: [0x%X]", (uintptr_t)tbl);
    ASSERT ((sizeof (page_desc_lvl3) * MMU_LEVEL3_ENTRIES == VTABLE_PAGE_SIZE), "MMU Level 3 Table is incorrect size");

    for (uintptr_t entry_idx = 0; entry_idx < MMU_LEVEL3_ENTRIES; ++entry_idx) {
        page_desc_lvl3* desc = &(tbl[entry_idx]);
        desc->entry_type     = 3;
        desc->attr_idx       = MAIR_NORMAL_IDX;
        desc->ns             = 0;
        /*
         * Using ALL_READ_WRITE causes the Pi to hang when the MMU is enabled
         */
        desc->ap = AP_EL1_READ_WRITE; // AP_ALL_READ_WRITE;
        desc->sh = SH_NORMAL_MEM_INNER_SHARE;
        desc->af = 1;
        desc->ng = 0;
        desc->ta = 0;
        ASSERT (ADDR_IS_ALIGNED (addr, 16), "");
        desc->phys_addr  = LEVEL3_PHYS_ADDR (addr);
        desc->dbm        = 0;
        desc->contiguous = 0;
        desc->pxn        = PXN_EL1_EXECUTION_ALLOWED;
        desc->uxn        = UXN_EL0_NO_EXECUTION;
        /*
         * The only memory section that will be executable ranges from 0x0
         * to kernel_start + kernel_text_size
         */
        if (addr > KERNEL_END_ADDR) {
            desc->pxn = PXN_EL1_NO_EXECUTION;
            if (addr >= ARM_DRAM_COHERENT_START && addr <= ARM_DRAM_COHERENT_END) {
                desc->attr_idx = MAIR_DEVICE_COHERENT_IDX;
                desc->sh       = SH_NORMAL_MEM_OUTER_SHARE;
            } else if (addr >= VC_DRAM_MEM_START && addr <= VC_DRAM_MEM_END) {
                desc->attr_idx = MAIR_NORMAL_NO_CACHE_IDX;
                desc->sh       = SH_NORMAL_MEM_OUTER_SHARE;
            } else if (addr >= PERIPH_BASE) { // && addr <= PERIPH_END) {
                desc->attr_idx = MAIR_DEVICE_COHERENT_IDX;
                desc->sh       = SH_NORMAL_MEM_OUTER_SHARE;
            }
        }
        addr += VTABLE_PAGE_SIZE;
    }
    return (void*)tbl;
}

static void* translation_tbl_init_ (void) {
    uintptr_t addr           = 0x0;
    tbl_desc_lvl2* table_ptr = vtable_page_allocate ();
    memset (table_ptr, 0, VTABLE_PAGE_SIZE);
    ASSERT (table_ptr != NULLPTR, "");
    ASSERT (ADDR_IS_ALIGNED (table_ptr, 16), "");
    // LOG_DEBUG ("MMU Level 2 Table Address [0x%X]", (uintptr_t)table_ptr);
    for (uintptr_t entry_idx = 0; entry_idx < MMU_LEVEL2_ENTRIES; ++entry_idx) {
        addr = entry_idx * MMU_LEVEL3_ENTRIES * VTABLE_PAGE_SIZE;
        /*
         * Leave entries greater than 4GB as invalid entries
         */
        // u64 pcie_start       = 0x600000000UL;
        // u64 pcie_end         = 0x7FFFFFFFFUL;
        bool isin_pcie_range = addr >= MEM_PCIE_RANGE_START_VIRTUAL &&
                               addr <= MEM_PCIE_RANGE_END_VIRTUAL;
        if (isin_pcie_range) {
            LOG_INFO ("PCIe Range: 0x%X 0x%X", (u32)(addr >> 32), (u32)addr);
        }
        if (addr > (4UL * 0x40000000UL) && isin_pcie_range == false) {
            continue;
        }

        void* subtbl = create_translation_tbl_lvl3_ (addr);
        ASSERT (subtbl != NULLPTR, "");

        tbl_desc_lvl2* desc = &(table_ptr[entry_idx]);
        desc->entry_type    = 3; // table descriptor
        desc->tbl_addr      = LEVEL2_TBL_ADDR (subtbl);
        desc->pxn_tbl       = 0;
        desc->xn_tbl        = 0;
        desc->ap_tbl        = 0;
        desc->ns_tbl        = 0;
    }
    // LOG_INFO ("MMU Finished Creating Page Tables");
    DATA_SYNC_BARRIER_FS_ANY ();
    return (void*)table_ptr;
}

static void mmu_init (void) {
    void* tran_tbl = translation_tbl_init_ ();

    u64 mair = MAIR_DEVICE << (MAIR_DEVICE_IDX * 8);
    mair |= MAIR_DEVICE_COHERENT << (MAIR_DEVICE_COHERENT_IDX * 8);
    mair |= MAIR_NORMAL << (MAIR_NORMAL_IDX * 8);
    mair |= MAIR_NORMAL_NO_CACHE << (MAIR_NORMAL_NO_CACHE_IDX * 8);
    asm volatile("msr mair_el1, %0" ::"r"(mair));

    // LOG_INFO ("MMU Setup MAIR Register [0x%X%X]", (mair >> 32), mair);

    asm volatile("msr ttbr0_el1, %0" ::"r"((uintptr_t)tran_tbl));

    // LOG_INFO ("MMU Setup ttbr0_el1 Register");

    u64 tcr;
    asm volatile("mrs %0, tcr_el1" : "=r"(tcr));
    tcr &= ~TCR_IPS_MASK;
    tcr &= ~TCR_TGN_MASK (0);
    tcr &= ~TCR_SHN_MASK (0);
    tcr &= ~TCR_ORGNN_MASK (0);
    tcr &= ~TCR_IRGNN_MASK (0);
    tcr &= ~TCR_EPDN_MASK (0);
    tcr &= ~TCR_A1_MASK;
    tcr &= ~TCR_TNSZ_MASK (0);

    tcr |= TCR_EPDN_NO_TBL_WALK (1); // disable tbl walks for ttbr1_el1 addresses (u64 starting with 0xFFFFF....)
    tcr |= TCR_TGN_64KB (0);
    tcr |= TCR_SHN_INNER (0);
    tcr |= TCR_ORGNN_NORMAL_OUTER_WB_CACHE (0);
    tcr |= TCR_IRGNN_NORMAL_INNER_WB_CACHE (0);
    tcr |= TCR_IPS_36;
    tcr |= TCR_TNSZ_64GB (0);
    asm volatile("msr tcr_el1, %0" ::"r"(tcr));

    // LOG_INFO ("MMU Setup tcr Register [0x%X%X]", (tcr >> 32), tcr);

    // Force changes to be seen before MMU is enabled
    ISB ();

    u64 sctlr;
    asm volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
    sctlr &= ~(SCTLR_WXN | SCTLR_ALIGNMENT);
    sctlr |= SCTLR_I_CACHE_ENABLED;
    sctlr |= SCTLR_D_CACHE_ENABLED;
    sctlr |= SCTLR_MMU_ENABLED;

    // LOG_INFO ("MMU About to Enable");
    asm volatile("msr sctlr_el1, %0" ::"r"(sctlr));
    // u64 par;
    // asm volatile("AT S12E1R, %0" ::"r"(0x80000));
    // asm volatile("mrs %0, par_el1" : "=r"(par));
    // LOG_INFO ("[%u]", par);
    ISB ();
    LOG_INFO ("MMU Setup Complete");
}

bool mem_init (heap_t (*config)[MEM_NUM_HEAP_SECTIONS]) {
    // if (config == NULLPTR) {
    // #ifndef ON_HOST_TESTING
    heap_t coherent = { .start    = ARM_DRAM_COHERENT_START,
                        .end      = ARM_DRAM_COHERENT_END,
                        .pcur_pos = (u8*)ARM_DRAM_COHERENT_START,
                        .size = ARM_DRAM_COHERENT_END - ARM_DRAM_COHERENT_START,
                        .pfree_list_head = NULLPTR };
    heap_t low      = { .start    = ARM_DRAM_LOW_MEM_START,
                        .end      = ARM_DRAM_LOW_MEM_END,
                        .pcur_pos = (u8*)ARM_DRAM_LOW_MEM_START,
                        .size     = ARM_DRAM_LOW_MEM_END - ARM_DRAM_LOW_MEM_START,
                        .pfree_list_head = NULLPTR };
    heap_t mid      = { .start    = ARM_DRAM_MID_MEM_START,
                        .end      = ARM_DRAM_MID_MEM_END,
                        .pcur_pos = (u8*)ARM_DRAM_MID_MEM_START,
                        .size     = ARM_DRAM_MID_MEM_END - ARM_DRAM_MID_MEM_START,
                        .pfree_list_head = NULLPTR };
    // heap_t high           = { .start    = (uintptr_t)ARM_DRAM_HIGH_MEM_START,
    //                           .end      = (uintptr_t)ARM_DRAM_HIGH_MEM_END,
    //                           .pcur_pos = (u8*)ARM_DRAM_HIGH_MEM_START,
    //                           .size = ARM_DRAM_HIGH_MEM_END - ARM_DRAM_HIGH_MEM_START,
    //                           .pfree_list_head = NULLPTR };
    heaps_[eHEAP_ID_COHERENT] = coherent;
    heaps_[eHEAP_ID_LOW]      = low;
    heaps_[eHEAP_ID_MID]      = mid;
    // heaps_[eHEAP_ID_HIGH] = high;
    LOG_INFO (
    "Coherent Heap Start [0x%X] End [0x%X]", heaps_[eHEAP_ID_COHERENT].start,
    heaps_[eHEAP_ID_COHERENT].end);
    LOG_INFO (
    "Low Heap Start [0x%X] End [0x%X]", heaps_[eHEAP_ID_LOW].start,
    heaps_[eHEAP_ID_LOW].end);
    LOG_INFO (
    "Mid Heap Start [0x%X] End [0x%X]", heaps_[eHEAP_ID_MID].start,
    heaps_[eHEAP_ID_MID].end);
    // LOG_INFO (
    // "High Heap Start [0x%X] End [0x%X]", heaps_[eHEAP_ID_HIGH].start,
    // heaps_[eHEAP_ID_HIGH].end);
    //#endif
    //}
    ASSERT (heaps_are_valid_ (&heaps_), "");

    // Invalidate instruction cache
    asm volatile("ic IALLUIS");
    // Invalidate TLB entries
    asm volatile("tlbi vmalle1");
    DATA_SYNC_BARRIER_FS_ANY ();
    ISB ();

    mmu_init ();
    return true;
}