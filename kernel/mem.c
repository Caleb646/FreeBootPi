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