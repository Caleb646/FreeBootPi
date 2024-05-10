#include "mem.h"
#ifndef ON_HOST_TESTING
    #include "printf.h"
    #include "sync.h"
#endif

static heap_s heap;

/*
* \brief
* \param sz size of allocation to make in bytes
* \param alignment if alignment == 8, align allocation address on 8 byte boundary.
*   Must be a power of 2.
*/
GCC_NODISCARD void* align_allocate(size_t sz, size_t alignment)
{
    // Interrupt but NOT thread safe
    s32 status = enter_critical(IRQ_DISABLED_FIQ_DISABLED_TARGET);
    node_s* prev = NULLPTR;
    node_s** cur = &heap.free_list_head;
    node_s* alloc = NULLPTR;
    while((*cur) != NULLPTR)
    {
        /* 
        * TODO: A very large previous allocation in the free list could be used to allocate a small object.
        * Should search for smallest allocation in free list that meets the requirements.
        */
        if((*cur)->sz >= sz)
        {
            if((*cur)->next != NULLPTR && prev != NULLPTR)
            {
                prev->next = (*cur)->next;
            }
            alloc = (node_s*)(*cur);
            alloc->sz = (*cur)->sz;
            *cur = NULLPTR;
            break;
        }
        prev = (*cur);
        (*cur) = (*cur)->next;
    }
    if(
        alloc == NULLPTR && 
        ( (uintptr_t)heap.end - (uintptr_t)heap.cur_pos ) < (sz + sizeof(size_t))
    )
    {
        /* Aligning address using modulo */
        // u32 alignment = ((uintptr_t)(heap_current + sizeof(size_t))) % MALLOC_ADDR_ALIGNMENT;
        // heap_current += MALLOC_ADDR_ALIGNMENT - alignment;

        /*
        * Align returned address (&alloc->next) by MALLOC_ADDR_ALIGNMENT (8, 16, 64...).
        * Depending on the alignment the last several least significant bits will not be used.
        * For example, with alignment 16, the 4 least significant bits will not be used. So a mask of ~0x0F anded
        * with the current memory address + (alignment - 1) + sizeof(size_t) returns the aligned address for
        * alloc->next.
        */
        heap.cur_pos = (u8*) ( ((uintptr_t)heap.cur_pos + (uintptr_t)(alignment - 1) + sizeof(size_t)) & (uintptr_t)(~(alignment-1)) );
        alloc = (node_s*)( (uintptr_t)heap.cur_pos - sizeof(size_t) );
        alloc->sz = sz;
        heap.cur_pos += sz;
    }

    void* ret = (void*)( &(alloc->next) );
    status = leave_critical();
    return ret;
}

GCC_NODISCARD void* align_allocate_set(size_t sz, u8 value, size_t alignment)
{
    void* ptr = align_allocate(sz, alignment);
    memset(ptr, value, sz);
    return ptr;
}

GCC_NODISCARD void* allocate(size_t sz)
{
    return align_allocate(sz, MALLOC_ADDR_ALIGNMENT);
}

void delete(void* ptr)
{
    s32 status = enter_critical(IRQ_DISABLED_FIQ_DISABLED_TARGET);
    node_s* node = (node_s*)( ((u8*)ptr) - sizeof(size_t) );
    size_t alloc_size = node->sz;
    if(heap.free_list_head == NULLPTR)
    {     
        heap.free_list_head = (node_s*)node;
        heap.free_list_head->sz = alloc_size;
        heap.free_list_head->next = NULLPTR;
        return;
    }
    /*
    * TODO: Should join free_list entries together 
    */
    node_s* cur = heap.free_list_head;
    while(cur != NULLPTR)
    {
        if(cur->next == NULLPTR)
        {
            cur->next = (node_s*)ptr;
            cur->next->sz = alloc_size;
            cur->next->next = NULLPTR;
            return;
        }
        cur = cur->next;
    }
    status = leave_critical();
}

s32 mem_init(heap_s* hp)
{
    if(hp == NULLPTR) 
    {
    #ifndef ON_HOST_TESTING
        heap.start = (u8*)ARM_DRAM_HIGH_MEM_START;
        heap.end = (u8*)ARM_DRAM_HIGH_MEM_END;
        heap.cur_pos = (u8*)ARM_DRAM_HIGH_MEM_START;
        heap.size = ARM_DRAM_HIGH_MEM_END - ARM_DRAM_HIGH_MEM_START;
        heap.free_list_head = NULLPTR;
    #endif
    }
    else 
    {
        heap = *hp;
    }
    return 1;
}