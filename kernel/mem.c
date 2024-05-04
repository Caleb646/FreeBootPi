#include "mem.h"
#include "printf.h"


static heap_s heap = {
    .start = (u8*)ARM_DRAM_HIGH_MEM_START,
    .end = (u8*)ARM_DRAM_HIGH_MEM_END,
    .cur_pos = (u8*)ARM_DRAM_HIGH_MEM_START,
    .size = ARM_DRAM_HIGH_MEM_END - ARM_DRAM_MID_MEM_START,
    .free_list_head = NULLPTR
};

void* malloc(size_t sz)
{
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
    if(alloc == NULLPTR && ( (uintptr_t)heap.end - (uintptr_t)heap.cur_pos ) < (sz + sizeof(size_t)))
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
        heap.cur_pos = (u8*) ( ((uintptr_t)heap.cur_pos + (uintptr_t)(MALLOC_ADDR_ALIGNMENT - 1) + sizeof(size_t)) & (uintptr_t)MALLOC_ADDR_ALIGNMENT_MASK );
        alloc = (node_s*)( (uintptr_t)heap.cur_pos - sizeof(size_t) );
        alloc->sz = sz;
        heap.cur_pos += sz;
    }

    return (void*)( &(alloc->next) );
}

void free(void* ptr)
{
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
}

s32 mem_init(heap_s* hp)
{
    if(hp == NULLPTR) {}
    else 
    {
        heap = *hp;
    }
    return 1;
}