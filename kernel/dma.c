#include "dma.h"
#include "sync.h"
#include "printf.h"
#include "mem.h"

/*
* GCC aligns the beginning of the array on a 32 byte boundary and because
* a control_block_s is 32 bytes each elements' addr is also aligned on a 32 byte
* boundary
*/
static anonymous_control_block_s GCC_ALIGN_ADDR(DMA_CONTROL_BLOCK_BYTE_ALIGNMENT)       allocated_channel_cbs[MAX_DMA_CHANNEL_ID + 1];
static u32                                                                              is_channel_allocated[MAX_DMA_CHANNEL_ID + 1];
static dma_config_s                                                                     dma_config;

static s32 find_open_channel(dma_type_t dma_type)
{   
    u32 start, end;
    if(dma_type == DMA_STANDARD)
    {
        start = dma_config.dma_start_id;
        end = (dma_config.dma_lite_start_id - 1);
    }
    else if(dma_type == DMA_LITE)
    {
        start = dma_config.dma_lite_start_id;
        end = (dma_config.dma4_start_id - 1);
    }
    else if(dma_type == DMA4)
    {
        start = dma_config.dma4_start_id;
        end = (dma_config.dma4_end_id);
    }
    else
    {
        LOG_ERROR("Invalid dma type passed to find_open_channel");
        return -1;
    }

    for(u32 i = start; i <= end; ++i)
    {
        if(is_channel_allocated[i] == FALSE)
        {
            return i;
        }
    }

    for(u32 i = start; i <= end; ++i)
    {
        if(DMA_CHANNEL_IS_ACTIVE(i) == FALSE)
        {
            return i;
        }
    }
    return -1;
}

/*
* \brief
* \param control_block The src and dest addresses MUST be VPU Bus Addresses not SRAM Physical addresses. Because of
*   this both the src and dest addresses must also point to a location in the 1st GB of SRAM. Unless DMA4 is used.
*   DMA4 can use 40 bit addresses.
*/
dma_status_t dma_transfer(void* control_block, dma_type_t dma_type)
{
    s32 channel_id = find_open_channel(dma_type);
    if(channel_id == -1)
    {
        return DMA_NO_OPEN_CHANNELS;
    }
    else if(DMA_CHANNEL_HAS_ERROR(channel_id))
    {
        LOG_ERROR("Previous DMA transfer failed");
    }
    is_channel_allocated[channel_id] = 1;
    memcopy(control_block, sizeof(anonymous_control_block_s), &(allocated_channel_cbs[channel_id]) );
    u32 control_block_addr = (u32)(&(allocated_channel_cbs[channel_id]));
    if(dma_type == DMA4)
    {
        control_block_addr >>= DMA4_CB_ADDR_SHIFT;
    }
    else
    {
        // NOTE: only have 32 - 5 bits to address each control block. allocated_channel_cbs
        // is currently stored somewhere around 0x84260 in the .bss section. If it is moved and the address
        // takes more than 27 bits to represent the u32 cast will truncate it and weird things
        // will happen.
        control_block_addr = ARM_TO_VPU_BUS_ADDR(control_block_addr);
    }
    put32(DMA_CONTROL_BLOCK_ADDR(channel_id), control_block_addr);
    // Ensure control block address is written and visible to DMA before DMA is activated
    DATA_MEMORY_BARRIER_OUTER_STORES();
    // Clear error, interrupt flags and enable channel
    u32 flags = DMA_CONTROL_STATUS_ERROR_BIT | DMA_CONTROL_STATUS_DMA_ACTIVE_BIT | DMA_CONTROL_STATUS_INT_STATUS_BIT;
    put32(DMA_CONTROL_STATUS(channel_id), get32(DMA_CONTROL_STATUS(channel_id)) | flags);

    return DMA_OK;
}

s32 dma_init(void)
{
    memset((void *)allocated_channel_cbs, 0, sizeof(allocated_channel_cbs));
    memset((void *)is_channel_allocated, 0, sizeof(is_channel_allocated));

    dma_config.dma_start_id = 0;
    dma_config.dma_lite_start_id = 7;
    dma_config.dma4_start_id = 11;
    dma_config.dma4_end_id = MAX_DMA_CHANNEL_ID;

    return 1;
}