#include "dma.h"
#include "sync.h"


// If 0 channel is not in use
static u32 allocated_channels[MAX_DMA_CHANNEL_ID + 1];

dma_status_t dma_transfer(void* control_block, u32 channel_id, dma_type_t dma_type)
{
    dma_status_t status = DMA_OK;
    if(channel_id <= MAX_DMA_CHANNEL_ID)
    {
        // 
        if(DMA_CHANNEL_IS_ACTIVE(channel_id))
        {
            status = DMA_CHANNEL_OCCUPIED;
        }  
        else if(DMA_CHANNEL_HAS_ERROR(channel_id))
        {
            REG_PTR32(DMA_CONTROL_STATUS(channel_id)) &= ~DMA_CONTROL_STATUS_DMA_ACTIVE_BIT;
        }
    }

    if(status == DMA_OK)
    {
        allocated_channels[channel_id] = 1;
        REG_PTR32(DMA_CONTROL_BLOCK_ADDR(channel_id)) = (u32)control_block;
        // Ensure control block address is written and visible to DMA before DMA is activated
        DATA_MEMORY_BARRIER_OUTER_STORES();
        // Clear error, interrupt flags and enable channel
        u32 flags = DMA_CONTROL_STATUS_ERROR_BIT | DMA_CONTROL_STATUS_DMA_ACTIVE_BIT | DMA_CONTROL_STATUS_INT_STATUS_BIT;
        REG_PTR32(DMA_CONTROL_STATUS(channel_id)) |= flags;
    }
    return status;
}

s32 dma_init(void)
{
    memset((void *)allocated_channels, 0, sizeof(allocated_channels));

    return 1;
}