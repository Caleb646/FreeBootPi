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
static u32 volatile                                                                     is_channel_allocated[MAX_DMA_CHANNEL_ID + 1];
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
dma_status_t dma_transfer(anonymous_control_block_s* control_block, dma_type_t dma_type)
{
    // enter_critical(IRQ_DISABLED_FIQ_DISABLED_TARGET);

    s32 channel_id = find_open_channel(dma_type);
    if(channel_id == -1)
    {
        return DMA_NO_OPEN_CHANNELS;
    }
    else if(DMA_CHANNEL_HAS_ERROR(channel_id))
    {
        LOG_ERROR("Previous DMA transfer failed");
    }
    // enter_critical(IRQ_DISABLED_FIQ_DISABLED_TARGET);
    is_channel_allocated[channel_id] = 1;
    // leave_critical();
    memcopy(control_block, sizeof(anonymous_control_block_s), &(allocated_channel_cbs[channel_id]) );
    u32 control_block_addr = (u32)(&(allocated_channel_cbs[channel_id]));

    // leave_critical();

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

static dma_status_t dma_memcpy_(u64 src_addr, u64 dest_addr, size_t transfer_length, dma_type_t dma_type)
{
    u64 src_bus = ARM_TO_VPU_BUS_ADDR(src_addr);
    u64 dest_bus = ARM_TO_VPU_BUS_ADDR(dest_addr);
    anonymous_control_block_s block;
    memset(&block, 0, sizeof(anonymous_control_block_s));
    if(dma_type == DMA_STANDARD)
    {
        control_block_s* dma_block = (control_block_s*)&block;
        dma_block->source_addr = src_bus;
        dma_block->dest_addr = dest_bus;
        dma_block->transfer_length = transfer_length;
    }
    else if(dma_type == DMA_LITE)
    {
        dma_lite_control_block_s* dma_block = (dma_lite_control_block_s*)&block;
        dma_block->source_addr = src_bus;
        dma_block->dest_addr = dest_bus;
        dma_block->transfer_length = transfer_length;
    }
    else if(dma_type == DMA4)
    {
        dma4_control_block_s* dma_block = (dma4_control_block_s*)&block;
        dma_block->source_addr = src_bus;
        dma_block->dest_addr = dest_bus;
        dma_block->transfer_length = transfer_length;
    }
    else
    {
        LOG_ERROR("Incorrect dma_type: [%u]", dma_type);
        return DMA_ERROR_ON_TRANSFER;
    }
    return dma_transfer(&block, dma_type);
}

dma_status_t dma_memcpy(u64 src_addr, u64 dest_addr, size_t transfer_length, dma_type_t dma_type)
{
    while(transfer_length > DMA_MAX_TRANSFER_LENGTH)
    {
        dma_status_t status = dma_memcpy_(src_addr, dest_addr, DMA_MAX_TRANSFER_LENGTH, dma_type);
        if(status == DMA_OK)
        {
            src_addr += DMA_MAX_TRANSFER_LENGTH;
            dest_addr += DMA_MAX_TRANSFER_LENGTH;
            transfer_length -= DMA_MAX_TRANSFER_LENGTH;
        }
        else if(status != DMA_NO_OPEN_CHANNELS)
        {
            return status;
        }
    }
    dma_status_t status = dma_memcpy_(src_addr, dest_addr, DMA_MAX_TRANSFER_LENGTH, dma_type);
    while(status != DMA_OK)
    {
        if(status != DMA_NO_OPEN_CHANNELS)
        {
            return status;
        }
    }
    return DMA_OK;
}

s32 dma_init(dma_config_s* config)
{
    memset((void *)allocated_channel_cbs, 0, sizeof(allocated_channel_cbs));
    memset((void *)is_channel_allocated, 0, sizeof(is_channel_allocated));

    if(config == NULLPTR)
    {
        dma_config.dma_start_id = DMA_CHANNEL_START;
        dma_config.dma_lite_start_id = DMA_LITE_CHANNEL_START;
        dma_config.dma4_start_id = DMA_4_CHANNEL_START;
        dma_config.dma4_end_id = DMA_4_CHANNEL_END;
    }
    else
    {
        dma_config = *config;
    }
    return 1;
}