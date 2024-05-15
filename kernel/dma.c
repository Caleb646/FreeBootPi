#include "dma.h"
#include "irq.h"
#include "mem.h"
#include "printf.h"
#include "sync.h"

/*
 * GCC aligns the beginning of the array on a 32 byte boundary and because
 * a control_block_s is 32 bytes each elements' addr is also aligned on a 32
 * byte boundary
 */
static anonymous_control_block_s GCC_ALIGN_ADDR (DMA_CONTROL_BLOCK_BYTE_ALIGNMENT)
allocated_channel_cbs[(MAX_DMA_CHANNEL_ID + 1) * DMA_MAX_CBS_PER_CHANNEL];
static u32 volatile is_channel_allocated[MAX_DMA_CHANNEL_ID + 1];
static dma_config_s dma_config;

static s32 find_open_channel (dma_type_t dma_type) {
    u32 start, end;
    if (dma_type == DMA_STANDARD) {
        start = dma_config.dma_start_id;
        end   = (dma_config.dma_lite_start_id - 1);
    } else if (dma_type == DMA_LITE) {
        start = dma_config.dma_lite_start_id;
        end   = (dma_config.dma4_start_id - 1);
    } else if (dma_type == DMA4) {
        start = dma_config.dma4_start_id;
        end   = (dma_config.dma4_end_id);
    } else {
        LOG_ERROR ("Invalid dma type passed to find_open_channel");
        return -1;
    }

    for (u32 i = start; i <= end; ++i) {
        if (is_channel_allocated[i] == FALSE) {
            return i;
        }
    }

    for (u32 i = start; i <= end; ++i) {
        if (DMA_CHANNEL_IS_ACTIVE (i) == FALSE) {
            return i;
        }
    }
    return -1;
}

static dma_status_t dma_allocate_channel (dma_type_t dma_type, u32* channel_out) {
    s32 channel_id = find_open_channel (dma_type);
    if (channel_id == -1) {
        return DMA_NO_OPEN_CHANNELS;
    }
    *channel_out                     = channel_id;
    is_channel_allocated[channel_id] = 1;
    return DMA_OK;
}

static dma_status_t dma_delete_channel (u32 channel_id) {
    /*
     * Clear the next control block address register to 0x0. Only the current cb
     * is aborted when the abort bit is set. So the next cb address needs to be
     * cleared as well.
     */
    write32 (DMA_NEXT_CB_ADDR (channel_id), 0x0);
    /*
     * The write needs to be visible to the DMA before the abort bit is set.
     */
    DATA_MEMORY_BARRIER_OUTER_STORES ();

    /*
     * Clear the error and interrupt bits and set the abort bit.
     */
    write32 (DMA_CS (channel_id), read32 (DMA_CS (channel_id)) | DMA_CS_ABORT_VAL);
    is_channel_allocated[channel_id] = 0;
    /*
     * Ensure DMA channel is stopped before the current cb is overwritten
     */
    DATA_MEMORY_BARRIER_OUTER_STORES ();
    for (u32 i = 0; i < DMA_MAX_CBS_PER_CHANNEL; ++i) {
        memset (&(allocated_channel_cbs[channel_id + i]), 0,
                sizeof (anonymous_control_block_s));
    }
    return DMA_OK;
}

/*
 * \brief
 * \param control_block The src and dest addresses MUST be VPU Bus Addresses not
 * SRAM Physical addresses. Because of this both the src and dest addresses must
 * also point to a location in the 1st GB of SRAM. Unless DMA4 is used. DMA4 can
 * use 40 bit addresses.
 */
dma_status_t dma_transfer (anonymous_control_block_s cbs[DMA_MAX_CBS_PER_CHANNEL],
                           u32 ncbs,
                           u32 channel_id,
                           dma_type_t dma_type) {
    if (DMA_CHANNEL_HAS_ERROR (channel_id)) {
        LOG_ERROR ("Previous DMA transfer failed");
    }

    /*
     * If multiple control blocks are given, link 1st to 2nd, 2nd to 3rd....
     */
    for (u32 i = 0; i < ncbs; ++i) {
        if ((i + 1) < ncbs) {
            if (dma_type == DMA_STANDARD) {
                control_block_s* dma_block = (control_block_s*)&(cbs[i]);
                dma_block->next_control_block_addr = ARM_TO_VPU_BUS_ADDR (
                (uintptr_t) & (allocated_channel_cbs[channel_id + i + 1]));
            } else if (dma_type == DMA_LITE) {
                dma_lite_control_block_s* dma_block =
                (dma_lite_control_block_s*)&(cbs[i]);
                dma_block->next_control_block_addr = ARM_TO_VPU_BUS_ADDR (
                (uintptr_t) & (allocated_channel_cbs[channel_id + i + 1]));
            } else if (dma_type == DMA4) {
                dma4_control_block_s* dma_block = (dma4_control_block_s*)&(cbs[i]);
                dma_block->next_control_block_addr = ARM_TO_VPU_BUS_ADDR (
                (uintptr_t) & (allocated_channel_cbs[channel_id + i + 1]));
            }
        }

        memcopy (&(cbs[i]), sizeof (anonymous_control_block_s),
                 &(allocated_channel_cbs[channel_id + i]));
    }

    u32 control_block_addr = (u32)(&(allocated_channel_cbs[channel_id]));

    if (dma_type == DMA4) {
        control_block_addr >>= DMA4_CB_ADDR_SHIFT;
    } else {
        // NOTE: only have 32 - 5 bits to address each control block.
        // allocated_channel_cbs is currently stored somewhere around 0x84260 in
        // the .bss section. If it is moved and the address takes more than 27
        // bits to represent the u32 cast will truncate it and weird things will
        // happen.
        control_block_addr = ARM_TO_VPU_BUS_ADDR (control_block_addr);
    }
    write32 (DMA_CONTROL_BLOCK_ADDR (channel_id), control_block_addr);
    // Ensure control block address is written and visible to DMA before DMA is activated
    DATA_MEMORY_BARRIER_OUTER_STORES ();
    // Clear error, interrupt flags and enable channel
    write32 (DMA_CS (channel_id), read32 (DMA_CS (channel_id)) | DMA_CS_INIT_TRANSFER_VAL);

    return DMA_OK;
}

static dma_status_t dma_setup_memcpy_ (u64 src_addr,
                                       u64 dest_addr,
                                       size_t transfer_length,
                                       anonymous_control_block_s* block,
                                       u32 channel_id,
                                       dma_type_t dma_type) {
    u64 src_bus  = ARM_TO_VPU_BUS_ADDR (src_addr);
    u64 dest_bus = ARM_TO_VPU_BUS_ADDR (dest_addr);
    memset (block, 0, sizeof (anonymous_control_block_s));

    if (dma_type == DMA_STANDARD) {
        control_block_s* dma_block = (control_block_s*)block;
        dma_block->source_addr     = src_bus;
        dma_block->dest_addr       = dest_bus;
        dma_block->transfer_length = transfer_length;
    } else if (dma_type == DMA_LITE) {
        dma_lite_control_block_s* dma_block = (dma_lite_control_block_s*)block;
        dma_block->source_addr              = src_bus;
        dma_block->dest_addr                = dest_bus;
        dma_block->transfer_length          = transfer_length;
    } else if (dma_type == DMA4) {
        dma4_control_block_s* dma_block = (dma4_control_block_s*)block;
        dma_block->source_addr          = src_bus;
        dma_block->dest_addr            = dest_bus;
        dma_block->transfer_length      = transfer_length;
    } else {
        LOG_ERROR ("Incorrect dma_type: [%u]", dma_type);
        return DMA_ERROR_ON_TRANSFER;
    }
    return DMA_OK;
}

dma_status_t dma_memcpy (u64 src_addr, u64 dest_addr, size_t transfer_length, dma_type_t dma_type) {
    u32 channel_id;
    dma_status_t status = dma_allocate_channel (dma_type, &channel_id);
    if (status != DMA_OK) {
        return status;
    } else if (transfer_length > (DMA_MAX_TRANSFER_LENGTH * DMA_MAX_CBS_PER_CHANNEL)) {
        return DMA_TRANSFER_TOO_LARGE;
    }

    u32 idx                 = channel_id;
    s32 n_overflow_cbs      = (transfer_length / DMA_MAX_TRANSFER_LENGTH);
    s32 max_failed_attempts = DMA_MAX_CBS_PER_CHANNEL * 2;
    anonymous_control_block_s blocks[DMA_MAX_CBS_PER_CHANNEL];
    while (n_overflow_cbs > 0 && max_failed_attempts > 0) {
        status = dma_setup_memcpy_ (src_addr, dest_addr, DMA_MAX_TRANSFER_LENGTH,
                                    &(blocks[idx++]), channel_id, dma_type);
        if (status == DMA_OK) {
            src_addr += DMA_MAX_TRANSFER_LENGTH;
            dest_addr += DMA_MAX_TRANSFER_LENGTH;
            transfer_length -= DMA_MAX_TRANSFER_LENGTH;
            --n_overflow_cbs;
        } else {
            --max_failed_attempts;
        }
    }
    status = dma_setup_memcpy_ (src_addr, dest_addr, transfer_length,
                                &(blocks[channel_id]), channel_id, dma_type);
    if (status == DMA_OK) {
        status = dma_transfer (&(blocks[channel_id]), n_overflow_cbs + 1, channel_id, dma_type);
    }

    dma_delete_channel (channel_id);
    return status;
}

static void dma_irq_default_handler (u32 irq_id) {
    s32 channel_id = (irq_id - VC_GIC_DMA_IRQ_START);
    if (channel_id < 0 || channel_id > MAX_DMA_CHANNEL_ID) {
        return;
    }
    // u32 status = read32 (DMA_CS (channel_id));
    if (DMA_CHANNEL_HAS_ERROR (channel_id)) {
        u32 error = read32 (DMA_DEBUG (channel_id));
        switch (error & 0x7) {
        case 0x7:
            LOG_ERROR ("Read, FIFO, and Read Not Set errors for DMA channel "
                       "[%u]",
                       channel_id);
            break;
        case 0x6:
            LOG_ERROR ("Read and FIFO errors for DMA channel [%u]", channel_id);
            break;
        case 0x5:
            LOG_ERROR ("Read and Read Not Set errors for DMA channel [%u]", channel_id);
            break;
        case 0x4:
            LOG_ERROR ("Read error for DMA channel [%u]", channel_id);
            break;
        case 0x3:
            LOG_ERROR ("FIFO and Read Not Set errors for DMA channel [%u]", channel_id);
            break;
        case 0x2:
            LOG_ERROR ("FIFO error for DMA channel [%u]", channel_id);
            break;
        case 0x1:
            LOG_ERROR ("Read Not Set error for DMA channel [%u]", channel_id);
            break;
        case 0x0: break;
        }
    }
    dma_delete_channel (channel_id);
}

s32 dma_init (dma_config_s* config) {
    memset ((void*)allocated_channel_cbs, 0, sizeof (allocated_channel_cbs));
    memset ((void*)is_channel_allocated, 0, sizeof (is_channel_allocated));

    for (u32 i = VC_GIC_DMA_IRQ_START; i <= VC_GIC_DMA_IRQ_END; ++i) {
        gic_enable_interrupt (i, &dma_irq_default_handler);
    }

    for (u32 i = 0; i < DMA_4_CHANNEL_END + 1; ++i) {
        /*
         * Enable interrupts by default for all channels
         */
        write32 (DMA_TF_INFO (i), read32 (DMA_TF_INFO (i)) | DMA_TF_INFO_INT_ENABLE_BIT);
    }

    DATA_MEMORY_BARRIER_OUTER_STORES ();

    if (config == NULLPTR) {
        dma_config.dma_start_id      = DMA_CHANNEL_START;
        dma_config.dma_lite_start_id = DMA_LITE_CHANNEL_START;
        dma_config.dma4_start_id     = DMA_4_CHANNEL_START;
        dma_config.dma4_end_id       = DMA_4_CHANNEL_END;
    } else {
        dma_config = *config;
    }
    return 1;
}