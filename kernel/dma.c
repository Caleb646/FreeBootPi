#include "dma.h"
#include "irq.h"
#include "mem.h"
#include "printf.h"
#include "sync.h"

#define DMA_BASE           (PBASE + 0x2007000)
#define DMA_CHANNEL_OFFSET 0x100
/*
 * Bit 31 = Reset = Writing 1 will reset DMA. WO and will self clear
 * Bit 30 = Abort = Writing 1 will abort current DMA Control Block. Will next CB
 * and continue Bit 8 = Error = is 1 if error flag set. RO Bit 6 = DMA waiting
 * on outstanding writes to be received if 1. RO Bit 4 = Paused = DMA is paused
 * if 1 Bit 2 = Interrupt status. Is 1 when transfer for the CB ends if INTEN
 * flag is set. Write 1 to clear. Must be cleared Bit 1 = Is 1, when transfer to
 * current CB is complete. Write 1 to clear Bit 0 = Set to 1 to enable DMA
 */
#define CS(channel_id)     (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x000)
#define CS_RESET_BIT       (1 << 31)
#define CS_ABORT_BIT       (1 << 30)
#define CS_ERROR_BIT       (1 << 8)
#define CS_WAITING_BIT     (1 << 6)
#define CS_PAUSED_BIT      (1 << 4)
/*
 * This is set when the transfer for the CB ends and INTEN is set to 1.
 * Once set it must be manually cleared down, even if the next CB has INTEN = 0.
 * Write 1 to clear.
 */
#define CS_INT_BIT         (1 << 2)
/*
 * Set when the transfer described by the current Control Block is complete. Write 1 to clear.
 */
#define CS_END_BIT         (1 << 1)
#define CS_DMA_ACTIVE_BIT  (1 << 0)
#define CS_ABORT_VAL       (CS_ABORT_BIT | CS_INT_BIT | CS_ERROR_BIT)
#define CS_INIT_TRANSFER_VAL \
    (CS_ERROR_BIT | CS_DMA_ACTIVE_BIT | CS_INT_BIT | CS_END_BIT)

/*
 * This tells the DMA where to find a Control Block stored in
 * memory. When the ACTIVE bit is set and this address is
 * non zero, the DMA will begin its transfer by loading the
 * contents of the addressed CB into the relevant DMA
 * channel registers.
 *
 * At the end of the transfer this register will be updated with
 * the ADDR field of the NEXTCONBK Control Block register. If
 * this field is zero, the DMA will stop.
 *
 * NOTE: The address must be 256-bit aligned, so
 * the bottom 5 bits of the address must be zero.
 *
 */
#define DMA_CONTROL_BLOCK_ADDR(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x004)

/*
 * Bit 0 = INTEN = Interrupt enable, set to 1 to generate interrupt when transfer is complete
 */
#define DMA_TF_INFO(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x008)

/*
 * DMA Source Address
 * Source address for the DMA operation. Updated by the
 * DMA engine as the transfer progresses.
 */
#define DMA_SOURCE_ADDR(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x00C)
#define DMA_DEST_ADDR(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x010)
/*
 * Specifies the amount of data to be transferred in bytes.
 * In normal (non 2D) mode this specifies the amount of bytes to be transferred.
 *
 * In 2D mode it is interpreted as an X and a Y length, and the DMA will perform Y transfers, each of length X bytes and
 * add the strides onto the addresses after each X leg of the transfer.
 */
#define DMA_TRANSFER_LENGTH(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x014)

/*
 * If in 2D Mode:
 *   Bits 31:16 = Signed (2 s complement) byte increment to apply to the
 *   destination address at the end of each row in 2D mode.
 *
 *   Bits 15:0 = Signed (2 s complement) byte increment to apply to the
 *   source address at the end of each row in 2D mode.
 */
#define DMA_STRIDE(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x018)

/*
 * Address of next CB for chained DMA operations.
 *
 * The address must be 256-bit aligned and so the
 * bottom 5 bits cannot be set and will read back as zero.
 */
#define DMA_NEXT_CB_ADDR(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x01C)

/*
* Bit  28 LITE DMA Lite = Set if the DMA is a reduced performance LITE engine. RO 0x0
* Bits 27:25 VERSION DMA Version DMA version number, indicating control bit field changes. RO 0x2
* Bits 24:16 DMA_STATE DMA State Machine State Returns the value of the
        DMA engine’s state machine for this channel. RO 0x000
* Bits 15:8 DMA_ID DMA ID Returns the DMA AXI ID of this DMA channel. RO 0x00
* Bits 7:4 OUTSTANDING_WRITES = DMA Outstanding Writes Counter
* Bit  2 READ_ERROR Slave Read Response Error = Set if the read operation
        returned an error value on the read response bus.
        It can be cleared by writing a 1.
* Bit 1 FIFO_ERROR FIFO Error = Set if the optional read FIFO records an error condition.
        It can be cleared by writing a 1.
* Bit 0 READ_LAST_NOT_SET_ERROR = If the AXI read last signal was not set when expected, then
        this error bit will be set. It can be cleared by writing a 1.
*/
#define DMA_DEBUG(channel_id) \
    (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x020)

/*
 * Interrupt status of each DMA channel. Read ONLY
 */
#define DMA_GLOBAL_INT_STATUS  (DMA_BASE + 0xFE0)
/*
 * Global enable bits for each DMA channel.
 * Bits 31:28 = PAGELITE = Set the 1G SDRAM ram page that the DMA Lite engines
 *   (DMA7-10) will access when addressing the 1G uncached
 *   range C000_0000->ffff_ffff
 *   E.g. setting this to 1 will mean that when the DMA writes to
 *   C000_0000 (uncached) the final address in SDRAM will be
 *   4000_0000 ( pagelite<<30 | addr[29:0] )
 *   This allows the 1G uncached page to be moved around the
 *   16G SDRAM space
 *
 * Bits 27:24 = PAGE = Set the 1G SDRAM ram page that the 30-bit DMA engines
 *   (DMA0-6) will access when addressing the 1G uncached
 *   range C000_0000->ffff_ffff
 *   E.g. setting this to 1 will mean that when the DMA writes to
 *   C000_0000 (uncached) the final address in SDRAM will be
 *   4000_0000 ( page<<30 | addr[29:0] )
 *   This allows the 1G uncached page to be moved around the
 *   16G SDRAM space
 *
 * Bits 14:0 = ENABLE = Control which DMA channels are enabled.
 *   If Bit 14 = 1 then DMA channel 14 = enabled
 *   ALL channels are enabled by default.
 */
#define DMA_GLOBAL_ENABLE      (DMA_BASE + 0xFF0)

/*
 * DMA Channel 15 is exclusively used by the VPU
 *
 * DMA channels 0 through 6 are Standard DMA channels
 * DMA channels 7 through 10 are DMA Lite channels
 * DMA channels 11 through 14 are DMA 4 channels
 */
#define MAX_DMA_CHANNEL_ID     14
#define DMA_CHANNEL_START      0
#define DMA_CHANNEL_END        6
#define DMA_LITE_CHANNEL_START 7
#define DMA_LITE_CHANNEL_END   10
#define DMA_4_CHANNEL_START    11
#define DMA_4_CHANNEL_END      14
/*
 * A control block's address must be aligned on a 32 byte boundary
 */
#define DMA_CB_BYTE_ALIGNMENT  32
#define DMA4_CB_ADDR_SHIFT     5

#define DMA_CHANNEL_IS_ACTIVE(channel_id) \
    ((read32 (CS (channel_id)) & CS_DMA_ACTIVE_BIT) == CS_DMA_ACTIVE_BIT)
#define DMA_CHANNEL_HAS_ERROR(channel_id) \
    ((read32 (CS (channel_id)) & CS_ERROR_BIT) == CS_ERROR_BIT)

/*
 * GCC aligns the beginning of the array on a 32 byte boundary and because
 * a dma_cb_t is 32 bytes each elements' addr is also aligned on a 32
 * byte boundary
 */
static anonymous_cb_t GCC_ALIGN_ADDR (DMA_CB_BYTE_ALIGNMENT)
allocated_channel_cbs[(MAX_DMA_CHANNEL_ID + 1) * DMA_MAX_CBS_PER_CHANNEL];
static u32 volatile is_channel_allocated[MAX_DMA_CHANNEL_ID + 1];
static dma_config_s dma_config;

static s32 find_open_channel (dma_type_t dma_type) {
    u32 start, end;
    if (dma_type == DMA_STANDARD) {
        start = dma_config.dma_start_id;
        end   = dma_config.dma_lite_start_id - 1;
    } else if (dma_type == DMA_LITE) {
        start = dma_config.dma_lite_start_id;
        end   = dma_config.dma4_start_id - 1;
    } else if (dma_type == DMA4) {
        start = dma_config.dma4_start_id;
        end   = dma_config.dma4_end_id;
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

/*
 * \brief The DMA controller should NOT be active when this function is called.
 */
static dma_status_t dma_delete_channel (u32 channel_id) {
    {
        u32 status = read32 (CS (channel_id));
        /*
         * Double check that the DMA controller is paused. If not paused then
         * pause it.
         */
        if ((status & CS_PAUSED_BIT) > 0) {
            LOG_ERROR ("DMA trying to delete an active channel [%u]", channel_id);
            // Clear the active bit
            write32 (CS (channel_id), status & ~CS_DMA_ACTIVE_BIT);
            DATA_MEMORY_BARRIER_OUTER_STORES ();
        }
    }
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
     * Clear the end, error and interrupt bits and set the abort bit.
     */
    write32 (CS (channel_id), read32 (CS (channel_id)) | CS_ABORT_VAL | CS_END_BIT);
    is_channel_allocated[channel_id] = 0;
    /*
     * Ensure DMA channel is stopped before the current cb is overwritten
     */
    DATA_MEMORY_BARRIER_OUTER_STORES ();
    memset (&(allocated_channel_cbs[channel_id]), 0, sizeof (anonymous_cb_t) * DMA_MAX_CBS_PER_CHANNEL);
    return DMA_OK;
}

/*
 * \brief
 * \param control_block The src and dest addresses MUST be VPU Bus Addresses not
 * SRAM Physical addresses. Both the src and dest addresses must
 * also point to a location in the 1st GB of SRAM. Unless DMA4 is used. DMA4 can
 * use 40 bit addresses.
 */
dma_status_t
dma_transfer (anonymous_cb_t cbs[DMA_MAX_CBS_PER_CHANNEL], u32 ncbs, u32 channel_id, dma_type_t dma_type) {
    if (DMA_CHANNEL_HAS_ERROR (channel_id)) {
        LOG_ERROR ("Previous DMA transfer failed");
    }
    /*
     * If multiple control blocks are given, link 1st to 2nd, 2nd to 3rd....
     */
    for (u32 i = 0; i < ncbs; ++i) {
        if ((i + 1) < ncbs) {
            if (dma_type == DMA_STANDARD) {
                dma_cb_t* dma_block     = (dma_cb_t*)&(cbs[i]);
                dma_block->next_cb_addr = ARM_TO_VPU_BUS_ADDR (
                (uintptr_t) & (allocated_channel_cbs[channel_id + i + 1]));
            } else if (dma_type == DMA_LITE) {
                dma_lite_cb_t* dma_block = (dma_lite_cb_t*)&(cbs[i]);
                dma_block->next_cb_addr  = ARM_TO_VPU_BUS_ADDR (
                 (uintptr_t) & (allocated_channel_cbs[channel_id + i + 1]));
            } else if (dma_type == DMA4) {
                dma4_cb_t* dma_block    = (dma4_cb_t*)&(cbs[i]);
                dma_block->next_cb_addr = ARM_TO_VPU_BUS_ADDR (
                (uintptr_t) & (allocated_channel_cbs[channel_id + i + 1]));
            }
        }

        uintptr_t addr = (uintptr_t) & (allocated_channel_cbs[channel_id + i]);
        if ((addr & 0x1F) > 0) {
            LOG_ERROR ("DMA control block is NOT 32 byte aligned");
        }
        memcopy (&(cbs[i]), sizeof (anonymous_cb_t), &(allocated_channel_cbs[channel_id + i]));
    }
    // cb addrs are not 64 bits but this makes the compiler happy
    u64 control_block_addr = (u64)(&(allocated_channel_cbs[channel_id]));
    if (dma_type == DMA4) {
        // TODO: dma4 has two seperate registers for the low and high part of
        // of the address.
        control_block_addr >>= DMA4_CB_ADDR_SHIFT;
    } else {
        // NOTE: only have 32 - 5 (32 byte alignment) - 2 (vpu cache control)
        // bits to address each control block. vvaa ........ 0 0000
        // allocated_channel_cbs is currently stored somewhere around 0x84260 in
        // the .bss section. If it is moved and the address takes more than 27
        // bits to represent the u32 cast will truncate it and weird things will
        // happen. 0011 1111 1111 1111 1111 1111 1110 0000
        if (control_block_addr > 0x3FFFFFE0) {
            LOG_ERROR ("DMA control block address is out of bounds for DMA and "
                       "DMA Lite Channels");
            return DMA_ERROR_ON_SETUP;
        }
        control_block_addr = ARM_TO_VPU_BUS_ADDR (control_block_addr);
    }
    write32 (DMA_CONTROL_BLOCK_ADDR (channel_id), control_block_addr);
    // Ensure control block address is written and visible to DMA before DMA is activated
    DATA_MEMORY_BARRIER_FS_STORES ();
    // Clear error, end, interrupt bits and enable channel
    write32 (CS (channel_id), read32 (CS (channel_id)) | CS_INIT_TRANSFER_VAL);

    LOG_INFO ("Started DMA transfer for channel [%u]", channel_id);

    return DMA_OK;
}

static dma_status_t dma_setup_memcpy_ (
uintptr_t src_addr,
uintptr_t dest_addr,
size_t transfer_length,
u32 transfer_info,
anonymous_cb_t* empty_cb,
u32 channel_id,
dma_type_t dma_type) {

    uintptr_t src_bus  = ARM_TO_VPU_BUS_ADDR (src_addr);
    uintptr_t dest_bus = ARM_TO_VPU_BUS_ADDR (dest_addr);
    if (dma_type == DMA4) {
        src_bus  = src_addr;
        dest_bus = dest_addr;
    }
    memset (empty_cb, 0, sizeof (anonymous_cb_t));

    if (dma_type == DMA_STANDARD) {
        dma_cb_t* dma_block        = (dma_cb_t*)empty_cb;
        dma_block->source_addr     = src_bus;
        dma_block->dest_addr       = dest_bus;
        dma_block->transfer_length = transfer_length;
        dma_block->transfer_info   = transfer_info;
    } else if (dma_type == DMA_LITE) {
        dma_lite_cb_t* dma_block   = (dma_lite_cb_t*)empty_cb;
        dma_block->source_addr     = src_bus;
        dma_block->dest_addr       = dest_bus;
        dma_block->transfer_length = transfer_length;
        dma_block->transfer_info   = transfer_info;
    } else if (dma_type == DMA4) {
        dma4_cb_t* dma_block       = (dma4_cb_t*)empty_cb;
        dma_block->source_addr     = src_bus;
        dma_block->dest_addr       = dest_bus;
        dma_block->transfer_length = transfer_length;
        dma_block->transfer_info   = transfer_info;
    } else {
        LOG_ERROR ("Incorrect dma_type: [%u]", dma_type);
        return DMA_ERROR_ON_TRANSFER;
    }
    return DMA_OK;
}

/*
 * \brief
 * \param src_addr Should be located in the 1 GB of SRAM if using standard DMA
 * or DMA Lite. This is because the VPU uses the upper 2 bits of a 32 bit
 * address for cache control.
 * \param dest_addr Should be located in the 1 GB of
 * SRAM if using standard DMA or DMA Lite. This is because the VPU uses the
 * upper 2 bits of a 32 bit address for cache control.
 */
dma_status_t
dma_memcpy (uintptr_t src_addr, uintptr_t dest_addr, size_t transfer_length, dma_type_t dma_type) {
    u32 channel_id;
    dma_status_t status = dma_allocate_channel (dma_type, &channel_id);
    if (status != DMA_OK) {
        return status;
    } else if (transfer_length > (DMA_MAX_TRANSFER_LENGTH * DMA_MAX_CBS_PER_CHANNEL)) {
        return DMA_TRANSFER_TOO_LARGE;
    }

    u32 idx                 = 0;
    s32 n_overflow_cbs      = (transfer_length / DMA_MAX_TRANSFER_LENGTH);
    s32 max_failed_attempts = DMA_MAX_CBS_PER_CHANNEL * 2;
    /*
     * Enable interrupts for all Control Blocks
     */
    u32 transfer_info = DMA_INT_ENABLE;
    anonymous_cb_t blocks[DMA_MAX_CBS_PER_CHANNEL];
    // LOG_INFO ("Channel Allocated [%u] overflow cbs [%d]", channel_id, n_overflow_cbs);
    while (n_overflow_cbs > 0 && max_failed_attempts > 0) {
        status = dma_setup_memcpy_ (
        src_addr, dest_addr, DMA_MAX_TRANSFER_LENGTH, transfer_info,
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
    status = dma_setup_memcpy_ (
    src_addr, dest_addr, transfer_length, transfer_info, &(blocks[idx]), channel_id, dma_type);
    if (status == DMA_OK) {
        status = dma_transfer (blocks, n_overflow_cbs + 1, channel_id, dma_type);
    }
    return status;
}

static void dma_irq_handler (u32 irq_id) {
    /*
     * 3 Possible states when the irq handler is called:
     *   1. The 1 and only control block has finished its transfer or
     *       the last of several control blocks has finished.
     *   2. 1 of several control blocks has finished and the next
     *       control block is about to start.
     *   3. An error was encountered during the transfer.
     *       It possible that more than 1 control block is pending.
     */
    u32 channel_id = (irq_id - VC_GIC_DMA_IRQ_START);
    if (channel_id > MAX_DMA_CHANNEL_ID) {
        LOG_ERROR ("DMA IRQ handler received invalid channel id [%u]", channel_id);
        return;
    }
    u32 status       = read32 (CS (channel_id));
    u32 next_cb_addr = read32 (DMA_NEXT_CB_ADDR (channel_id));

    if ((status & CS_ERROR_BIT) == 0) {
        /*
         * The 1 and only control block has finished its transfer or
         * the last of several control blocks has finished.
         */
        if (next_cb_addr == 0) {
            LOG_INFO ("DMA Channel [%u] finished transfer", channel_id);
            dma_delete_channel (channel_id);
            return;
        }
        /*
         * 1 of several control blocks has finished and the next
         * control block is about to start.
         */
        else {
            /*
             * clear interrupt bit
             */
            write32 (CS (channel_id), status | CS_INT_BIT | CS_END_BIT);
            return;
        }
    }
    u32 error = read32 (DMA_DEBUG (channel_id));
    switch (error & 0x7) {
    case 0x7:
        LOG_ERROR (
        "Read, FIFO, and Read Not Set errors for DMA channel "
        "[%u]",
        channel_id);
        break;
    case 0x6:
        LOG_ERROR ("Read and FIFO errors for DMA channel [%u]", channel_id);
        break;
    case 0x5:
        LOG_ERROR ("Read and Read Not Set errors for DMA channel [%u]", channel_id);
        break;
    case 0x4: LOG_ERROR ("Read error for DMA channel [%u]", channel_id); break;
    case 0x3:
        LOG_ERROR ("FIFO and Read Not Set errors for DMA channel [%u]", channel_id);
        break;
    case 0x2: LOG_ERROR ("FIFO error for DMA channel [%u]", channel_id); break;
    case 0x1:
        LOG_ERROR ("Read Not Set error for DMA channel [%u]", channel_id);
        break;
    case 0x0: break;
    }
    /*
     * Clear all current and pending control blocks.
     * They will have to be re-initiated by the caller.
     */
    dma_delete_channel (channel_id);
}

s32 dma_init (dma_config_s* config) {
    memset ((void*)allocated_channel_cbs, 0, sizeof (allocated_channel_cbs));
    memset ((void*)is_channel_allocated, 0, sizeof (is_channel_allocated));

    /*
     * Enable DMA Interrupts on the GIC and register a handler
     */
    for (u32 i = VC_GIC_DMA_IRQ_START; i <= VC_GIC_DMA_IRQ_END; ++i) {
        gic_enable_interrupt (i, &dma_irq_handler);
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