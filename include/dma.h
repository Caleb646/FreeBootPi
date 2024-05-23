#ifndef _DMA_H
#define _DMA_H

#include "base.h"

/*
 * \brief Used with the `transfer_info` member in the control block structs.
 * Enables/Disables interrupts for the control block transfer complete event.
 * \param 1 = Generate an interrupt when the transfer described by the current
 * Control Block completes. \param 0 = Do not generate an interrupt.
 */
#define DMA_INT_ENABLE          (1 << 0)

/*
 * 2 most significant bits are reserved
 */
#define DMA_MAX_TRANSFER_LENGTH 0x3FFFFFFFUL
/*
 * Determines how many control blocks can be linked together for a single
 * transfer
 */
#define DMA_MAX_CBS_PER_CHANNEL 3

#ifndef __ASSEMBLER__

typedef struct dma_config_s {
    u32 dma_start_id;
    u32 dma_lite_start_id;
    u32 dma4_start_id;
    u32 dma4_end_id;
} dma_config_s;

typedef struct anonymous_cb_t {
    u32 a1;
    u32 a2;
    u32 a3;
    u32 a4;
    u32 a5;
    u32 a6;
    u32 a7;
    u32 a8;
} anonymous_cb_t;

// extern anonymous_cb_t GCC_ALIGN_ADDR(DMA_CB_BYTE_ALIGNMENT) allocated_channel_cbs[MAX_DMA_CHANNEL_ID + 1];

/*
 * A control block's address must be aligned on a 32 byte boundary
 * Each 32-bit word of the Control Block is automatically loaded into the
 * corresponding 32-bit DMA Control Block register at the start of a DMA transfer:
 *      transfer_info -> TI_INFO Register
 *      source_addr   -> SOURCE_AD Register
 */
typedef struct dma_cb_t {
    u32 transfer_info;
    u32 source_addr;
    u32 dest_addr;
    u32 transfer_length;
    u32 stride_mode;
    u32 next_cb_addr;
    u32 reserved1;
    u32 reserved2;
} dma_cb_t;

typedef struct dma_lite_cb_t {
    u32 transfer_info;
    u32 source_addr;
    u32 dest_addr;
    u32 transfer_length;
    u32 reserved1;
    u32 next_cb_addr;
    u32 reserved2;
    u32 reserved3;
} dma_lite_cb_t;

typedef struct dma4_cb_t {
    u32 transfer_info;
    u32 source_addr;
    u32 source_info;
    u32 dest_addr;
    u32 dest_info;
    u32 transfer_length;
    u32 next_cb_addr;
    u32 reserved;
} dma4_cb_t;

typedef enum dma_type_t { DMA_STANDARD, DMA_LITE, DMA4 } dma_type_t;

typedef enum dma_status_t {
    DMA_OK = 1,
    DMA_NO_OPEN_CHANNELS,
    DMA_ERROR_ON_TRANSFER,
    DMA_TRANSFER_TOO_LARGE,
    DMA_FAILED_ALLOC_CB

} dma_status_t;

dma_status_t
dma_transfer (anonymous_cb_t cbs[DMA_MAX_CBS_PER_CHANNEL], u32 ncbs, u32 channel_id, dma_type_t dma_type);
dma_status_t
dma_memcpy (uintptr_t src_addr, uintptr_t dest_addr, size_t transfer_length, dma_type_t dma_type);
s32 dma_init (dma_config_s* config);

#endif // __ASSEMBLER__

#endif /*_DMA_H */