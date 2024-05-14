#ifndef	_DMA_H
#define	_DMA_H

#include "base.h"

#define DMA_BASE                                    (PBASE + 0x2007000)
#define DMA_CHANNEL_OFFSET                          0x100
/*
* Bit 31 = Reset = Writing 1 will reset DMA. WO and will self clear
* Bit 30 = Abort = Writing 1 will abort current DMA Control Block. Will next CB and continue
* Bit 8 = Error = is 1 if error flag set. RO
* Bit 6 = DMA waiting on outstanding writes to be received if 1. RO
* Bit 4 = Paused = DMA is paused if 1
* Bit 2 = Interrupt status. Is 1 when transfer for the CB ends if INTEN flag is set. Write 1 to clear. Must be cleared
* Bit 1 = Is 1, when transfer to current CB is complete. Write 1 to clear
* Bit 0 = Set to 1 to enable DMA
*/
#define DMA_CONTROL_STATUS(channel_id)              (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x000)
#define DMA_CONTROL_STATUS_RESET_BIT                (1 << 31)
#define DMA_CONTROL_STATUS_ABORT_BIT                (1 << 30)
#define DMA_CONTROL_STATUS_ERROR_BIT                (1 << 8 )
#define DMA_CONTROL_STATUS_WAITING_BIT              (1 << 6 )
#define DMA_CONTROL_STATUS_PAUSED_BIT               (1 << 4 )
#define DMA_CONTROL_STATUS_INT_STATUS_BIT           (1 << 2 )
#define DMA_CONTROL_STATUS_TRANSFER_COMPLETE_BIT    (1 << 1 )
#define DMA_CONTROL_STATUS_DMA_ACTIVE_BIT           (1 << 0 )

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
#define DMA_CONTROL_BLOCK_ADDR(channel_id)          (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x004)

/*
* Bit 0 = INTEN = Interrupt enable, set to 1 to generate interrupt when transfer is complete
*/
#define DMA_TRANSFER_INFO(channel_id)               (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x008)
#define DMA_TRANSFER_INFO_INT_ENABLE_BIT            (1 << 0)

/*
* DMA Source Address
* Source address for the DMA operation. Updated by the
* DMA engine as the transfer progresses.
*/
#define DMA_SOURCE_ADDR(channel_id)                 (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x00C)
#define DMA_DEST_ADDR(channel_id)                   (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x010)
/*
* Specifies the amount of data to be transferred in bytes.
* In normal (non 2D) mode this specifies the amount of bytes to be transferred.
* 
* In 2D mode it is interpreted as an X and a Y length, and the DMA will perform Y transfers, each of length X bytes and
* add the strides onto the addresses after each X leg of the transfer.
*/
#define DMA_TRANSFER_LENGTH(channel_id)             (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x014)

/*
* If in 2D Mode:
*   Bits 31:16 = Signed (2 s complement) byte increment to apply to the
*   destination address at the end of each row in 2D mode.
*
*   Bits 15:0 = Signed (2 s complement) byte increment to apply to the
*   source address at the end of each row in 2D mode.
*/
#define DMA_STRIDE(channel_id)                      (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x018)

/* 
* Address of next CB for chained DMA operations.
* 
* The address must be 256-bit aligned and so the
* bottom 5 bits cannot be set and will read back as zero.
*/
#define DMA_NEXT_CONTROL_BLOCK_ADDR(channel_id)     (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x01C)

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
#define DMA_DEBUG(channel_id)                       (DMA_BASE + (DMA_CHANNEL_OFFSET * channel_id) + 0x020)

/*
* Interrupt status of each DMA channel. Read ONLY
*/
#define DMA_GLOBAL_INT_STATUS                       (DMA_BASE + 0xFE0)
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
#define DMA_GLOBAL_ENABLE                           (DMA_BASE + 0xFF0)

/*
* DMA Channel 15 is exclusively used by the VPU
*
* DMA channels 0 through 6 are Standard DMA channels
* DMA channels 7 through 10 are DMA Lite channels
* DMA channels 11 through 14 are DMA 4 channels
*/
#define MAX_DMA_CHANNEL_ID                      14
#define DMA_CHANNEL_START                       0
#define DMA_CHANNEL_END                         6
#define DMA_LITE_CHANNEL_START                  7
#define DMA_LITE_CHANNEL_END                    10
#define DMA_4_CHANNEL_START                     11
#define DMA_4_CHANNEL_END                       14
/*
* A control block's address must be aligned on a 32 byte boundary
*/
#define DMA_CONTROL_BLOCK_BYTE_ALIGNMENT        32
#define DMA4_CB_ADDR_SHIFT                      5
/*
* 2 most significant bits are reserved
*/
#define DMA_MAX_TRANSFER_LENGTH                 0x3FFFFFFF

#ifndef __ASSEMBLER__

#define DMA_CHANNEL_IS_ACTIVE(channel_id)       ((get32(DMA_CONTROL_STATUS(channel_id)) & DMA_CONTROL_STATUS_DMA_ACTIVE_BIT) == DMA_CONTROL_STATUS_DMA_ACTIVE_BIT)
#define DMA_CHANNEL_HAS_ERROR(channel_id)       ((get32(DMA_CONTROL_STATUS(channel_id)) & DMA_CONTROL_STATUS_ERROR_BIT) == DMA_CONTROL_STATUS_ERROR_BIT)

typedef struct dma_config_s
{
    u32 dma_start_id;
    u32 dma_lite_start_id;
    u32 dma4_start_id;
    u32 dma4_end_id;
} dma_config_s;

typedef struct anonymous_control_block_s
{
    u32 a1;
    u32 a2;
    u32 a3;
    u32 a4;
    u32 a5;
    u32 a6;
    u32 a7;
    u32 a8;
} anonymous_control_block_s;

// extern anonymous_control_block_s GCC_ALIGN_ADDR(DMA_CONTROL_BLOCK_BYTE_ALIGNMENT) allocated_channel_cbs[MAX_DMA_CHANNEL_ID + 1];

/*
* A control block's address must be aligned on a 32 byte boundary
*/
typedef struct control_block_s
{
    u32 transfer_info;
    u32 source_addr;
    u32 dest_addr;
    u32 transfer_length;
    u32 stride_mode;
    u32 next_control_block_addr;
    u32 reserved1;
    u32 reserved2;
} control_block_s;

typedef struct dma_lite_control_block_s
{
    u32 transfer_info;
    u32 source_addr;
    u32 dest_addr;
    u32 transfer_length;
    u32 reserved1;
    u32 next_control_block_addr;
    u32 reserved2;
    u32 reserved3;
} dma_lite_control_block_s;

typedef struct dma4_control_block_s
{
    u32 transfer_info;
    u32 source_addr;
    u32 source_info;
    u32 dest_addr;
    u32 dest_info;
    u32 transfer_length;
    u32 next_control_block_addr;
    u32 reserved;
} dma4_control_block_s;

typedef enum dma_type_t 
{
    DMA_STANDARD,
    DMA_LITE,
    DMA4
} dma_type_t;

typedef enum dma_status_t 
{
    DMA_OK = 1,
    DMA_NO_OPEN_CHANNELS,
    DMA_ERROR_ON_TRANSFER,
    DMA_FAILED_ALLOC_CB
    
} dma_status_t;

dma_status_t dma_transfer(anonymous_control_block_s* control_block, dma_type_t dma_type);
dma_status_t dma_memcpy(u64 src_addr, u64 dest_addr, size_t transfer_length, dma_type_t dma_type);
s32 dma_init(dma_config_s* config);

#endif // __ASSEMBLER__

#endif  /*_DMA_H */