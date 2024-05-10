#ifndef	_DMA_H
#define	_DMA_H

#include "base.h"
#include "mem.h"

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
* DMA Channel 15 is exclusively used by the VPU
*
* DMA channels 0 through 6 are Standard DMA channels
* DMA channels 7 through 10 are DMA Lite channels
* DMA channels 11 through 14 are DMA 4 channels
*/
#define MAX_DMA_CHANNEL_ID                      14
/*
* A control block's address must be aligned on a 32 byte boundary
*/
#define DMA_CONTROL_BLOCK_BYTE_ALIGNMENT        32

#define DMA_CHANNEL_IS_ACTIVE(channel_id)       ((REG_PTR32(DMA_CONTROL_STATUS(channel_id)) & DMA_CONTROL_STATUS_DMA_ACTIVE_BIT) == DMA_CONTROL_STATUS_DMA_ACTIVE_BIT)
#define DMA_CHANNEL_HAS_ERROR(channel_id)       ((REG_PTR32(DMA_CONTROL_STATUS(channel_id)) & DMA_CONTROL_STATUS_ERROR_BIT) == DMA_CONTROL_STATUS_ERROR_BIT)

#ifndef __ASSEMBLER__

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
    u64 reserved;
} control_block_s;

typedef struct dma_lite_control_block_s
{
    u32 transfer_info;
    u32 source_addr;
    u32 dest_addr;
    u32 transfer_length;
    u32 reserved1;
    u32 next_control_block_addr;
    u64 reserved2;
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
    DMA_CHANNEL_OCCUPIED,
    DMA_ERROR_ON_TRANSFER,
    DMA_OK,
} dma_status_t;

/*
* Dynamically allocate and zero initialize control block
*/
#define DMA_CONTROL_BLOCK_INIT()     (align_allocate_set(sizeof(control_block_s), 0, DMA_CONTROL_BLOCK_BYTE_ALIGNMENT))

void dma_allocate_channel(u32 channel_id);
void dma_start_transfer(u32 channel_id);
dma_status_t dma_transfer(void* control_block, u32 channel_id, dma_type_t dma_type);

s32 dma_init(void);

#endif // __ASSEMBLER__

#endif  /*_DMA_H */