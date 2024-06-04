#include "peripherals/vpu.h"
#include "sync.h"

#define VPU_BASE        (PERIPH_BASE + 0x200B880)
#define VPU_MBOX_READ   (VPU_BASE + 0x0)
#define VPU_MBOX_POLL   (VPU_BASE + 0x10)
#define VPU_MBOX_SENDER (VPU_BASE + 0x14)
#define VPU_MBOX_STATUS (VPU_BASE + 0x18)
#define VPU_MBOX_CONFIG (VPU_BASE + 0x1C)
#define VPU_MBOX_WRITE  (VPU_BASE + 0x20)
#define VPU_IS_VALID_MBOX_RESPONSE(response) \
    (response == VPU_MBOX_RESPONSE_CODE)
#define VPU_IS_MBOX_REPLY(data_sent) (read32 (VPU_MBOX_READ) == data_sent)
#define VPU_IS_MBOX_FULL             (read32 (VPU_MBOX_STATUS) & VPU_MBOX_RESPONSE_CODE)
#define VPU_IS_MBOX_EMPTY            (read32 (VPU_MBOX_STATUS) & 0x40000000)

/*
 * The alignment attribute specifies the alignment of variables
 * or structure fields, not single array elements.
 *
 * 28-bit address (MSB) and 4-bit value (LSB)
 * 0nly the upper 28 bits of the address can be passed via the mailbox
 * and the last 4 bits are the channel. This is why the buffer is 16 byte
 * aligned. So the memory address to the buffer will only use the msb 28
 * bits and the lsb 4 bits will be free
 */
static u32 volatile GCC_ALIGN_ADDR (16) cmd_buffer_[VPU_CMD_BUFFER_SIZE] = { 0 };

// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
vpu_status_t vpu_call (u32 (*buffer_ptr)[VPU_CMD_BUFFER_SIZE], u8 channel) {
    memset ((void*)cmd_buffer_, 0, sizeof (cmd_buffer_));
    memcpy (*buffer_ptr, sizeof (cmd_buffer_), (void*)cmd_buffer_);
    if (channel > 0xF) {
        LOG_ERROR ("Max Channel is 4 bits in size. Got [%u]", channel);
        return 0;
    }
    // Wait until we can write
    while (VPU_IS_MBOX_FULL) {
    }
    // Clean and Invalidate the cmd buffer from the cache
    clean_invalidate_data_cache_vaddr ((uintptr_t)cmd_buffer_, sizeof (cmd_buffer_));
    // Write the address of our buffer
    // to the mailbox with the channel appended
    u32 buff_addr_w_channel = ARM_TO_VPU_BUS_ADDR (cmd_buffer_) | channel;
    write32 (VPU_MBOX_WRITE, buff_addr_w_channel);
    while (1) {
        while (VPU_IS_MBOX_EMPTY) {
        }
        if (VPU_IS_MBOX_REPLY (buff_addr_w_channel)) {
            LOG_DEBUG ("Received reply from VPU: Response [%X] -- Channel ID [%u]", cmd_buffer_[1], channel);
            // response should always be at index 1
            if (VPU_IS_VALID_MBOX_RESPONSE (cmd_buffer_[1])) {
                memcpy ((void*)cmd_buffer_, sizeof (cmd_buffer_), *buffer_ptr);
                return eVPU_STATUS_OK;
            }
            return 0;
        }
    }
    return 0;
}