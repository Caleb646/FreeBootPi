#include "peripherals/vidcore.h"

// The alignment attribute specifies the alignment of variables or structure fields, not single array elements.
u32 volatile __attribute__((aligned(16))) vid_core_buffer[36];

// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
s32 vid_core_call(u32 volatile* buffer, u8 vid_core_channel)
{
    // 28-bit address (MSB) and 4-bit value (LSB)
    // 0nly the upper 28 bits of the address can be passed via the mailbox
    // and the last 4 bits are the channel. This is why the buffer is 16 byte aligned.
    // So the memory address to the buffer will only use the msb 28 bits and 
    // the lsb 4 bits will be free
    u32 buff_addr_w_channel = ((u32)((long) &buffer) &~ 0xF) | (vid_core_channel & 0xF);
    // Wait until we can write
    while(VID_CORE_IS_MBOX_FULL);
    // Write the address of our buffer to the mailbox with the channel appended
    REG_PTR32(VID_CORE_MBOX_WRITE) = buff_addr_w_channel;
    while (1) {
        while(VID_CORE_IS_MBOX_EMPTY);
        if (VID_CORE_IS_MBOX_REPLY(buff_addr_w_channel))
        {
            return VID_CORE_IS_VALID_MBOX_RESPONSE(buffer[1]); // response should always be at index 1
        } 
    }
    return 0;
}