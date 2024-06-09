#include "peripherals/vpu.h"
#include "peripherals/timer.h"
#include "sync.h"

#define VPU_BASE         (PERIPH_BASE + 0x200B880)
#define VPU_MBOX0_READ   (VPU_BASE + 0x0)
// #define VPU_MBOX_POLL   (VPU_BASE + 0x10)
// #define VPU_MBOX_SENDER (VPU_BASE + 0x14)
#define VPU_MBOX0_STATUS (VPU_BASE + 0x18)
// #define VPU_MBOX_CONFIG (VPU_BASE + 0x1C)
#define VPU_MBOX1_WRITE  (VPU_BASE + 0x20)
#define VPU_MBOX1_STATUS (VPU_BASE + 0x38)
#define VPU_MBOX_EMPTY   0x40000000
#define VPU_MBOX_FULL    0x80000000

static u32 vpu_read_ (u8 channel) {
    u32 result;
    do {
        while (read32 (VPU_MBOX0_STATUS) & VPU_MBOX_EMPTY) {
        }
        result = read32 (VPU_MBOX0_READ);
    } while ((result & 0xF) != channel);
    return result;
}

static void vpu_write_ (uintptr_t addr, u32 size_inbytes, u8 channel) {
    while (read32 (VPU_MBOX1_STATUS) & VPU_MBOX_FULL) {
    }
    clean_invalidate_data_cache_vaddr (addr, size_inbytes);
    write32 (VPU_MBOX1_WRITE, ARM_TO_VPU_BUS_ADDR (addr) | channel);
}

static void vpu_flush_ (void) {
    while (!(read32 (VPU_MBOX0_STATUS) & VPU_MBOX_EMPTY)) {
        read32 (VPU_MBOX0_READ);
        wait_ms (20);
    }
}
/*
 * The alignment attribute specifies the alignment of variables
 * or structure fields, not single array elements.
 *
 * 28-bit address (MSB) and 4-bit value (LSB)
 * 0nly the upper 28 bits of the address can be passed via the mailbox
 * and the last 4 bits are the channel. This is why the buffer is 16 byte
 * aligned. So the memory address to the buffer will only use the msb 28
 * bits and the lsb 4 bits will be free
 *
 * NOTE: address and size of cmd_buffer_ HAS to be a multiple of the
 * cache line size
 */
static u32 volatile GCC_CACHE_ALIGNED cmd_buffer_[VPU_CMD_BUFFER_SIZE] = { 0 };

// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
vpu_status_t vpu_call (u32 (*buffer_ptr)[VPU_CMD_BUFFER_SIZE], u8 channel) {
    memset ((void*)cmd_buffer_, 0, sizeof (cmd_buffer_));
    memcpy (*buffer_ptr, sizeof (cmd_buffer_), (void*)cmd_buffer_);
    DATA_MEMORY_BARRIER_INNER_STORES ();
    if (channel > 0xF) {
        LOG_ERROR ("Max Channel is 4 bits in size. Got [%u]", channel);
        return 0;
    }
    vpu_flush_ ();
    vpu_write_ ((uintptr_t)cmd_buffer_, sizeof (cmd_buffer_), channel);
    u32 recv_addr = vpu_read_ (channel);
    u32 sent_addr = ARM_TO_VPU_BUS_ADDR (cmd_buffer_) | channel;
    // Make sure to read from the cmd buffer in DRAM and not a stale version
    // in the cache.
    invalidate_data_cache_vaddr ((uintptr_t)cmd_buffer_, sizeof (cmd_buffer_));
    u32 response_code = cmd_buffer_[1];
    // The address sent to the vpu needs to match the received address
    if (recv_addr != sent_addr) {
        LOG_ERROR ("VPU sent addr [%u] != recv addr [%u] because [0x%X]", sent_addr, recv_addr, response_code);
        return 0;
    }
    // 0x80000000 is the valid response code
    if (response_code != 0x80000000) {
        LOG_ERROR ("VPU sent invalid response code [0x%X]", response_code);
        return 0;
    }
    memcpy ((void*)cmd_buffer_, sizeof (cmd_buffer_), *buffer_ptr);
    return eVPU_STATUS_OK;
}