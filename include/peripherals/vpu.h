#ifndef _VIDCORE_H
#define _VIDCORE_H

#include "base.h"

#define VPU_MBOX_REQUEST_CODE         0x0
#define VPU_MBOX_RESPONSE_CODE        0x80000000
#define VPU_MBOX_PARSING_ERROR_CODE   0x80000001

/*
 * https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 */
#define VPU_MBOX_TAG_SETPOWER         0x28001
#define VPU_MBOX_TAG_SETCLKRATE       0x38002
#define VPU_MBOX_TAG_SETPHYWH         0x48003
#define VPU_MBOX_TAG_SETVIRTWH        0x48004
#define VPU_MBOX_TAG_SETVIRTOFF       0x48009
#define VPU_MBOX_TAG_SETDEPTH         0x48005
#define VPU_MBOX_TAG_SETPXLORDR       0x48006
#define VPU_MBOX_TAG_GETFB            0x40001
#define VPU_MBOX_TAG_GETPITCH         0x40008
#define VPU_MBOX_TAG_GET_ARM_MEM_SIZE 0x00010005
#define VPU_MBOX_TAG_LAST             0x0
#define VPU_MBOX_CH_POWER             0
#define VPU_MBOX_CH_FB                1
#define VPU_MBOX_CH_VUART             2
#define VPU_MBOX_CH_VCHIQ             3
#define VPU_MBOX_CH_LEDS              4
#define VPU_MBOX_CH_BTNS              5
#define VPU_MBOX_CH_TOUCH             6
#define VPU_MBOX_CH_COUNT             7
#define VPU_MBOX_CH_PROP              8 // Channel 8: Request from ARM for response by VC

#define VPU_DEV_ID_USB_HCD            3

#define VPU_CMD_BUFFER_SIZE           64


#define NUMARGS32(...)                (sizeof ((u32[]){ __VA_ARGS__ }) / sizeof (u32))
/*
 * \brief Creates a u32 `cmd_buffer` array on the stack, makes it the correct
 * size, and adds the appropriate starting and ending tags
 *
 * Tag format:
 *  cmd_buffer[i] = size of the entire buffer
 *  cmd_buffer[i+1] = 4 byte tag id
 *  cmd_buffer[i+2] = request length  (4 bytes, 8 bytes....)
 *  cmd_buffer[i+3] = response length (4 bytes, 8 bytes....)
 *  cmd_buffer[i+4] = 1st 4 bytes of request data
 *  cmd_buffer[i+n] = next 4 bytes of request data.....
 *  cmd_buffer[i+n+1] = 4 byte ending tag
 */
#define VPU_CMB_BUFF_INIT(...)                                                            \
    u32 cmd_buffer[VPU_CMD_BUFFER_SIZE] = { (NUMARGS32 (__VA_ARGS__) + 3) * sizeof (u32), \
                                            VPU_MBOX_REQUEST_CODE, __VA_ARGS__, VPU_MBOX_TAG_LAST }

typedef enum vpu_status_t {
    eVPU_STATUS_FAILED = 0,
    eVPU_STATUS_OK     = 1
} vpu_status_t;

vpu_status_t vpu_call (u32 (*buffer_ptr)[VPU_CMD_BUFFER_SIZE], u8 channel);

// extern u32 volatile vid_core_buffer[VID_CORE_BUFFER_SIZE];

#endif /*_VIDCORE_H */