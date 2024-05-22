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

#define VPU_CMD_BUFFER_SIZE           36

s32 vpu_call (u32 (*buffer_ptr)[VPU_CMD_BUFFER_SIZE], u8 channel);

// extern u32 volatile vid_core_buffer[VID_CORE_BUFFER_SIZE];

#endif /*_VIDCORE_H */