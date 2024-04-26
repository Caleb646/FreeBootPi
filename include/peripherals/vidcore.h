#ifndef	_VIDCORE_H
#define	_VIDCORE_H

#include "base.h"

#define VID_CORE_MBOX_REQUEST_CODE                      0x0
#define VID_CORE_MBOX_RESPONSE_CODE                     0x80000000
#define VID_CORE_MBOX_PARSING_ERROR_CODE                0x80000001

#define VID_CORE_MBOX_TAG_SETPOWER                      0x28001
#define VID_CORE_MBOX_TAG_SETCLKRATE                    0x38002
#define VID_CORE_MBOX_TAG_SETPHYWH                      0x48003
#define VID_CORE_MBOX_TAG_SETVIRTWH                     0x48004
#define VID_CORE_MBOX_TAG_SETVIRTOFF                    0x48009
#define VID_CORE_MBOX_TAG_SETDEPTH                      0x48005
#define VID_CORE_MBOX_TAG_SETPXLORDR                    0x48006
#define VID_CORE_MBOX_TAG_GETFB                         0x40001
#define VID_CORE_MBOX_TAG_GETPITCH                      0x40008
#define VID_CORE_MBOX_TAG_LAST                          0x0

#define VID_CORE_BASE                                   (PBASE + 0x200B880)
#define VID_CORE_MBOX_READ                              (VID_CORE_BASE + 0x0)
#define VID_CORE_MBOX_POLL                              (VID_CORE_BASE + 0x10)
#define VID_CORE_MBOX_SENDER                            (VID_CORE_BASE + 0x14)
#define VID_CORE_MBOX_STATUS                            (VID_CORE_BASE + 0x18)
#define VID_CORE_MBOX_CONFIG                            (VID_CORE_BASE + 0x1C)
#define VID_CORE_MBOX_WRITE                             (VID_CORE_BASE + 0x20)
#define VID_CORE_IS_VALID_MBOX_RESPONSE(response)       (response == VID_CORE_MBOX_RESPONSE_CODE)
#define VID_CORE_IS_MBOX_REPLY(data_sent)               (REG_PTR32(VID_CORE_MBOX_READ) == data_sent)
#define VID_CORE_IS_MBOX_FULL                           (REG_PTR32(VID_CORE_MBOX_STATUS) & VID_CORE_MBOX_RESPONSE_CODE)
#define VID_CORE_IS_MBOX_EMPTY                          (REG_PTR32(VID_CORE_MBOX_STATUS) & 0x40000000)

#define VID_CORE_MBOX_CHANNEL_POWER                     0
#define VID_CORE_MBOX_CHANNEL_FB                        1
#define VID_CORE_MBOX_CHANNEL_VUART                     2
#define VID_CORE_MBOX_CHANNEL_VCHIQ                     3
#define VID_CORE_MBOX_CHANNEL_LEDS                      4
#define VID_CORE_MBOX_CHANNEL_BTNS                      5
#define VID_CORE_MBOX_CHANNEL_TOUCH                     6
#define VID_CORE_MBOX_CHANNEL_COUNT                     7
#define VID_CORE_MBOX_CHANNEL_PROP                      8 // Channel 8: Request from ARM for response by VC

s32 vid_core_call(u32 volatile* buffer, u8 vid_core_channel);

// extern u32 volatile vid_core_buffer[36];
// extern u8 volatile* frame_buffer;

#endif  /*_VIDCORE_H */