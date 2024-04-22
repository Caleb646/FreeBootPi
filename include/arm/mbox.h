#ifndef	_ARM_MBOX_H
#define	_ARM_MBOX_H

#include "base.h"

// If Low Peripheral mode is enabled
#define ARM_MBOX_BASE (0xFF800000)
/*
* Writing a '1' to a bit position in this register causes the corresponding bit in the mailbox word to be set to 1.
* There are 16 mailboxes in total, four per ARM core. Mailboxes 4C to 4C+3 'belong' to core number C.
* Each mailbox may raise an interrupt to its core when any bits in the 32-bit word are set to '1'
*/
#define ARM_MBOX_SET00 (ARM_MBOX_BASE + 0x80) // 00 Set Bit Register
#define ARM_MBOX_SET01 (ARM_MBOX_BASE + 0x84) // 01 Set Bit Register
#define ARM_MBOX_SET02 (ARM_MBOX_BASE + 0x88) // 02 Set Bit Register
#define ARM_MBOX_SET03 (ARM_MBOX_BASE + 0x8c) // 03 Set Bit Register
#define ARM_MBOX_SET04 (ARM_MBOX_BASE + 0x90) // 04 Set Bit Register
#define ARM_MBOX_SET05 (ARM_MBOX_BASE + 0x94) // 05 Set Bit Register
#define ARM_MBOX_SET06 (ARM_MBOX_BASE + 0x98) // 06 Set Bit Register
#define ARM_MBOX_SET07 (ARM_MBOX_BASE + 0x9c) // 07 Set Bit Register
#define ARM_MBOX_SET08 (ARM_MBOX_BASE + 0xa0) // 08 Set Bit Register
#define ARM_MBOX_SET09 (ARM_MBOX_BASE + 0xa4) // 09 Set Bit Register
#define ARM_MBOX_SET10 (ARM_MBOX_BASE + 0xa8) // 10 Set Bit Register
#define ARM_MBOX_SET11 (ARM_MBOX_BASE + 0xac) // 11 Set Bit Register
#define ARM_MBOX_SET12 (ARM_MBOX_BASE + 0xb0) // 12 Set Bit Register
#define ARM_MBOX_SET13 (ARM_MBOX_BASE + 0xb4) // 13 Set Bit Register
#define ARM_MBOX_SET14 (ARM_MBOX_BASE + 0xb8) // 14 Set Bit Register
#define ARM_MBOX_SET15 (ARM_MBOX_BASE + 0xbc) // 15 Set Bit Register
/*
* Writing a '1' to a bit position in this register causes the corresponding bit in the mailbox word to be cleared to 0. 
* A read returns the current state of the mailbox word.
* There are 16 mailboxes in total, four per ARM core. Mailboxes 4C to 4C+3 'belong' to core number C.
* Each mailbox may raise an interrupt to its core when any bits in the 32-bit word are set to '1'
*/
#define ARM_MBOX_CLR00 (ARM_MBOX_BASE + 0xc0) // 00 Clear Bit Register
#define ARM_MBOX_CLR01 (ARM_MBOX_BASE + 0xc4) // 01 Clear Bit Register
#define ARM_MBOX_CLR02 (ARM_MBOX_BASE + 0xc8) // 02 Clear Bit Register
#define ARM_MBOX_CLR03 (ARM_MBOX_BASE + 0xcc) // 03 Clear Bit Register
#define ARM_MBOX_CLR04 (ARM_MBOX_BASE + 0xd0) // 04 Clear Bit Register
#define ARM_MBOX_CLR05 (ARM_MBOX_BASE + 0xd4) // 05 Clear Bit Register
#define ARM_MBOX_CLR06 (ARM_MBOX_BASE + 0xd8) // 06 Clear Bit Register
#define ARM_MBOX_CLR07 (ARM_MBOX_BASE + 0xdc) // 07 Clear Bit Register
#define ARM_MBOX_CLR08 (ARM_MBOX_BASE + 0xe0) // 08 Clear Bit Register
#define ARM_MBOX_CLR09 (ARM_MBOX_BASE + 0xe4) // 09 Clear Bit Register
#define ARM_MBOX_CLR10 (ARM_MBOX_BASE + 0xe8) // 10 Clear Bit Register
#define ARM_MBOX_CLR11 (ARM_MBOX_BASE + 0xec) // 11 Clear Bit Register
#define ARM_MBOX_CLR12 (ARM_MBOX_BASE + 0xf0) // 12 Clear Bit Register
#define ARM_MBOX_CLR13 (ARM_MBOX_BASE + 0xf4) // 13 Clear Bit Register
#define ARM_MBOX_CLR14 (ARM_MBOX_BASE + 0xf8) // 14 Clear Bit Register
#define ARM_MBOX_CLR15 (ARM_MBOX_BASE + 0xfc) // 15 Clear Bit Register

#endif 