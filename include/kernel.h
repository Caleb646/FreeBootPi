#ifndef _KERNEL_H
#define _KERNEL_H

#include "base.h"

#define S_FRAME_SIZE           256 // size of all saved registers

#define SYNC_INVALID_EL1t      0
#define IRQ_INVALID_EL1t       1
#define FIQ_INVALID_EL1t       2
#define ERROR_INVALID_EL1t     3

#define SYNC_INVALID_EL1h      4
#define IRQ_INVALID_EL1h       5
#define FIQ_INVALID_EL1h       6
#define ERROR_INVALID_EL1h     7

#define SYNC_INVALID_EL0_64    8
#define IRQ_INVALID_EL0_64     9
#define FIQ_INVALID_EL0_64     10
#define ERROR_INVALID_EL0_64   11

#define SYNC_INVALID_EL0_32    12
#define IRQ_INVALID_EL0_32     13
#define FIQ_INVALID_EL0_32     14
#define ERROR_INVALID_EL0_32   15

// #define PAGE_SHIFT 12
// #define TABLE_SHIFT 9
// #define SECTION_SHIFT (PAGE_SHIFT + TABLE_SHIFT)
// #define PAGE_SIZE (1 << PAGE_SHIFT)
// #define SECTION_SIZE (1 << SECTION_SHIFT)

/********************** ARM Multicore ***************************/
/* https://github.com/raspberrypi/tools/blob/master/armstubs/armstub8.S */
#define ARM_SEC_CORE_SPIN_BASE 0xD8

#ifndef __ASSEMBLER__


#endif // __ASSEMBLER__

#endif // _KERNEL_H