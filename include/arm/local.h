#ifndef	_ARM_LOCAL_H
#define	_ARM_LOCAL_H

#include "base.h"

// If Low Peripheral mode is enabled
// Page 93 of BCM2711 ARM Peripherals
#define ARM_LOCAL_BASE                  (0xFF800000)
#define ARM_LOCAL_CONTROL               (ARM_LOCAL_BASE + 0x00) // Timer and AXI Error IRQ control
#define ARM_LOCAL_CORE_IRQ_CONTROL      (ARM_LOCAL_BASE + 0x0c) // VideoCore Interrupt Control
#define ARM_LOCAL_PMU_CONTROL_SET       (ARM_LOCAL_BASE + 0x10) // Performance Monitoring Unit (PMU) Bit Set
#define ARM_LOCAL_PMU_CONTROL_CLR       (ARM_LOCAL_BASE + 0x14) // Performance Monitoring Unit (PMU) Bit Clear
#define ARM_LOCAL_PERI_IRQ_ROUTE0       (ARM_LOCAL_BASE + 0x24) // Peripheral Interrupt Routing (Bank 0)
#define ARM_LOCAL_AXI_QUIET_TIME        (ARM_LOCAL_BASE + 0x30) // AXI Outstanding Transaction Time and IRQ Control
#define ARM_LOCAL_LOCAL_TIMER_CONTROL   (ARM_LOCAL_BASE + 0x34) // Local Timer Control
#define ARM_LOCAL_LOCAL_TIMER_IRQ       (ARM_LOCAL_BASE + 0x38) // Local Timer Reload and Interrupt
#define ARM_LOCAL_TIMER_CNTRL0          (ARM_LOCAL_BASE + 0x40) // Timer Interrupt Control for ARM Core 0
#define ARM_LOCAL_TIMER_CNTRL1          (ARM_LOCAL_BASE + 0x44) // Timer Interrupt Control for ARM Core 1
#define ARM_LOCAL_TIMER_CNTRL2          (ARM_LOCAL_BASE + 0x48) // Timer Interrupt Control for ARM Core 2
#define ARM_LOCAL_TIMER_CNTRL3          (ARM_LOCAL_BASE + 0x4c) // Timer Interrupt Control for ARM Core 3

// Controls the routing of the mailbox interrupts to an ARM core’s IRQ or FIQ interrupt request pins. Each
// ARM can receive interrupts from four of the sixteen mailbox registers. For ARM core 0, these are mailboxes 0-3; for
// ARM core 1, mailboxes 4-7 and so on.
// Bits [3 to 0] --- When set to 1, causes the mailbox 4C+3 for ARM core number C, to trigger an IRQ
// interrupt when any bit is set in the mailbox. 
#define ARM_LOCAL_MAILBOX_CNTRL0        (ARM_LOCAL_BASE + 0x50) // Mailbox Interrupt Control for ARM Core 0
#define ARM_LOCAL_MAILBOX_CNTRL1        (ARM_LOCAL_BASE + 0x54) // Mailbox Interrupt Control for ARM Core 1
#define ARM_LOCAL_MAILBOX_CNTRL2        (ARM_LOCAL_BASE + 0x58) // Mailbox Interrupt Control for ARM Core 2
#define ARM_LOCAL_MAILBOX_CNTRL3        (ARM_LOCAL_BASE + 0x5c) // Mailbox Interrupt Control for ARM Core 3

#define ARM_LOCAL_IRQ_SOURCE0           (ARM_LOCAL_BASE + 0x60) // IRQ Source flags for ARM Core 0
#define ARM_LOCAL_IRQ_SOURCE1           (ARM_LOCAL_BASE + 0x64) // IRQ Source flags for ARM Core 1
#define ARM_LOCAL_IRQ_SOURCE2           (ARM_LOCAL_BASE + 0x68) // IRQ Source flags for ARM Core 2
#define ARM_LOCAL_IRQ_SOURCE3           (ARM_LOCAL_BASE + 0x6c) // IRQ Source flags for ARM Core 3
#define ARM_LOCAL_FIQ_SOURCE0           (ARM_LOCAL_BASE + 0x70) // FIQ Source flags for ARM Core 0
#define ARM_LOCAL_FIQ_SOURCE1           (ARM_LOCAL_BASE + 0x74) // FIQ Source flags for ARM Core 1
#define ARM_LOCAL_FIQ_SOURCE2           (ARM_LOCAL_BASE + 0x78) // FIQ Source flags for ARM Core 2
#define ARM_LOCAL_FIQ_SOURCE3           (ARM_LOCAL_BASE + 0x7c) // FIQ Source flags for ARM Core 3
/*
* Writing a '1' to a bit position in this register causes the corresponding bit in the mailbox word to be set to 1.
* There are 16 mailboxes in total, four per ARM core. Mailboxes 4C to 4C+3 'belong' to core number C.
* Each mailbox may raise an interrupt to its core when any bits in the 32-bit word are set to '1'
*/
#define ARM_LOCAL_MBOX_SET00 (ARM_LOCAL_BASE + 0x80) // 00 Set Bit Register
#define ARM_LOCAL_MBOX_SET01 (ARM_LOCAL_BASE + 0x84) // 01 Set Bit Register
#define ARM_LOCAL_MBOX_SET02 (ARM_LOCAL_BASE + 0x88) // 02 Set Bit Register
#define ARM_LOCAL_MBOX_SET03 (ARM_LOCAL_BASE + 0x8c) // 03 Set Bit Register
#define ARM_LOCAL_MBOX_SET04 (ARM_LOCAL_BASE + 0x90) // 04 Set Bit Register
#define ARM_LOCAL_MBOX_SET05 (ARM_LOCAL_BASE + 0x94) // 05 Set Bit Register
#define ARM_LOCAL_MBOX_SET06 (ARM_LOCAL_BASE + 0x98) // 06 Set Bit Register
#define ARM_LOCAL_MBOX_SET07 (ARM_LOCAL_BASE + 0x9c) // 07 Set Bit Register
#define ARM_LOCAL_MBOX_SET08 (ARM_LOCAL_BASE + 0xa0) // 08 Set Bit Register
#define ARM_LOCAL_MBOX_SET09 (ARM_LOCAL_BASE + 0xa4) // 09 Set Bit Register
#define ARM_LOCAL_MBOX_SET10 (ARM_LOCAL_BASE + 0xa8) // 10 Set Bit Register
#define ARM_LOCAL_MBOX_SET11 (ARM_LOCAL_BASE + 0xac) // 11 Set Bit Register
#define ARM_LOCAL_MBOX_SET12 (ARM_LOCAL_BASE + 0xb0) // 12 Set Bit Register
#define ARM_LOCAL_MBOX_SET13 (ARM_LOCAL_BASE + 0xb4) // 13 Set Bit Register
#define ARM_LOCAL_MBOX_SET14 (ARM_LOCAL_BASE + 0xb8) // 14 Set Bit Register
#define ARM_LOCAL_MBOX_SET15 (ARM_LOCAL_BASE + 0xbc) // 15 Set Bit Register
/*
* Writing a '1' to a bit position in this register causes the corresponding bit in the mailbox word to be cleared to 0. 
* A read returns the current state of the mailbox word.
* There are 16 mailboxes in total, four per ARM core. Mailboxes 4C to 4C+3 'belong' to core number C.
* Each mailbox may raise an interrupt to its core when any bits in the 32-bit word are set to '1'
*/
#define ARM_LOCAL_MBOX_CLR00 (ARM_LOCAL_BASE + 0xc0) // 00 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR01 (ARM_LOCAL_BASE + 0xc4) // 01 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR02 (ARM_LOCAL_BASE + 0xc8) // 02 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR03 (ARM_LOCAL_BASE + 0xcc) // 03 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR04 (ARM_LOCAL_BASE + 0xd0) // 04 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR05 (ARM_LOCAL_BASE + 0xd4) // 05 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR06 (ARM_LOCAL_BASE + 0xd8) // 06 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR07 (ARM_LOCAL_BASE + 0xdc) // 07 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR08 (ARM_LOCAL_BASE + 0xe0) // 08 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR09 (ARM_LOCAL_BASE + 0xe4) // 09 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR10 (ARM_LOCAL_BASE + 0xe8) // 10 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR11 (ARM_LOCAL_BASE + 0xec) // 11 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR12 (ARM_LOCAL_BASE + 0xf0) // 12 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR13 (ARM_LOCAL_BASE + 0xf4) // 13 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR14 (ARM_LOCAL_BASE + 0xf8) // 14 Clear Bit Register
#define ARM_LOCAL_MBOX_CLR15 (ARM_LOCAL_BASE + 0xfc) // 15 Clear Bit Register

#endif 