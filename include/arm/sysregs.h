#ifndef _SYSREGS_H
#define _SYSREGS_H

// ***************************************
// SCTLR_EL1, System Control Register (EL1), Page 2654 of AArch64-Reference-Manual.
// ***************************************

// Some bits in the description of sctlr_el1 register are marked as RES1.
// Those bits are reserved for future usage and should be initialized with 1.
#define SCTLR_RESERVED (3 << 28) | (3 << 22) | (1 << 20) | (1 << 11)
// Controls endianess of explicit data access at EL1.
#define SCTLR_EE_LITTLE_ENDIAN (0 << 25)
// Controls endianess of explicit data access at EL0
#define SCTLR_EOE_LITTLE_ENDIAN (0 << 24)
// Disable instruction cache
#define SCTLR_I_CACHE_DISABLED (0 << 12)
// Disable data cache
#define SCTLR_D_CACHE_DISABLED (0 << 2)
#define SCTLR_MMU_DISABLED (0 << 0)
#define SCTLR_MMU_ENABLED (1 << 0)

#define SCTLR_VALUE_MMU_DISABLED (SCTLR_RESERVED | SCTLR_EE_LITTLE_ENDIAN | SCTLR_I_CACHE_DISABLED | SCTLR_D_CACHE_DISABLED | SCTLR_MMU_DISABLED)

// ***************************************
// HCR_EL2, Hypervisor Configuration Register (EL2), Page 2487 of AArch64-Reference-Manual.
// ***************************************

#define HCR_RW (1 << 31)
// Execution state must be AArch64 and not AArch32
#define HCR_VALUE HCR_RW

// ***************************************
// SCR_EL3, Secure Configuration Register (EL3), Page 2648 of AArch64-Reference-Manual.
// ***************************************

#define SCR_RESERVED (3 << 4)
#define SCR_RW (1 << 10)
#define SCR_NS (1 << 0)
// Set that EL2 will execute at AArch64 state, and all lower exception levels will be "non secure".
#define SCR_VALUE (SCR_RESERVED | SCR_RW | SCR_NS)

// ***************************************
// SPSR_EL3, Saved Program Status Register (EL3) Page 389 of AArch64-Reference-Manual.
// ***************************************

// After changing EL to EL1 all types of interrupts will be masked (or disabled, which is the same).
#define SPSR_MASK_ALL (7 << 6)
// At EL1 we can either use our own dedicated stack pointer or use EL0 stack pointer. 
// EL1h mode means that we are using EL1 dedicated stack pointer.
#define SPSR_EL1h (5 << 0)
#define SPSR_VALUE (SPSR_MASK_ALL | SPSR_EL1h)

#endif