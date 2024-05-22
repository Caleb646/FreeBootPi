#ifndef _SYSREGS_H
#define _SYSREGS_H

// ***************************************
// SCTLR_EL1, System Control Register (EL1), Page 2654 of AArch64-Reference-Manual.
// ***************************************
// Some bits in the description of sctlr_el1 register are marked as RES1.
// Those bits are reserved for future usage and should be initialized with 1.
// [31]	-	Reserved.
// [30]	TE	Thumb Exception enable.
// [29]	AFE	Access flag enable bit.
// [28]	TRE	TEX remap enable bit.
// [27:26]	-	Reserved.
// [25]	EE	Exception Endianness bit.
// [24:21]	-	Reserved.
// [20]	UWXN	Unprivileged write permission implies PL1 Execute Never (XN).
// [19]	WXN	Write permission implies Execute Never (XN).
// [18:14]	-	Reserved.
// [13]	V	Vectors bit.
// [12]	I	Instruction cache enable bit.
// [11]	Z	Branch prediction enable bit.
// [10]	SW	SWP and SWPB enable bit.
// [9:3]	-	Reserved.
// [2]	C	Cache enable bit.
// [1]	A	Alignment bit.
// [0]	M	Address translation enable bit.
#define SCTLR_RESERVED \
    (1 << 31) | (3 << 26) | (0b1111 << 21) | (0b1111 << 14) | (0b1111111 << 3)
// Controls endianess of explicit data access at EL1.
#define SCTLR_EE_BIG_ENDIAN              (1 << 25)
// Controls endianess of explicit data access at EL0
#define SCTLR_EOE_BIG_ENDIAN             (1 << 24)
#define SCTLR_WXN                        (1 << 19)
// Enable instruction cache
#define SCTLR_I_CACHE_ENABLED            (1 << 12)
// SP Alignment check enable for EL0. When set to 1, if a load or store instruction
// executed at EL0 uses the SP as the base address and the SP is not
// aligned to a 16-byte boundary, then an SP alignment fault exception is generated
#define SCTLR_EL1_SP_ALIGNMENT_CHECK_EL0 (1 << 4)
// SP Alignment check enable for EL1. When set to 1, if a load or store instruction
// executed at EL1 uses the SP as the base address and the SP is not
// aligned to a 16-byte boundary, then an SP alignment fault exception is generated
#define SCTLR_EL1_SP_ALIGNMENT_CHECK_EL1 (1 << 3)
// Enable data cache
#define SCTLR_D_CACHE_ENABLED            (1 << 2)
/*
 * This is the enable bit for Alignment fault checking at EL1 and EL0.
 *   0b0	-- Alignment fault checking disabled when executing at EL1 or EL0.
 *       Instructions that load or store one or more registers, other than load/store exclusive
 *       and load-acquire/store-release, do not check that the address being accessed is aligned
 *       to the size of the data element(s) being accessed.
 *
 *   0b1	-- Alignment fault checking enabled when executing at EL1 or EL0. All instructions that
 *       load or store one or more registers have an alignment check that the address being accessed
 *       is aligned to the size of the data element(s) being accessed. If this check fails it causes
 *       an Alignment fault, which is taken as a Data Abort exception.
 */
#define SCTLR_ALIGNMENT                  (1 << 1)
// MMU enable for (if using sctlr_el1) EL1&0 stage 1 address translation.
#define SCTLR_MMU_ENABLED                (1 << 0)
#define SCTLR_EL1_VALUE                                                         \
    (                                                                           \
    SCTLR_RESERVED | SCTLR_I_CACHE_ENABLED | SCTLR_EL1_SP_ALIGNMENT_CHECK_EL0 | \
    SCTLR_EL1_SP_ALIGNMENT_CHECK_EL1 | SCTLR_D_CACHE_ENABLED)

// ***************************************
// HCR_EL2, Hypervisor Configuration Register (EL2), Page 2487 of AArch64-Reference-Manual.
// ***************************************
#define HCR_RW         (1 << 31)
// Execution state must be AArch64 and not AArch32
#define HCR_EL2_VALUE  HCR_RW

// ***************************************
// SCR_EL3, Secure Configuration Register (EL3), Page 2648 of AArch64-Reference-Manual.
// ***************************************
#define SCR_RESERVED   (3 << 4)
#define SCR_RW         (1 << 10)
#define SCR_NS         (1 << 0)
// Set that EL2 will execute at AArch64 state, and all lower exception levels will be "non secure".
#define SCR_VALUE      (SCR_RESERVED | SCR_RW | SCR_NS)

// ***************************************
// SPSR_ELn, Saved Program Status Register (EL3) Page 389 of AArch64-Reference-Manual.
// https://developer.arm.com/documentation/ddi0601/2024-03/AArch64-Registers/SPSR-EL2--Saved-Program-Status-Register--EL2-?lang=en
// ***************************************
// After changing EL to EL1 all types of interrupts will be masked (or disabled, which is the same).
#define SPSR_DAIF      (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6)
// At EL1 we can either use our own dedicated stack pointer or use EL0 stack
// pointer. EL1h mode means that we are using EL1 dedicated stack pointer.
#define SPSR_EL1h      0b0101 // EL1 with SP_EL1 (EL1h).
#define SPSR_EL2_VALUE (SPSR_DAIF | SPSR_EL1h)

#endif