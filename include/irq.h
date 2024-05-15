#ifndef _P_IRQ_H
#define _P_IRQ_H

#include "base.h"

// Raspberry Pi 4b
// Manuals:
// BCM2711 ARM Peripherals
// CoreLink GIC-400 Generic Interrupt Controller Technical Reference Manual
// Arm Generic Interrupt Controller Architecture Specification GIC architecture
// Example: Linux Driver
// https://github.com/torvalds/linux/blob/v6.9-rc3/drivers/irqchip/irq-gic-v3.c
// https://github.com/torvalds/linux/blob/v6.9-rc3/include/linux/irqchip/arm-gic-v3.h

/*
 * https://github.com/raspberrypi/linux/blob/rpi-6.6.y/arch/arm/boot/dts/broadcom/bcm2711.dtsi
 */

#define GIC_BASE (0xFF840000)

/************************* Distributor Registers *********************************/
#define GICD_DIST_BASE (GIC_BASE + 0x00001000)

#define GICD_CTLR \
    (GICD_DIST_BASE + 0x000)      // RW 0x00000000c Distributor Control Register
#define GICD_CTLR_DISABLE 0x0     /* Value to disable distributor */
#define GICD_CTLR_ENABLE (1 << 0) /* Value to enable distributor */

#define GICD_TYPER \
    (GICD_DIST_BASE + 0x004) // RO Configuration-dependentd Interrupt Controller Type Register
#define GICD_IIDR \
    (GICD_DIST_BASE + 0x008) // RO 0x0200143B Distributor Implementer Identification Register

#define GICD_IGROUPR0 \
    (GICD_DIST_BASE + 0x080) // GICD_IGROUPRn RW 0x00000000 Interrupt Group Registerse
#define GICD_IGROUPR1 \
    (GICD_DIST_BASE + 0x0BC) // GICD_IGROUPRn RW 0x00000000 Interrupt Group Registerse

#define GICD_ISENABLER0 \
    (GICD_DIST_BASE + 0x100) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER1 \
    (GICD_DIST_BASE + 0x104) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER2 \
    (GICD_DIST_BASE + 0x108) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER3 \
    (GICD_DIST_BASE + 0x10C) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER4 \
    (GICD_DIST_BASE + 0x110) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER5 \
    (GICD_DIST_BASE + 0x114) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER6 \
    (GICD_DIST_BASE + 0x118) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER7 \
    (GICD_DIST_BASE + 0x11C) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER8 \
    (GICD_DIST_BASE + 0x120) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER9 \
    (GICD_DIST_BASE + 0x124) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER10 \
    (GICD_DIST_BASE + 0x128) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER11 \
    (GICD_DIST_BASE + 0x12C) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER12 \
    (GICD_DIST_BASE + 0x130) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER13 \
    (GICD_DIST_BASE + 0x134) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER14 \
    (GICD_DIST_BASE + 0x138) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER15 \
    (GICD_DIST_BASE + 0x13C) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLERn(interrupt_reg_id) \
    (GICD_ISENABLER0 + interrupt_reg_id * sizeof (u32))

#define GICD_ICENABLER0 \
    (GICD_DIST_BASE + 0x180) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER1 \
    (GICD_DIST_BASE + 0x184) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER2 \
    (GICD_DIST_BASE + 0x188) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER3 \
    (GICD_DIST_BASE + 0x18C) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER4 \
    (GICD_DIST_BASE + 0x190) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER5 \
    (GICD_DIST_BASE + 0x194) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER6 \
    (GICD_DIST_BASE + 0x198) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER7 \
    (GICD_DIST_BASE + 0x19C) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER8 \
    (GICD_DIST_BASE + 0x1A0) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER9 \
    (GICD_DIST_BASE + 0x1A4) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER10 \
    (GICD_DIST_BASE + 0x1A8) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER11 \
    (GICD_DIST_BASE + 0x1AC) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER12 \
    (GICD_DIST_BASE + 0x1B0) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER13 \
    (GICD_DIST_BASE + 0x1B4) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER14 \
    (GICD_DIST_BASE + 0x1B8) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER15 \
    (GICD_DIST_BASE + 0x1BC) // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ISENABLER15 \
    (GICD_DIST_BASE + 0x13C) // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ICENABLERn(interrupt_reg_id) \
    (GICD_ICENABLER0 + interrupt_reg_id * sizeof (u32))

// Adds the pending state to interrupt number 32n + x. Reads and writes have the following behavior:
#define GICD_ISPENDR0 \
    (GICD_DIST_BASE + 0x200) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR1 \
    (GICD_DIST_BASE + 0x204) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR2 \
    (GICD_DIST_BASE + 0x208) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR3 \
    (GICD_DIST_BASE + 0x20C) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR4 \
    (GICD_DIST_BASE + 0x210) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR5 \
    (GICD_DIST_BASE + 0x214) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR6 \
    (GICD_DIST_BASE + 0x218) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR7 \
    (GICD_DIST_BASE + 0x21C) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR8 \
    (GICD_DIST_BASE + 0x220) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR9 \
    (GICD_DIST_BASE + 0x224) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR10 \
    (GICD_DIST_BASE + 0x228) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR11 \
    (GICD_DIST_BASE + 0x22C) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR12 \
    (GICD_DIST_BASE + 0x230) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR13 \
    (GICD_DIST_BASE + 0x234) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR14 \
    (GICD_DIST_BASE + 0x238) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDR15 \
    (GICD_DIST_BASE + 0x23C) // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers
#define GICD_ISPENDRn(interrupt_reg_id) \
    (GICD_ISPENDR0 + interrupt_reg_id * sizeof (u32))

// For SPIs and PPIs, removes the pending state from interrupt number 32n + x
#define GICD_ICPENDR0 \
    (GICD_DIST_BASE + 0x280) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR1 \
    (GICD_DIST_BASE + 0x284) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR2 \
    (GICD_DIST_BASE + 0x288) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR3 \
    (GICD_DIST_BASE + 0x28C) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR4 \
    (GICD_DIST_BASE + 0x290) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR5 \
    (GICD_DIST_BASE + 0x294) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR6 \
    (GICD_DIST_BASE + 0x298) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR7 \
    (GICD_DIST_BASE + 0x29C) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR8 \
    (GICD_DIST_BASE + 0x2A0) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR9 \
    (GICD_DIST_BASE + 0x2A4) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR10 \
    (GICD_DIST_BASE + 0x2A8) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR11 \
    (GICD_DIST_BASE + 0x2AC) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR12 \
    (GICD_DIST_BASE + 0x2B0) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR13 \
    (GICD_DIST_BASE + 0x2B4) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR14 \
    (GICD_DIST_BASE + 0x2B8) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDR15 \
    (GICD_DIST_BASE + 0x2BC) // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers
#define GICD_ICPENDRn(interrupt_reg_id) \
    (GICD_ICPENDR0 + interrupt_reg_id * sizeof (u32))

// Adds the active state to interrupt number 32n + x
#define GICD_ISACTIVER0 \
    (GICD_DIST_BASE + 0x300) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER1 \
    (GICD_DIST_BASE + 0x304) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER2 \
    (GICD_DIST_BASE + 0x308) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER3 \
    (GICD_DIST_BASE + 0x30C) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER4 \
    (GICD_DIST_BASE + 0x310) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER5 \
    (GICD_DIST_BASE + 0x314) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER6 \
    (GICD_DIST_BASE + 0x318) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER7 \
    (GICD_DIST_BASE + 0x31C) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER8 \
    (GICD_DIST_BASE + 0x320) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER9 \
    (GICD_DIST_BASE + 0x324) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER10 \
    (GICD_DIST_BASE + 0x328) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER11 \
    (GICD_DIST_BASE + 0x32C) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER12 \
    (GICD_DIST_BASE + 0x330) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER13 \
    (GICD_DIST_BASE + 0x334) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER14 \
    (GICD_DIST_BASE + 0x338) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVER15 \
    (GICD_DIST_BASE + 0x33C) // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers
#define GICD_ISACTIVERn(interrupt_reg_id) \
    (GICD_ISACTIVER0 + interrupt_reg_id * sizeof (u32))

// Removes the active state from interrupt number 32n + x
#define GICD_ICACTIVER0 \
    (GICD_DIST_BASE + 0x380) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER1 \
    (GICD_DIST_BASE + 0x384) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER2 \
    (GICD_DIST_BASE + 0x388) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER3 \
    (GICD_DIST_BASE + 0x38C) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER4 \
    (GICD_DIST_BASE + 0x390) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER5 \
    (GICD_DIST_BASE + 0x394) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER6 \
    (GICD_DIST_BASE + 0x398) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER7 \
    (GICD_DIST_BASE + 0x39C) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER8 \
    (GICD_DIST_BASE + 0x3A0) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER9 \
    (GICD_DIST_BASE + 0x3A4) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER10 \
    (GICD_DIST_BASE + 0x3A8) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER11 \
    (GICD_DIST_BASE + 0x3AC) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER12 \
    (GICD_DIST_BASE + 0x3B0) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER13 \
    (GICD_DIST_BASE + 0x3B4) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER14 \
    (GICD_DIST_BASE + 0x3B8) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVER15 \
    (GICD_DIST_BASE + 0x3BC) // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers
#define GICD_ICACTIVERn(interrupt_reg_id) \
    (GICD_ICACTIVER0 + interrupt_reg_id * sizeof (u32))

// Holds the priority of the corresponding interrupt
// Offset -> 0x400 to 0x5FC
#define GICD_IPRIORITYR0 (GICD_DIST_BASE + 0x400)
#define GICD_IPRIORITYRn(interrupt_reg_id) \
    (GICD_IPRIORITYR0 + interrupt_reg_id * sizeof (u32)) // Interrupt Priority Registers
/*
 * GICC_PMR set to allow sixteen levels ranging from values 0 to 240 on init.
 * Each level needs to be a multiple of 16. So 0, 16, 32, 48
 */
#define GICD_IPRIORITY_DEFAULT_PRIORITY 0xA0

// When affinity routing is not enabled, holds the list of target PEs for the
// interrupt. That is, it holds the list of CPU interfaces to which the
// Distributor forwards the interrupt if it is asserted and has sufficient
// priority. GICD_ITARGETSR0 to GICD_ITARGETSR7 are read-only. Each field
// returns a value that corresponds only to the PE (Core or Processor) reading
// the register. Read ONLY -- 0x800 to 0x81C
#define GICD_ITARGETSR0 \
    (GICD_DIST_BASE + 0x800) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSR1 \
    (GICD_DIST_BASE + 0x804) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSR2 \
    (GICD_DIST_BASE + 0x808) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSR3 \
    (GICD_DIST_BASE + 0x80C) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSR4 \
    (GICD_DIST_BASE + 0x810) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSR5 \
    (GICD_DIST_BASE + 0x814) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSR6 \
    (GICD_DIST_BASE + 0x818) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSR7 \
    (GICD_DIST_BASE + 0x81C) // GICD_ITARGETSRn  Read ONLY - Interrupt Processor Targets Registers
#define GICD_ITARGETSRn(interrupt_reg_id) \
    (GICD_ITARGETSR0 + interrupt_reg_id * 4)
// READ and WRITE
//#define GICD_ITARGETSRn  (GICD_DIST_BASE + 0x820-0x9FC) // GICD_ITARGETSRn R/W - Interrupt Processor Targets Registers

// Determines whether the corresponding interrupt is edge-triggered or level-sensitive.
#define GICD_ICFGR0 \
    (GICD_DIST_BASE + 0xC00) // GICD_ICFGRn RO SGIs: 0xAAAAAAAA Interrupt Configuration Registers, GICD_ICFGRn
#define GICD_ICFGR1 \
    (GICD_DIST_BASE + 0xC04) // GICD_ICFGRn RO PPIs: 0x55540000 Interrupt Configuration Registers, GICD_ICFGRn
#define GICD_ICFGRn(interrupt_reg_id) (GICD_ICFGR0 + interrupt_reg_id * 4)
#define GICD_ICFGR_LEVEL_TRIGGERED 0b00
#define GICD_ICFGR_EDGE_TRIGGERED 0b10
// #define GICD_ICFGRn 0xC08-0xC7C // GICD_ICFGRn R/W SPIs: 0x55555555 Interrupt Configuration Registers, GICD_ICFGRn

// Enables a processor to access the status of the PPI (Private Peripheral
// Interrupt) inputs on the Distributor A processor can only read the status of
// its own PPI and cannot read the status of PPIs for other processors.
// Non-secure accesses can only read the status of Group 1 interrupts
#define GICD_PPISR \
    (GICD_DIST_BASE + 0xD00) // GICD_PPISR RO 0x00000000 Private Peripheral Interrupt Status Register,

// Enables a processor to access the status of the IRQS inputs on the
// Distributor. Returns the status of the IRQS inputs on the Distributor from
// interrupt number 32n + x 0 IRQS is LOW 1 IRQS is HIGH
#define GICD_SPISR0 \
    (GICD_DIST_BASE + 0xD04) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR1 \
    (GICD_DIST_BASE + 0xD08) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR2 \
    (GICD_DIST_BASE + 0xD0C) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR3 \
    (GICD_DIST_BASE + 0xD10) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR4 \
    (GICD_DIST_BASE + 0xD14) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR5 \
    (GICD_DIST_BASE + 0xD18) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR6 \
    (GICD_DIST_BASE + 0xD1C) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR7 \
    (GICD_DIST_BASE + 0xD20) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR8 \
    (GICD_DIST_BASE + 0xD24) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR9 \
    (GICD_DIST_BASE + 0xD28) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR10 \
    (GICD_DIST_BASE + 0xD2C) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR11 \
    (GICD_DIST_BASE + 0xD30) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR12 \
    (GICD_DIST_BASE + 0xD34) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR13 \
    (GICD_DIST_BASE + 0xD38) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers
#define GICD_SPISR14 \
    (GICD_DIST_BASE + 0xD3C) // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers

// Controls the generation of SGIs
// 0b00 Forward the interrupt to the CPU interfaces specified by GICD_SGIR.CPUTargetList.
// 0b01 Forward the interrupt to all CPU interfaces except that of the PE that requested the interrupt.
// 0b10 Forward the interrupt only to the CPU interface of the PE that requested the interrupt.
// 0b11 Reserved.
#define GICD_SGIR \
    (GICD_DIST_BASE + 0xF00) // GICD_SGIR WO - Software Generated Interrupt Register
// #define GICD_CPENDSGIRn 0xF10-0xF1C // GICD_CPENDSGIRn RW 0x00000000 SGI Clear-Pending Registers
// #define GICD_SPENDSGIRn 0xF20-0xF2C // GICD_SPENDSGIRn RW 0x00000000 SGI Set-Pending Registers

#define GICD_PIDR4 \
    (GICD_DIST_BASE + 0xFD0) // GICD_PIDR4 RO 0x00000004 Peripheral ID 4 Register
#define GICD_PIDR5 \
    (GICD_DIST_BASE + 0xFD4) // GICD_PIDR5 RO 0x00000000 Peripheral ID 5 Register
#define GICD_PIDR6 \
    (GICD_DIST_BASE + 0xFD8) // GICD_PIDR6 RO 0x00000000 Peripheral ID 6 Register
#define GICD_PIDR7 \
    (GICD_DIST_BASE + 0xFDC) // RO 0x00000000 Peripheral ID 7 Register
#define GICD_PIDR0 \
    (GICD_DIST_BASE + 0xFE0) // RO 0x00000090 Peripheral ID 0 Register
#define GICD_PIDR1 \
    (GICD_DIST_BASE + 0xFE4) // RO 0x000000B4 Peripheral ID 1 Register
#define GICD_PIDR2 \
    (GICD_DIST_BASE + 0xFE8) // RO 0x0000002B Peripheral ID 2 Register
#define GICD_PIDR3 \
    (GICD_DIST_BASE + 0xFEC) // RO 0x00000000 Peripheral ID 3 Register
#define GICD_CIDR0 \
    (GICD_DIST_BASE + 0xFF0) // RO 0x0000000D Component ID 0 Register
#define GICD_CIDR1 \
    (GICD_DIST_BASE + 0xFF4) // RO 0x000000F0 Component ID 1 Register
#define GICD_CIDR2 \
    (GICD_DIST_BASE + 0xFF8) // RO 0x00000005 Component ID 2 Register
#define GICD_CIDR3 \
    (GICD_DIST_BASE + 0xFFC) // RO 0x000000B1 Component ID 3 Register

/***************************************** CPU Interface ***************************************************/

#define GICC_CPU_BASE (GIC_BASE + 0x00002000)

// Controls the CPU interface, including enabling of interrupt groups, interrupt signal bypass, binary
// point registers used, and separation of priority drop and interrupt deactivation.
#define GICC_CTLR \
    (GICC_CPU_BASE + 0x0000) //  RW 0x00000000 CPU Interface Control Register
#define GICC_CTLR_DISABLE (0 << 0)
#define GICC_CTLR_ENABLE (1 << 0)

// This register provides an interrupt priority filter. Only interrupts with a
// higher priority than the value in this register are signaled to the PE
#define GICC_PMR \
    (GICC_CPU_BASE + 0x0004) // RW 0x00000000 Interrupt Priority Mask Register
#define GICC_PMR_16_PRIORITY_LEVELS \
    0xF0 /* 0x00 - 0xF0 (0-240), in steps of 16	with 16 priority levels */

// Defines the point at which the priority value fields split into two parts,
// the group priority field and the subpriority field.
#define GICC_BPR \
    (GICC_CPU_BASE + 0x0008) // RW 0x00000002b Binary Point Register

// Provides the INTID of the signaled interrupt. A read of this register by the PE acts as an
// acknowledge for the interrupt.
// INTID, bits [23:0] The INTID of the signaled interrupt.
/*
 * When affinity routing is not enabled:
 *   Bits [23:13] are RES0.
 *   For SGIs, bits [12:10] identify the CPU interface corresponding to the
 * source PE. For all other interrupts these bits are RES0.
 */
#define GICC_IAR \
    (GICC_CPU_BASE + 0x000C) //  RO 0x000003FF Interrupt Acknowledge Register

// A write to this register performs priority drop for the specified interrupt
// For EVERY read of a valid INTID from GICC_IAR, the connected PE MUST perform a matching write to GICC_EOIR.
// The value written to GICC_EOIR must be the INTID from GICC_IAR
#define GICC_EOIR (GICC_CPU_BASE + 0x0010) //  WO - End of Interrupt Register

#define GICC_RPR \
    (GICC_CPU_BASE + 0x0014) //  RO 0x000000FF Running Priority Register
#define GICC_HPPIR \
    (GICC_CPU_BASE + 0x0018) //  RO 0x000003FF Highest Priority Pending Interrupt Register c
#define GICC_ABPR \
    (GICC_CPU_BASE + 0x001C) //  RW 0x00000003 Aliased Binary Point Registerd
#define GICC_AIAR \
    (GICC_CPU_BASE + 0x0020) //   RO 0x000003FF Aliased Interrupt Acknowledge Registerd
#define GICC_AEOIR \
    (GICC_CPU_BASE + 0x0024) //    WO - Aliased End of Interrupt Registerd
#define GICC_AHPPIR \
    (GICC_CPU_BASE + 0x0028) //    RO 0x000003FF Aliased Highest Priority Pending Interrupt Registercd
#define GICC_APR0 \
    (GICC_CPU_BASE + 0x00D0) //   RW 0x00000000 Active Priority Register
#define GICC_NSAPR0 \
    (GICC_CPU_BASE + 0x00E0) //     RW 0x00000000 Non-Secure Active Priority Registerd
#define GICC_IIDR \
    (GICC_CPU_BASE + 0x00FC) //   RO 0x0202143B CPU Interface Identification Register, GICC_IIDR on page 3-11
#define GICC_DIR \
    (GICC_CPU_BASE + 0x1000) //   WO - Deactivate Interrupt Register

/*
 * Private Peripheral Interrupts
 *
 * https://github.com/raspberrypi/linux/blob/rpi-6.6.y/arch/arm/boot/dts/broadcom/bcm2711.dtsi
 */
#define GIC_NUM_PPIs 32
#define GIC_PPI_END 31

/*
 * ARM Per Core (PPIs) Interrupt IDs
 * 6 interrupts per core. Range from 7 to 31
 */
#define ARM_CORE_HP_TIMER_IRQ(core_id) ((core_id * 4) + 0 + 7)
#define ARM_CORE_V_TIMER_IRQ(core_id) ((core_id * 4) + 1 + 7)
#define ARM_CORE_LEGACY_FIQ(core_id) ((core_id * 4) + 2 + 7)
#define ARM_CORE_PS_TIMER_IRQ(core_id) ((core_id * 4) + 3 + 7)
#define ARM_CORE_PNS_TIMER_IRQ(core_id) ((core_id * 4) + 4 + 7)
#define ARM_CORE_LEGACY_IRQ(core_id) ((core_id * 4) + 5 + 7)
#define ARM_CORE_IRQID_END 31

// ARM Local Interrupts
// ARM Local GIC IRQ Ids: 32 through 47 - (Page 89 and 90 of BCM2711 ARM Peripherals)
#define ARM_LOCAL_IRQ_BASE (32)
#define ARM_LOCAL_MBOX_IRQ0 (ARM_LOCAL_IRQ_BASE + 0)
#define ARM_LOCAL_MBOX_IRQ1 (ARM_LOCAL_IRQ_BASE + 1)
#define ARM_LOCAL_MBOX_IRQ2 (ARM_LOCAL_IRQ_BASE + 2)
#define ARM_LOCAL_MBOX_IRQ3 (ARM_LOCAL_IRQ_BASE + 3)
#define ARM_LOCAL_MBOX_IRQ4 (ARM_LOCAL_IRQ_BASE + 4)
#define ARM_LOCAL_MBOX_IRQ5 (ARM_LOCAL_IRQ_BASE + 5)
#define ARM_LOCAL_MBOX_IRQ6 (ARM_LOCAL_IRQ_BASE + 6)
#define ARM_LOCAL_MBOX_IRQ7 (ARM_LOCAL_IRQ_BASE + 7)
#define ARM_LOCAL_MBOX_IRQ8 (ARM_LOCAL_IRQ_BASE + 8)
#define ARM_LOCAL_MBOX_IRQ9 (ARM_LOCAL_IRQ_BASE + 9)
#define ARM_LOCAL_MBOX_IRQ10 (ARM_LOCAL_IRQ_BASE + 10)
#define ARM_LOCAL_MBOX_IRQ11 (ARM_LOCAL_IRQ_BASE + 11)
#define ARM_LOCAL_MBOX_IRQ12 (ARM_LOCAL_IRQ_BASE + 12)
#define ARM_LOCAL_MBOX_IRQ13 (ARM_LOCAL_IRQ_BASE + 13)
#define ARM_LOCAL_MBOX_IRQ14 (ARM_LOCAL_IRQ_BASE + 14)
#define ARM_LOCAL_MBOX_IRQ15 (ARM_LOCAL_IRQ_BASE + 15)

// ARMC Interrupts
// ARMC GIC IRQ Ids: 64 through 79 - (Page 87 and 90 of BCM2711 ARM Peripherals)
#define ARMC_IRQ_BASE (64)
#define ARMC_TIMER_IRQ (AMRC_IRQ_BASE + 0)
#define ARMC_MAILBOX_IRQ (AMRC_IRQ_BASE + 1)
#define ARMC_DOORBELL_IRQ_0 (AMRC_IRQ_BASE + 2)
#define ARMC_DOORBELL_IRQ_1 (AMRC_IRQ_BASE + 3)
#define ARMC_VPU_IRQ_0 (AMRC_IRQ_BASE + 4)
#define ARMC_VPU_IRQ_1 (AMRC_IRQ_BASE + 5)
#define ARMC_ADDRESS_ERROR_IRQ (AMRC_IRQ_BASE + 6)
#define ARMC_AXI_IRQ (AMRC_IRQ_BASE + 7)
#define ARMC_SOFTWARE_IRQ_0 (AMRC_IRQ_BASE + 8)
#define ARMC_SOFTWARE_IRQ_1 (AMRC_IRQ_BASE + 9)
#define ARMC_SOFTWARE_IRQ_2 (AMRC_IRQ_BASE + 10)
#define ARMC_SOFTWARE_IRQ_3 (AMRC_IRQ_BASE + 11)
#define ARMC_SOFTWARE_IRQ_4 (AMRC_IRQ_BASE + 12)
#define ARMC_SOFTWARE_IRQ_5 (AMRC_IRQ_BASE + 13)
#define ARMC_SOFTWARE_IRQ_6 (AMRC_IRQ_BASE + 14)
#define ARMC_SOFTWARE_IRQ_7 (AMRC_IRQ_BASE + 15)


// VC (VideoCore) GIC IRQ Ids: 96 through 159 - (Page 88 and 90 of BCM2711 ARM Peripherals)
#define VC_GIC_BASE_IRQ (96)
#define VC_GIC_SYSTEM_TIMER_IRQ_0 (VC_GIC_BASE_IRQ + 0) // 96
#define VC_GIC_SYSTEM_TIMER_IRQ_1 (VC_GIC_BASE_IRQ + 1) // 97
#define VC_GIC_SYSTEM_TIMER_IRQ_2 (VC_GIC_BASE_IRQ + 2) // 98
#define VC_GIC_SYSTEM_TIMER_IRQ_3 (VC_GIC_BASE_IRQ + 3) // 99

#define VC_GIC_DMA_IRQ_0 (VC_GIC_BASE_IRQ + 16)
#define VC_GIC_DMA_IRQ_1 (VC_GIC_BASE_IRQ + 17)
#define VC_GIC_DMA_IRQ_2 (VC_GIC_BASE_IRQ + 18)
#define VC_GIC_DMA_IRQ_3 (VC_GIC_BASE_IRQ + 19)
#define VC_GIC_DMA_IRQ_4 (VC_GIC_BASE_IRQ + 20)
#define VC_GIC_DMA_IRQ_5 (VC_GIC_BASE_IRQ + 21)
#define VC_GIC_DMA_IRQ_6 (VC_GIC_BASE_IRQ + 22)
#define VC_GIC_DMA_IRQ_7n8 (VC_GIC_BASE_IRQ + 23)
#define VC_GIC_DMA_IRQ_9n10 (VC_GIC_BASE_IRQ + 24)
#define VC_GIC_DMA_IRQ_11 (VC_GIC_BASE_IRQ + 26)
#define VC_GIC_DMA_IRQ_12 (VC_GIC_BASE_IRQ + 26)
#define VC_GIC_DMA_IRQ_13 (VC_GIC_BASE_IRQ + 27)
#define VC_GIC_DMA_IRQ_14 (VC_GIC_BASE_IRQ + 28)
#define VC_GIC_DMA_IRQ_START VC_GIC_DMA_IRQ_0
#define VC_GIC_DMA_IRQ_END VC_GIC_DMA_IRQ_14

#define GIC_NUM_INTERRUPTS \
    (16 * 32) /* 16 ISEnable Registers and each register is 32 bits */

/********************** ARM Interrupts ***************************/
#define IRQ_BIT (1 << 7)
#define FIQ_BIT (1 << 6)
#define ENABLE_IRQ() asm volatile("msr daifclr, #2")
#define DISABLE_IRQ() asm volatile("msr daifset, #2")
#define ENABLE_FIQ() asm volatile("msr daifclr, #1")
#define DISABLE_FIQ() asm volatile("msr daifset, #1")
#define ENABLE_IRQ_FIQ() asm volatile("msr daifclr, #3")
#define DISABLE_IRQ_FIQ() asm volatile("msr daifset, #3")
#define MAX_NESTED_INTERRUPTS 12

#ifndef __ASSEMBLER__

// extern irq_handler_t gic_irq_handlers[ARM_NUM_CORES][GIC_NUM_INTERRUPTS];

u64 get_daif_flags (void);
void set_daif_flags (u64);

u32 gic_get_cpu_id (void);
void gic_assign_irq_handler (u32 irq_id, irq_handler_t handler);
void gic_enable_interrupt (u32 irq, irq_handler_t handler);
void gic_assign_target (u32 irq_id, u32 gic_cpu_id);
void gic_general_irq_handler (u32 irq_id);
s32 gic_init (void);

s32 irq_init (void);


void handle_irq (void);

#endif // __ASSEMBLER__

#endif /*_P_IRQ_H */