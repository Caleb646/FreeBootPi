#ifndef	_P_IRQ_H
#define	_P_IRQ_H

#include "base.h"

// Raspberry Pi 4b
// Manuals:
// BCM2711 ARM Peripherals
// CoreLink GIC-400 Generic Interrupt Controller Technical Reference Manual
// Arm Generic Interrupt Controller Architecture Specification GIC architecture
// Example: Linux Driver
// https://github.com/torvalds/linux/blob/v6.9-rc3/drivers/irqchip/irq-gic-v3.c
// https://github.com/torvalds/linux/blob/v6.9-rc3/include/linux/irqchip/arm-gic-v3.h
#define GIC_BASE (0xFF840000)

/************************* Distributor Registers *********************************/
#define GICD_DIST_BASE (GIC_BASE + 0x00001000)

#define GICD_CTLR  0x000 // RW 0x00000000c Distributor Control Register
#define GICD_TYPER 0x004 // RO Configuration-dependentd Interrupt Controller Type Register
#define GICD_IIDR  0x008 // RO 0x0200143B Distributor Implementer Identification Register

#define GICD_IGROUPR0 0x080 // GICD_IGROUPRn RW 0x00000000 Interrupt Group Registerse
#define GICD_IGROUPR1 0x0BC // GICD_IGROUPRn RW 0x00000000 Interrupt Group Registerse

#define GICD_ISENABLER0 0x100 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER1 0x104 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER2 0x108 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER3 0x10C // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER4 0x110 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER5 0x114 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER6 0x118 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER7 0x11C // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER8 0x120 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER9 0x124 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER10 0x128 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER11 0x12C // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER12 0x130 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER13 0x134 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER14 0x138 // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers
#define GICD_ISENABLER15 0x13C // GICD_ISENABLERn RWf SGIs and PPIs: 0x0000FFFFg Interrupt Set-Enable Registers

#define GICD_ICENABLER0 0x180 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER1 0x184 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER2 0x188 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER3 0x18C // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER4 0x190 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER5 0x194 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER6 0x198 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER7 0x19C // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER8 0x1A0 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER9 0x1A4 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER10 0x1A8 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER11 0x1AC // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER12 0x1B0 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER13 0x1B4 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER14 0x1B8 // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers
#define GICD_ICENABLER15 0x1BC // GICD_ICENABLERn RWf 0x0000FFFFg Interrupt Clear-Enable Registers

#define GICD_ISPENDRn    0x200-0x23C // GICD_ISPENDRn    RW 0x00000000 Interrupt Set-Pending Registers

#define GICD_ICPENDRn    0x280-0x2BC // GICD_ICPENDRn    RW 0x00000000 Interrupt Clear-Pending Registers

#define GICD_ISACTIVERn  0x300-0x33C // GICD_ISACTIVERn  RW 0x00000000 Interrupt Set-Active Registers

#define GICD_ICACTIVERn  0x380-0x3BC // GICD_ICACTIVERn  RW 0x00000000 Interrupt Clear-Active Registers

#define GICD_IPRIORITYRn 0x400-0x5FC // GICD_IPRIORITYRn RW 0x00000000 Interrupt Priority Registers

#define GICD_ITARGETSRn  0x800-0x81C // GICD_ITARGETSRn  ROh - Interrupt Processor Targets Registers
#define GICD_ITARGETSRn  0x820-0x9FC // GICD_ITARGETSRn  ROh - Interrupt Processor Targets Registers

#define GICD_ICFGRn 0xC00 // GICD_ICFGRn RO SGIs: 0xAAAAAAAA Interrupt Configuration Registers, GICD_ICFGRn
#define GICD_ICFGRn 0xC04 // GICD_ICFGRn RO SGIs: 0xAAAAAAAA Interrupt Configuration Registers, GICD_ICFGRn
#define GICD_ICFGRn 0xC08-0xC7C // GICD_ICFGRn RO SGIs: 0xAAAAAAAA Interrupt Configuration Registers, GICD_ICFGRn

#define GICD_PPISR 0xD00 // GICD_PPISR RO 0x00000000 Private Peripheral Interrupt Status Register,
#define GICD_SPISRn 0xD04-0xD3C // GICD_SPISRn RO 0x00000000 Shared Peripheral Interrupt Status Registers,
#define GICD_SGIR 0xF00 // GICD_SGIR WO - Software Generated Interrupt Register
#define GICD_CPENDSGIRn 0xF10-0xF1C // GICD_CPENDSGIRn RW 0x00000000 SGI Clear-Pending Registers
#define GICD_SPENDSGIRn 0xF20-0xF2C // GICD_SPENDSGIRn RW 0x00000000 SGI Set-Pending Registers

#define GICD_PIDR4 0xFD0 // GICD_PIDR4 RO 0x00000004 Peripheral ID 4 Register
#define GICD_PIDR5 0xFD4 // GICD_PIDR5 RO 0x00000000 Peripheral ID 5 Register
#define GICD_PIDR6 0xFD8 // GICD_PIDR6 RO 0x00000000 Peripheral ID 6 Register
#define GICD_PIDR7 0xFDC // RO 0x00000000 Peripheral ID 7 Register
#define GICD_PIDR0 0xFE0 // RO 0x00000090 Peripheral ID 0 Register
#define GICD_PIDR1 0xFE4 // RO 0x000000B4 Peripheral ID 1 Register
#define GICD_PIDR2 0xFE8 // RO 0x0000002B Peripheral ID 2 Register
#define GICD_PIDR3 0xFEC // RO 0x00000000 Peripheral ID 3 Register
#define GICD_CIDR0 0xFF0 // RO 0x0000000D Component ID 0 Register
#define GICD_CIDR1 0xFF4 // RO 0x000000F0 Component ID 1 Register
#define GICD_CIDR2 0xFF8 // RO 0x00000005 Component ID 2 Register
#define GICD_CIDR3 0xFFC // RO 0x000000B1 Component ID 3 Register



#define GICC_CPU_BASE           (GIC_BASE + 0x00002000)

#define GICD_ENABLE_IRQ_BASE    (GICD_DIST_BASE + 0x00000100)
#define GICC_IAR                (GICC_CPU_BASE + 0x0000000C)
#define GICC_EOIR               (GICC_CPU_BASE + 0x00000010)
#define GIC_IRQ_TARGET_BASE     (GICD_DIST_BASE + 0x00000800)

// VC (=VideoCore) starts at 96
// Pg. 89 of BCM2711 ARM Peripherals
// CoreLink GIC-400 Generic Interrupt Controller Technical Reference Manual
#define SYSTEM_TIMER_IRQ_0 (0x60) //96
#define SYSTEM_TIMER_IRQ_1 (0x61) //97
#define SYSTEM_TIMER_IRQ_2 (0x62) //98
#define SYSTEM_TIMER_IRQ_3 (0x63) //99

// Raspberry Pi 3b

// #define IRQ_BASIC_PENDING (PBASE+0x0000B200)
// #define IRQ_PENDING_1 (PBASE+0x0000B204)
// #define IRQ_PENDING_2 (PBASE+0x0000B208)
// #define FIQ_CONTROL	(PBASE+0x0000B20C)
// #define ENABLE_IRQS_1 (PBASE+0x0000B210)
// #define ENABLE_IRQS_2 (PBASE+0x0000B214)
// #define ENABLE_BASIC_IRQS (PBASE+0x0000B218)
// #define DISABLE_IRQS_1 (PBASE+0x0000B21C)
// #define DISABLE_IRQS_2 (PBASE+0x0000B220)
// #define DISABLE_BASIC_IRQS (PBASE+0x0000B224)

// #define SYSTEM_TIMER_IRQ_0 (1 << 0)
// #define SYSTEM_TIMER_IRQ_1 (1 << 1)
// #define SYSTEM_TIMER_IRQ_2 (1 << 2)
// #define SYSTEM_TIMER_IRQ_3 (1 << 3)

#endif  /*_P_IRQ_H */