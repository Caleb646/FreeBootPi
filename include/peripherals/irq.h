#ifndef	_P_IRQ_H
#define	_P_IRQ_H

#include "peripherals/base.h"

// Raspberry Pi 4b
// Manuals:
// BCM2711 ARM Peripherals
// CoreLink GIC-400 Generic Interrupt Controller Technical Reference Manual
// Arm Generic Interrupt Controller Architecture Specification GIC architecture
// Example: Linux Driver
// https://github.com/torvalds/linux/blob/v6.9-rc3/drivers/irqchip/irq-gic-v3.c
// https://github.com/torvalds/linux/blob/v6.9-rc3/include/linux/irqchip/arm-gic-v3.h
#define GIC_BASE 0xFF840000
#define GICD_DIST_BASE (GIC_BASE+0x00001000)
#define GICC_CPU_BASE (GIC_BASE+0x00002000)
#define GICD_ENABLE_IRQ_BASE (GICD_DIST_BASE+0x00000100)
#define GICC_IAR (GICC_CPU_BASE+0x0000000C)
#define GICC_EOIR (GICC_CPU_BASE+0x00000010)
#define GIC_IRQ_TARGET_BASE (GICD_DIST_BASE+0x00000800)

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