#ifndef	_P_GPIO_H
#define	_P_GPIO_H

#include "base.h"
#include "peripherals/base.h"

// Physical Address with Low Peripheral mode disabled is: 0x4_7C00_0000
// GPIO Physical Address with Low Peripheral mode disabled is: 0x4_7E20_0000
// Subtracting 0x4_7E20_0000 - 0x4_7C00_0000 = 0x220_0000
#define GPIO_BASE        PBASE + 0x2200000
#define GPFSEL0         (GPIO_BASE + 0x00)
// Determines the alternate function of GPIO pins 13-19
#define GPFSEL1         (GPIO_BASE + 0x04)
#define GPFSEL2         (GPIO_BASE + 0x08)
#define GPFSEL3         (GPIO_BASE + 0x0C)
#define GPFSEL4         (GPIO_BASE + 0x10)
#define GPFSEL5         (GPIO_BASE + 0x14)
#define GPSET0          (GPIO_BASE + 0x1C)
#define GPSET1          (GPIO_BASE + 0x20)
// The output clear registers are used to clear a GPIO pin
#define GPCLR0          (GPIO_BASE + 0x28)
#define GPLEV0          (GPIO_BASE + 0x34)
#define GPLEV1          (GPIO_BASE + 0x38)
#define GPEDS0          (GPIO_BASE + 0x40)
#define GPEDS1          (GPIO_BASE + 0x44)
#define GPHEN0          (GPIO_BASE + 0x64)
#define GPHEN1          (GPIO_BASE + 0x68)
#define GPPUD           (GPIO_BASE + 0x94)
#define GPPUDCLK0       (GPIO_BASE + 0x98)
#define GPPUDCLK1       (GPIO_BASE + 0x9C)
// The GPIO Pull-up / Pull-down Registers control the actuation of the internal pull-up/down resistors. 
// Reading these registers gives the current pull-state.
// The Alternate function table also has the pull state which is applied after a power down
// For GPIO Pins 10-15
// NOTE: this may be a bad idea because of having a MMU
#define GPIO_PUP_PDN_CNTRL_REG0 (GPIO_BASE + 0xE4)  // *((u32 volatile *)(GPIO_BASE + 0xE4))

#endif  /*_P_GPIO_H */