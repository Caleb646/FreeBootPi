#ifndef	_P_GPIO_H
#define	_P_GPIO_H

#include "actypes.h"
#include "peripherals/base.h"

// Physical Address with Low Peripheral mode disabled is: 0x4_7C00_0000
// GPIO Physical Address with Low Peripheral mode disabled is: 0x4_7E20_0000
// Subtracting 0x4_7E20_0000 - 0x4_7C00_0000 = 0x220_0000
#define GPIO_BASE        PBASE + 0x2200000
// Determines the alternate function of GPIO pins 13-19
#define GPFSEL1         (GPIO_BASE + 0x04)
#define GPSET0          (GPIO_BASE + 0x1C)
// The output clear registers are used to clear a GPIO pin
#define GPCLR0          (GPIO_BASE + 0x28)
// The GPIO Pull-up / Pull-down Registers control the actuation of the internal pull-up/down resistors. 
// Reading these registers gives the current pull-state.
// The Alternate function table also has the pull state which is applied after a power down
// For GPIO Pins 10-15
#define GPIO_PUP_PDN_CNTRL_REG0 *((u32 volatile *)(GPIO_BASE + 0xE4))

#endif  /*_P_GPIO_H */