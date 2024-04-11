#ifndef MINI_UART_H
#define MINI_UART_H

#include "peripherals/base.h"

// Physical Address with Low Peripheral mode disabled is: 0x4_7C00_0000
// Mini UART Physical Address with Low Peripheral mode disabled is: 0x4_7E21_5000
// Subtracting 0x4_7E21_5000 - 0x4_7C00_0000 = 0x221_5000
#define MINI_UART_BASE     PBASE + 0x2215000
#define AUX_ENABLES_REG    (MINI_UART_BASE + 0x04)
#define AUX_MU_IO_REG      (MINI_UART_BASE + 0x40)
#define AUX_MU_IER_REG     (MINI_UART_BASE + 0x44)
#define AUX_MU_IIR_REG     (MINI_UART_BASE + 0x48)
#define AUX_MU_LCR_REG     (MINI_UART_BASE + 0x4C)
#define AUX_MU_MCR_REG     (MINI_UART_BASE + 0x50)
#define AUX_MU_LSR_REG     (MINI_UART_BASE + 0x54)
#define AUX_MU_MSR_REG     (MINI_UART_BASE + 0x58)
#define AUX_MU_SCRATCH_REG (MINI_UART_BASE + 0x5C)
#define AUX_MU_CNTL_REG    (MINI_UART_BASE + 0x60)
#define AUX_MU_STAT_REG    (MINI_UART_BASE + 0x64)
#define AUX_MU_BAUD_REG    (MINI_UART_BASE + 0x68)

void uart_init (void);
char uart_recv (void);
void uart_send (char c);
void uart_send_string(char* str);
void putc (void* p, char c);

#endif