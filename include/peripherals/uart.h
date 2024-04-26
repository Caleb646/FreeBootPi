#ifndef _UART_H
#define _UART_H

#include "base.h"
#include "peripherals/gpio.h"
#include "peripherals/aux.h"

// Physical Address with Low Peripheral mode disabled is: 0x4_7C00_0000
// UART0 Physical Address with Low Peripheral mode disabled is: 0x47e201000
// Subtracting 0x47e201000 - 0x4_7C00_0000 = 0x2201000
#define UART0_BASE          (PBASE + 0x2201000)
#define UART0_DR            (UART0_BASE)        // Data Register
#define UART0_RSECR         (UART0_BASE + 0x04)
#define UART0_FR            (UART0_BASE + 0x18) // Flag register
#define UART0_ILPR          (UART0_BASE + 0x20) // not in use
#define UART0_IBRD          (UART0_BASE + 0x24) // Integer Baud rate divisor
#define UART0_FBRD          (UART0_BASE + 0x28) // Fractional Baud rate divisor
#define UART0_LCRH          (UART0_BASE + 0x2c) // Line Control register
#define UART0_CR            (UART0_BASE + 0x30) // Control register
#define UART0_IFLS          (UART0_BASE + 0x34) // Interrupt FIFO Level Select Register
#define UART0_IMSC          (UART0_BASE + 0x38) // Interrupt Mask Set Clear Register
#define UART0_RIS           (UART0_BASE + 0x3c) // Raw Interrupt Status Register
#define UART0_MIS           (UART0_BASE + 0x40) // Masked Interrupt Status Register
#define UART0_ICR           (UART0_BASE + 0x44) // Interrupt Clear Register
#define UART0_DMAC          (UART0_BASE + 0x48) // DMA Control Register
#define UART0_ITCR          (UART0_BASE + 0x80) // Test Control register
#define UART0_ITIP          (UART0_BASE + 0x84) // Integration test input reg
#define UART0_ITOP          (UART0_BASE + 0x88) // Integration test output reg
#define UART0_TDR           (UART0_BASE + 0x8c) // Test Data reg


void uart_init(void);
char uart_recv(void);
void uart_send(char c);
void uart_send_string(char* str);
void putc (void* p, char c);

#endif