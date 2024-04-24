#include "peripherals/gpio.h"
#include "peripherals/aux.h"
#include "base.h"

#define REG_PTR(addr) *((u32 volatile *)addr)
#define KERNEL_LOAD_ADDR 0x8000

static void mini_uart_init(void)
{
    REG_PTR(AUX_ENABLES_REG) = 1;               // Enable mini uart (this also enables access to its registers)
    REG_PTR(AUX_MU_CNTL_REG) = 0;               // Disable auto flow control and disable receiver and transmitter (for now)
    REG_PTR(AUX_MU_IER_REG) = 0;             // Disable receive and transmit interrupts
    REG_PTR(AUX_MU_LCR_REG) = 3;             // Enable 8 bit mode
    REG_PTR(AUX_MU_MCR_REG) = 0;             // Set RTS line to be always high
    REG_PTR(AUX_MU_IIR_REG) = 0xC6;             // Disable Interrupts
    REG_PTR(AUX_MU_BAUD_REG) = AUX_MU_BAUD(115200);             // Set baud rate

    u32 selector;
    selector = REG_PTR(GPFSEL1);
    selector &= ~(7 << 12);                   // clean gpio 14
    selector |= 2 << 12;                      // set alt5 for gpio 14
    selector &= ~(7 << 15);                   // clean gpio 15
    selector |= 2 << 15;                      // set alt5 for gpio 15
    REG_PTR(GPFSEL1) = selector;                     

    // remove pull down/up resistors for both pins 14 and 15
    // set bits 31-28 to 0 but leave the rest unchanged
    REG_PTR(GPIO_PUP_PDN_CNTRL_REG0) &= ~((3 << 30) | (3 << 28));
    REG_PTR(AUX_MU_CNTL_REG) = 3;               // Enable transmitter and receiver
}

static void uart_send(char c)
{
    while(1) 
    {
        if(REG_PTR(AUX_MU_LSR_REG) & 0x20) 
        {
            break;
        }
    }
    REG_PTR(AUX_MU_IO_REG) = c;
}

static void uart_send_string(char* str)
{
    for (int i = 0; str[i] != '\0'; ++i) 
    {
        uart_send((char)str[i]);
    }
}

static char uart_recv(void)
{
    while(1) 
    {
        if(REG_PTR(AUX_MU_LSR_REG) & 0x01) 
        {
            break;
        }
    }
    return(REG_PTR(AUX_MU_IO_REG) & 0xFF);
}

static void delay(u32 cycles)
{
    while(cycles-- > 0)
    {
        asm volatile ("nop");
    }
}

void loader_main(void)
{
    mini_uart_init();
    while (1) 
    {
        uart_send_string("Hello\r\n");
        delay(10000);
    }

    restart:
    uart_send_string("Beginning Kernel Loading Process");
    // request kernel by sending 3 breaks
    uart_send_string("\x03\x03\x03");

    // get kernel size
    u32 size = uart_recv();
    size |= uart_recv() << 8;
    size |= uart_recv() << 16;
    size |= uart_recv() << 24;
    uart_send_string("OK");
    
    // get kernel
    u8* kernel = (u8*)KERNEL_LOAD_ADDR;
    while(size-- > 0)
    {
	    *kernel++ = uart_recv();
    }

    // Kernel is loaded at 0x8000, call it via function pointer
    uart_send_string("Booting");
    typedef void (*entry_fn)();
    entry_fn fn = (entry_fn)KERNEL_LOAD_ADDR;
    fn();
}