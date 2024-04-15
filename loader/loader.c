#include "peripherals/gpio.h"
#include "peripherals/aux.h"
#include "base.h"

#define MOD_REG(addr) *((u32 volatile *)addr)

static void mini_uart_init(void)
{
    MOD_REG(AUX_ENABLES_REG) = 1;               // Enable mini uart (this also enables access to its registers)
    MOD_REG(AUX_MU_CNTL_REG) = 0;               // Disable auto flow control and disable receiver and transmitter (for now)
    MOD_REG(AUX_MU_IER_REG) = 0;             // Disable receive and transmit interrupts
    MOD_REG(AUX_MU_LCR_REG) = 3;             // Enable 8 bit mode
    MOD_REG(AUX_MU_MCR_REG) = 0;             // Set RTS line to be always high
    MOD_REG(AUX_MU_IIR_REG) = 0xC6;             // Disable Interrupts
    MOD_REG(AUX_MU_BAUD_REG) = AUX_MU_BAUD(115200);             // Set baud rate

    u32 selector;
    selector = MOD_REG(GPFSEL1);
    selector &= ~(7 << 12);                   // clean gpio 14
    selector |= 2 << 12;                      // set alt5 for gpio 14
    selector &= ~(7 << 15);                   // clean gpio 15
    selector |= 2 << 15;                      // set alt5 for gpio 15
    MOD_REG(GPFSEL1) = selector;                     

    // remove pull down/up resistors for both pins 14 and 15
    // set bits 31-28 to 0 but leave the rest unchanged
    MOD_REG(GPIO_PUP_PDN_CNTRL_REG0) &= ~((3 << 30) | (3 << 28));
    MOD_REG(AUX_MU_CNTL_REG) = 3;               // Enable transmitter and receiver
}

static void uart_send(char c)
{
    while(1) 
    {
        if(MOD_REG(AUX_MU_LSR_REG) & 0x20) 
        {
            break;
        }
    }
    MOD_REG(AUX_MU_IO_REG) = c;
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
        if(MOD_REG(AUX_MU_LSR_REG) & 0x01) 
        {
            break;
        }
    }
    return(MOD_REG(AUX_MU_IO_REG) & 0xFF);
}

void loader_main(void)
{
    mini_uart_init();
    while (1) 
    {
        uart_send_string("Hello\r\n");
        //delay(500);
    }
}