#include "mini_uart.h"
#include "peripherals/gpio.h"

void uart_init (void)
{
    unsigned int selector;
    selector = get32(GPFSEL1);
    selector &= ~(7<<12);                   // clean gpio 14
    selector |= 2<<12;                      // set alt5 for gpio 14
    selector &= ~(7<<15);                   // clean gpio 15
    selector |= 2<<15;                      // set alt5 for gpio 15
    put32(GPFSEL1, selector);

    /*
     * The GPIO Pull-up/down Clock Registers control the actuation of internal pull-downs on
     * the respective GPIO pins. These registers must be used in conjunction with the GPPUD
     * register to effect GPIO Pull-up/down changes. 
     * 00 = Off – disable pull-up/down
     * 01 = Enable Pull Down control
     * 10 = Enable Pull Up control
     * 11 = Reserved 
   */
    // remove pull down/up resistors for both pins 14 and 15
    // set bits 31-28 to 0 but leave the rest unchanged
    GPIO_PUP_PDN_CNTRL_REG0 &= ~((3<<30) | (3<<28));
    delay(150);

    put32(AUX_ENABLES_REG, 1);                   // Enable mini uart (this also enables access to its registers)
    put32(AUX_MU_CNTL_REG, 0);               // Disable auto flow control and disable receiver and transmitter (for now)
    put32(AUX_MU_IER_REG, 0);                // Disable receive and transmit interrupts
    put32(AUX_MU_LCR_REG, 3);                // Enable 8 bit mode
    put32(AUX_MU_MCR_REG, 0);                // Set RTS line to be always high
    put32(AUX_MU_BAUD_REG, 270);             // Set baud rate to 115200
 
    put32(AUX_MU_CNTL_REG, 3);               // Finally, enable transmitter and receiver
}

void uart_send (char c)
{
    while(1) 
    {
        if(get32(AUX_MU_LSR_REG) & 0x20) 
        {
            break;
        }
    }
    put32(AUX_MU_IO_REG,c);
}

char uart_recv (void)
{
    while(1) 
    {
        if(get32(AUX_MU_LSR_REG) & 0x01) 
        {
            break;
        }
    }
    return(get32(AUX_MU_IO_REG) & 0xFF);
}

void uart_send_string(char* str)
{
    for (int i = 0; str[i] != '\0'; ++i) 
    {
        uart_send((char)str[i]);
    }
}

// This function is required by printf function
void putc ( void* p, char c)
{
	uart_send(c);
}