#include "./mini_uart.h"

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
     * register to effect GPIO Pull-up/down changes. The following sequence of events is
     * required:
     * 1. Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
     * to remove the current Pull-up/down)
     * 2. Wait 150 cycles – this provides the required set-up time for the control signal
     * 3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
     * modify – NOTE only the pads which receive a clock will be modified, all others will
     * retain their previous state.
     * 4. Wait 150 cycles – this provides the required hold time for the control signal
     * 5. Write to GPPUD to remove the control signal
     * 6. Write to GPPUDCLK0/1 to remove the clock
    */
   /*
    00 = Off – disable pull-up/down
    01 = Enable Pull Down control
    10 = Enable Pull Up control
    11 = Reserved 
   */
    put32(GPPUD, 0);
    delay(150);
    put32(GPPUDCLK0, (1<<14) | (1<<15)); // remove pull down/up resistors for both pins 14 and 15
    delay(150);
    put32(GPPUDCLK0, 0);

    put32(AUX_ENABLES, 1);                   // Enable mini uart (this also enables access to its registers)
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
            break;
    }
    put32(AUX_MU_IO_REG,c);
}

char uart_recv (void)
{
    while(1) 
    {
        if(get32(AUX_MU_LSR_REG) & 0x01) 
            break;
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