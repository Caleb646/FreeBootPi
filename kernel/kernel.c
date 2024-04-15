//#include "printf.h"
//#include "irq.h"
#include "utils.h"
#include "uart.h"

void kernel_main(void)
{
    uart_init();
	//init_printf(0, putc);
	//irq_vector_init();
	//enable_interrupt_controller();
	//enable_irq();
    while (1) {
        //uart_send(uart_recv());
        uart_send_string("Hello\r\n");
        delay(500);
    }
}
