//#include "printf.h"
//#include "utils.h"
//#include "irq.h"
#include "mini_uart.h"

void kernel_main(void)
{
    uart_init();
	//init_printf(0, putc);
	//irq_vector_init();
	//enable_interrupt_controller();
	//enable_irq();

    uart_send_string("Hello, world!\r\n");
    while (1) {
        uart_send(uart_recv());
    }
}
