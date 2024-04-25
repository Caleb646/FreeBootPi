#include "base.h"
#include "utils.h"
#include "uart.h"
#include "printf.h"
#include "timer.h"
#include "kernel.h"
#include "peripherals/gic.h"
#include "arm/sysregs.h"

const char* entry_error_messages[] = {
	"SYNC_INVALID_EL1t",
	"IRQ_INVALID_EL1t",		
	"FIQ_INVALID_EL1t",		
	"ERROR_INVALID_EL1t",		

	"SYNC_INVALID_EL1h",		
	"IRQ_INVALID_EL1h",		
	"FIQ_INVALID_EL1h",		
	"ERROR_INVALID_EL1h",		

	"SYNC_INVALID_EL0_64",		
	"IRQ_INVALID_EL0_64",		
	"FIQ_INVALID_EL0_64",		
	"ERROR_INVALID_EL0_64",	

	"SYNC_INVALID_EL0_32",		
	"IRQ_INVALID_EL0_32",		
	"FIQ_INVALID_EL0_32",		
	"ERROR_INVALID_EL0_32"	
};

void enable_interrupt(u32 irq) 
{
	 LOG_DEBUG("%x", irq);
	 u32 n = irq / 32;
	 u32 offset = irq % 32;
	 u32 enable_register = GICD_ENABLE_IRQ_BASE + (4*n);
	 LOG_DEBUG("EnableRegister: %x", enable_register);
	 put32(enable_register, 1 << offset);
}

void assign_target(u32 irq, u32 cpu)
{
	 u32 n = irq / 4;
	 u32 target_register = GIC_IRQ_TARGET_BASE + (4*n);
	 // Currently only enter the target CPU 0
	 put32(target_register, get32(target_register) | (1 << 8));
}

void show_invalid_entry_message(s32 type, u32 esr, u32 address) 
{
	 LOG_ERROR("%s, ESR: %x, address, %x", entry_error_messages[type], esr, address);
}

void enable_interrupt_controller() 
{	
	 assign_target(SYSTEM_TIMER_IRQ_1, 0);
	 enable_interrupt(SYSTEM_TIMER_IRQ_1);
}

void handle_irq(void) 
{
	 u32 irq_ack_reg = get32(GICC_IAR);
	 u32 irq = irq_ack_reg & 0x2FF;
	 switch (irq) 
     {
	 	 case (SYSTEM_TIMER_IRQ_1):
	 	 	 //handle_timer_irq();
	 	 	 put32(GICC_EOIR, irq_ack_reg);
	 	 	 break;
	 	 default:
	 	 	 LOG_ERROR("Unknown pending irq: %x", irq);
	 	 	 break;
	}
}

void kernel_main(void)
{
	init_printf(0, putc);
	enable_interrupt_controller();
	enable_irq();
    while (1) {
        uart_send_string("Hello from Kernel\r\n");
        delay(500);
    }
}
