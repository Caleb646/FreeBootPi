#include "utils.h"
#include "printf.h"
#include "timer.h"
#include "entry.h"
#include "peripherals/irq.h"
#include "arm/sysregs.h"

const char *entry_error_messages[] = {
	"SYNC_INVALID_EL1t",
	"IRQ_INVALID_EL1t",		
	"FIQ_INVALID_EL1t",		
	"ERROR_INVALID_EL1T",		

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

void enable_interrupt(unsigned int irq) 
{
	 printf("%x\r\n", irq);
	 unsigned int n = irq / 32;
	 unsigned int offset = irq % 32;
	 unsigned int enable_register = GICD_ENABLE_IRQ_BASE + (4*n);
	 printf("EnableRegister: %x\r\n", enable_register);
	 put32(enable_register, 1 << offset);
}

void assign_target(unsigned int irq, unsigned int cpu)
{
	 unsigned int n = irq / 4;
	 unsigned int target_register = GIC_IRQ_TARGET_BASE + (4*n);
	 // Currently we only enter the target CPU 0
	 put32(target_register, get32(target_register) | (1 << 8));
}

void show_invalid_entry_message(int type, unsigned long esr, unsigned long address) 
{
	 printf("%s, ESR: %x, address, %x\r\n", entry_error_messages[type], esr, address);
}

void enable_interrupt_controller() 
{	
	 assign_target(SYSTEM_TIMER_IRQ_1, 0);
	 enable_interrupt(SYSTEM_TIMER_IRQ_1);
}

void handle_irq(void) 
{
	 unsigned int irq_ack_reg = get32(GICC_IAR);
	 unsigned int irq = irq_ack_reg & 0x2FF;
	 switch (irq) 
     {
	 	 case (SYSTEM_TIMER_IRQ_1):
	 	 	 //handle_timer_irq();
	 	 	 put32(GICC_EOIR, irq_ack_reg);
	 	 	 break;
	 	 default:
	 	 	 printf("Unknown pending irq: %x\r\n", irq);
	 	 	 break;
	}
}