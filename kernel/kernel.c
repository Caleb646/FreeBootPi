#include "base.h"
#include "printf.h"
#include "kernel.h"
#include "irq.h"
#include "peripherals/timer.h"
#include "peripherals/uart.h"
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

void show_invalid_entry_message(s32 type, u32 esr, u32 address) 
{
	LOG_ERROR("%s, ESR: %x, address, %x", entry_error_messages[type], esr, address);
}

void kernel_main(void)
{
	init_printf(0, putc);
	LOG_DEBUG("Hello from Kernel");
	irq_init();
	timer_init(VC_GIC_SYSTEM_TIMER_IRQ_3);
    while (1) 
	{
        LOG_DEBUG("System Time: [%u]", get_sys_time_s());
		wait_ms(1000); //
    }
}
