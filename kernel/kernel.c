#include "base.h"
#include "printf.h"
#include "kernel.h"
#include "peripherals/timer.h"
#include "peripherals/gic.h"
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

u32 gic_get_cpu_id(void)
{
	u32 target = get32(GICD_ITARGETSR0) & 0xFF;
	if(target > 7)
	{
		LOG_ERROR("Invalid CPU id read from the GIC: [%u]", target);
	}
	return target;
}

void gic_enable_interrupt(u32 irq) 
{
	u32 int_reg_id = irq / 32;
	u32 bit_offset = irq % 32;
	u64 enable_register = GICD_ISENABLERn(int_reg_id);
	LOG_DEBUG("Enabling interrupt [%u] with register [%u]", irq, int_reg_id);
	put32(enable_register, get32(enable_register) | (1 << bit_offset));
}

void gic_assign_target(u32 irq_id, u32 gic_cpu_id)
{
	// u32 int_reg_id = irq / 4;
	// u32 target_register = GICD_ITARGETSRn(int_reg_id);
	// // Currently only enter the target CPU 0
	// put32(target_register, get32(target_register) | (1 << 8));
	u32 int_reg_id = irq_id / 4;
	u32 bit_offset = irq_id % 4;
	// Page 598 of Arm Generic Interrupt Controller Spec
	u32 offsets[4] = {0, 8, 16, 24};
	if(bit_offset > 3)
	{
		LOG_ERROR(
			"Failed to Assign Target: [%u] to GIC CPU [%u] because of invalid bit offset [%u]", 
			irq_id, 
			gic_cpu_id, 
			bit_offset
			);
		return;
	}
	u64 reg = GICD_ITARGETSRn(int_reg_id);
	put32(reg, get32(reg) | (1 << (gic_cpu_id + offsets[bit_offset])));
}

void gic_enable(void) 
{	
	gic_assign_target(VC_SYSTEM_TIMER_IRQ_1, gic_get_cpu_id());
	gic_enable_interrupt(VC_SYSTEM_TIMER_IRQ_1);
}

static u32 volatile s_ncritical_lvl = 0;
static u64 volatile s_int_flags[MAX_NESTED_INTERRUPTS];
/*
* \brief	Returns ARM core interrupt status
* \return	
* 0 is not masked (enabled) & 1 is masked (disabled)
* Bit 9 = Watchpoint, Breakpoint, and Software Step exceptions 
* Bit 8 = SError exception
* Bit 7 = IRQ mask
* Bit 6 = FIQ mask
*/
u64 get_daif_flags(void)
{
	u64 flags;
	asm volatile ("mrs %0, daif" : "=r" (flags));
	return flags;
}

void set_daif_flags(u64 flags)
{
	asm volatile ("msr daif, %0" :: "r" (flags));
}

s32 enter_critical(u32 target_lvl)
{
	u64 flags = get_daif_flags();
	if(s_ncritical_lvl >= MAX_NESTED_INTERRUPTS)
	{
		return 0;
	}
	// Save DAIF flags to be restored on leaving interrupt
	s_int_flags[s_ncritical_lvl++] = flags;
	DISABLE_IRQ_FIQ;
	return 1;
}

s32 leave_critical(void)
{
	if(s_ncritical_lvl <= 0)
	{
		return 0;
	}
	// restore saved daif flags
	u64 flags = s_int_flags[--s_ncritical_lvl];
	set_daif_flags(flags);
	return 1;
}

void show_invalid_entry_message(s32 type, u32 esr, u32 address) 
{
	LOG_ERROR("%s, ESR: %x, address, %x", entry_error_messages[type], esr, address);
}

void handle_irq(void) 
{
	u32 irq_ack_reg = get32(GICC_IAR);
	u32 irq = irq_ack_reg & 0x2FF;
	switch (irq) 
	{
		case (VC_SYSTEM_TIMER_IRQ_1):
			//handle_timer_irq();
			put32(GICC_EOIR, irq_ack_reg);
			LOG_DEBUG("Received VC Timer Interrupt");
			break;
		default:
			LOG_ERROR("Unknown pending irq: %x", irq);
			put32(GICC_EOIR, irq_ack_reg);
			break;
	}
}

void kernel_main(void)
{
	init_printf(0, putc);
	enable_irq();
	// armstub8.S sets all interrupts to group 1 (non-secure irqs)
	// and enables the GICD and each GICC for ALL cores
	gic_enable();
    while (1) 
	{
        uart_send_string("Hello from Kernel\r\n");
        delay(500);
    }
}
