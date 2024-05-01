#include "irq.h"
#include "printf.h"

extern irq_handler_t gic_irq_handlers[ARM_NUM_CORES][GIC_NUM_INTERRUPTS];

u32 gic_get_cpu_id(void)
{
	u32 target = get32(GICD_ITARGETSR0) & 0xFF;
	if(target > 7)
	{
		LOG_ERROR("Invalid CPU id read from the GIC Distributor: [%u]", target);
	}
	return target;
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

void gic_enable_interrupt(u32 irq, irq_handler_t handler) 
{
    // Store handler for use later
    gic_irq_handlers[get_arm_core_id()][irq] = handler;
    // Assign this irq to the calling cpu core
    gic_assign_target(irq, gic_get_cpu_id());

	u32 int_reg_id = irq / 32;
	u32 bit_offset = irq % 32;
	u64 enable_register = GICD_ISENABLERn(int_reg_id);
	LOG_DEBUG("Enabling interrupt [%u] with register [%u]", irq, int_reg_id);
	put32(enable_register, get32(enable_register) | (1 << bit_offset));
}

void gic_general_irq_handler(u32 irq_id)
{
    LOG_DEBUG(
        "Interrupt [%u] for core [%u] does NOT have a handler", irq_id, get_arm_core_id()
        );
}

void gic_init(void) 
{	
	// gic_assign_target(VC_GIC_SYSTEM_TIMER_IRQ_1, gic_get_cpu_id());
	// gic_enable_interrupt(VC_GIC_SYSTEM_TIMER_IRQ_1);
    for(u32 cid = 0; cid < ARM_NUM_CORES; ++cid)
    {
        for(u32 i = 0; i < GIC_NUM_INTERRUPTS; ++i)
        {
            gic_irq_handlers[cid][i] = &gic_general_irq_handler;
        }
    }
}

void handle_irq(void) 
{
	u32 irq_ack_reg = get32(GICC_IAR);
	u32 irq = irq_ack_reg & 0x2FF;
    if(irq < GIC_NUM_INTERRUPTS)
    {
        gic_irq_handlers[get_arm_core_id()][irq](irq);
        put32(GICC_EOIR, irq_ack_reg);
    }
    else
    {
        LOG_ERROR("Received IRQ ID [%u] outside acceptable range", irq);
    }
}
