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
    LOG_INFO("Arm Core [%u] has GIC Id of [%u]", target);
	return target;
}

void gic_assign_target(u32 irq_id, u32 gic_cpu_id)
{
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

s32 gic_init(void) 
{	
    // armstub8.S sets all interrupts to group 1 (non-secure irqs)
	// and enables the GICD and each GICC for ALL cores
    if(get_arm_core_id() == 0) /* Primary Core */
    {
        put32(GICD_CTLR, GICD_CTLR_DISABLE);

        /* Clear any enabled, active, or pending interrupts */
        for (u32 n = 0; n < GIC_NUM_INTERRUPTS / 32; ++n)
        {
            put32(GICD_ICENABLERn(n), ~0);
            put32(GICD_ICPENDRn(n), ~0);
            put32(GICD_ICACTIVERn(n), ~0);
        }
        /* 
        * Set all interrupts priorities to 0 and set every interrupts target to be core 0.
        * Higher interrupt priority corresponds to a lower value of the Priority field.
        * Each 32 bit register holds the priority status for 4 interrupt types. Allowing
        * for 8 bits per interrupt type.
        */
        for (u32 n = 0; n < GIC_NUM_INTERRUPTS / 4; ++n)
        {
            put32(
                GICD_IPRIORITYRn(n), 
                (GICD_IPRIORITY_DEFAULT_PRIORITY << 0)  |
                (GICD_IPRIORITY_DEFAULT_PRIORITY << 8)  |
                (GICD_IPRIORITY_DEFAULT_PRIORITY << 16) |
                (GICD_IPRIORITY_DEFAULT_PRIORITY << 24)
            );
            put32(GICD_ITARGETSRn(n), 0x0);
        }
        /* 
        * Set all interrupts to level-triggered.
        * Each GICD_ICFGR register is 32 bits with 2 bits allocated for each
        * interrupt type.
        * 0b00 == level-triggered
        * 0b10 == edge-triggered
        */
        for (u32 n = 0; n < GIC_NUM_INTERRUPTS / 16; ++n)
        {
            put32(GICD_ICFGRn(n), GICD_ICFGR_LEVEL_TRIGGERED);
        }

        /* Initialize all handlers to the general handler */
        for(u32 cid = 0; cid < ARM_NUM_CORES; ++cid)
        {
            for(u32 i = 0; i < GIC_NUM_INTERRUPTS; ++i)
            {
                gic_irq_handlers[cid][i] = &gic_general_irq_handler;
            }
        }
        put32(GICD_CTLR, GICD_CTLR_ENABLE);
    }
    /* Enable GIC CPU Interface for each core */
    put32(GICC_PMR, GICC_PMR_16_PRIORITY_LEVELS);
	put32(GICC_CTLR, GICC_CTLR_ENABLE);
    
    return 1;
}

s32 irq_init(void)
{
    s32 status = gic_init();
    ENABLE_IRQ();
    return status;
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
