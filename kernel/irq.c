#include "irq.h"

static irq_handler_t volatile irq_handlers_[ARM_NUM_CORES][GIC_NUM_INTERRUPTS];
static void* volatile irq_contexts_[ARM_NUM_CORES][GIC_NUM_INTERRUPTS];

/*
 * \brief	Returns ARM core interrupt status
 * \return
 * 0 is not masked (enabled) & 1 is masked (disabled)
 * Bit 9 = Watchpoint, Breakpoint, and Software Step exceptions
 * Bit 8 = SError exception
 * Bit 7 = IRQ mask
 * Bit 6 = FIQ mask
 */
u64 get_daif_flags (void) {
    u64 flags;
    asm volatile("mrs %0, daif" : "=r"(flags));
    return flags;
}

void set_daif_flags (u64 flags) {
    asm volatile("msr daif, %0" ::"r"(flags));
}

// u32 gic_get_cpu_id(void)
// {
// 	u32 target = read32(GICD_ITARGETSR0) & 0xFF;
// 	if(target > 7)
// 	{
// 		LOG_ERROR("Invalid CPU id read from the GIC Distributor: [%u]", target);
// 	}
//     LOG_INFO("Arm Core [%u] has GIC Id of [%u]", get_arm_core_id(), target);
// 	return target;
// }

void gic_assign_target (u32 irq_id, u32 cpu_id) {
    u32 int_reg_id = irq_id / 4;
    u32 bit_offset = irq_id % 4;
    // Page 598 of Arm Generic Interrupt Controller Spec
    u32 offsets[4] = { 0, 8, 16, 24 };
    if (bit_offset > 3) {
        LOG_ERROR (
        "Failed to Assign Target: [%u] to CPU [%u] because of "
        "invalid bit offset [%u]",
        irq_id, cpu_id, bit_offset);
        return;
    }
    u64 reg = GICD_ITARGETSRn (int_reg_id);
    write32 (reg, read32 (reg) | (1 << (cpu_id + offsets[bit_offset])));
}

/*
 * \brief Updates the irq's handler to the new handler for the calling cpu core.
 *  Expects that the irq has already been enabled on the GIC and the calling cpu
 * core has been assigned as the target.
 */
void gic_assign_irq_handler (u32 irq_id, irq_handler_t handler, void* context) {
    irq_handlers_[get_arm_core_id ()][irq_id] = handler;
    irq_contexts_[get_arm_core_id ()][irq_id] = context;
}

/*
 * \brief Enables the irq on the GIC and assigns the calling cpu core as the
 * target of the interrupt.
 */
void gic_enable_interrupt (u32 irq, irq_handler_t handler, void* context) {
    // Store handler for use later
    gic_assign_irq_handler (irq, handler, context);
    // Assign this irq to the calling cpu core
    gic_assign_target (irq, get_arm_core_id ());

    u32 int_reg_id      = irq / 32;
    u32 bit_offset      = irq % 32;
    u64 enable_register = GICD_ISENABLERn (int_reg_id);
    LOG_DEBUG ("Enabling interrupt [%u] with register [%u]", irq, int_reg_id);
    write32 (enable_register, read32 (enable_register) | (1 << bit_offset));
}

static void gic_general_irq_handler (u32 irq_id, void* context) {
    LOG_DEBUG ("Interrupt [%u] for core [%u] does NOT have a handler", irq_id, get_arm_core_id ());
}

s32 gic_init (void) {
    // armstub8.S sets all interrupts to group 1 (non-secure irqs)
    // and enables the GICD and each GICC for ALL cores
    if (get_arm_core_id () == 0) /* Primary Core */
    {
        write32 (GICD_CTLR, GICD_CTLR_DISABLE);

        /* Clear any enabled, active, or pending interrupts */
        for (u32 n = 0; n < GIC_NUM_INTERRUPTS / 32; ++n) {
            write32 (GICD_ICENABLERn (n), ~0);
            write32 (GICD_ICPENDRn (n), ~0);
            write32 (GICD_ICACTIVERn (n), ~0);
        }
        /*
         * Set all interrupts priorities to 0xA0 and set all interrupt targets
         * to be core 0. Higher interrupt priority corresponds to a lower value
         * of the Priority field. Each 32 bit register holds the priority status
         * for 4 interrupt types. Allowing for 8 bits per interrupt type.
         */
        for (u32 n = 0; n < GIC_NUM_INTERRUPTS / 4; ++n) {
            write32 (
            GICD_IPRIORITYRn (n), (GICD_IPRIORITY_DEFAULT_PRIORITY << 0) |
                                  (GICD_IPRIORITY_DEFAULT_PRIORITY << 8) |
                                  (GICD_IPRIORITY_DEFAULT_PRIORITY << 16) |
                                  (GICD_IPRIORITY_DEFAULT_PRIORITY << 24));
            write32 (GICD_ITARGETSRn (n), 0x0);
        }
        /*
         * Set all interrupts to level-triggered.
         * Each GICD_ICFGR register is 32 bits with 2 bits allocated for each
         * interrupt type.
         * 0b00 == level-triggered
         * 0b10 == edge-triggered
         */
        for (u32 n = 0; n < GIC_NUM_INTERRUPTS / 16; ++n) {
            write32 (GICD_ICFGRn (n), GICD_ICFGR_LEVEL_TRIGGERED);
        }

        /* Initialize all handlers to the general handler */
        for (u32 cid = 0; cid < ARM_NUM_CORES; ++cid) {
            for (u32 i = 0; i < GIC_NUM_INTERRUPTS; ++i) {
                irq_handlers_[cid][i] = &gic_general_irq_handler;
                irq_contexts_[cid][i] = NULLPTR;
            }
        }
        write32 (GICD_CTLR, GICD_CTLR_ENABLE);
    }

    /* Allow each core to Enable their GIC CPU Interface */
    write32 (GICC_PMR, GICC_PMR_16_PRIORITY_LEVELS);
    write32 (GICC_CTLR, GICC_CTLR_ENABLE);

    return 1;
}

s32 irq_init (void) {
    s32 status = gic_init ();
    ENABLE_IRQ ();
    return status;
}

void handle_irq (void) {
    u32 irq_ack_reg = read32 (GICC_IAR);
    u32 irq         = irq_ack_reg & 0x3FF;
    if (irq < GIC_NUM_INTERRUPTS) {
        // LOG_DEBUG ("Interrupt %u", irq);
        u32 core_id   = get_arm_core_id ();
        void* context = irq_contexts_[core_id][irq];
        irq_handlers_[get_arm_core_id ()][irq](irq, context);
        write32 (GICC_EOIR, irq_ack_reg);
    } else {
        LOG_ERROR ("Received IRQ ID [%u] outside acceptable range", irq);
    }
}
