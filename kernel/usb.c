#include "usb.h"
#include "irq.h"
#include "peripherals/timer.h"
#include "peripherals/vpu.h"

static usb_status_t power_on_ (void) {
    /*
     * bit 0 = if 1 turn power on
     * bit 1 = if 1 wait for device to power on
     */
    u32 flags = (1 << 0) | (1 << 1);
    VPU_CMB_BUFF_INIT (VPU_MBOX_TAG_SETPOWER, VPU_DEV_ID_USB_HCD, flags);
    vpu_status_t status = vpu_call (&cmd_buffer, VPU_MBOX_CH_PROP);
    u32 dev_state       = cmd_buffer[3];
    if (status != eVPU_STATUS_OK) {
        return eUSB_STATUS_POWERON_FAILED;
    }
    /*
     * bit 0 - 0 = off, 1 = on
     * bit 1 - 0 = device exists, 1 = device does not exist
     */
    if ((dev_state & 0x1) != 0x1) {
        return eUSB_STATUS_POWERON_FAILED;
    } else if ((dev_state & 0x2) == 0x2) {
        return eUSB_STATUS_POWERON_FAILED;
    }
    return eUSB_STATUS_OK;
}

static usb_status_t reset_ (void) {
    s32 max_wait = 100;
    // Wait for AHB master idle state
    while ((read32 (CORE_RESET_REG) & (1 << 31)) == 0x0 && max_wait-- > 0) {
        wait_ms (10);
    }

    if (max_wait <= 0) {
        return eUSB_STATUS_RESET_FAILED;
    }

    max_wait = 100;
    write32 (CORE_RESET_REG, read32 (CORE_RESET_REG) | (1 << 0));
    // wait for soft reset
    while ((read32 (CORE_RESET_REG) & (1 << 0)) == 0x0 && max_wait-- > 0) {
        wait_ms (10);
    }

    if (max_wait <= 0) {
        return eUSB_STATUS_RESET_FAILED;
    }
    wait_ms (100);
    return eUSB_STATUS_OK;
}

static usb_status_t hcd_init_ (void) {
}

static void usb_irq_handler (u32 irq_id) {
}

usb_status_t usb_init (void) {
    u32 vendor_id = read32 (CORE_VENDOR_ID_REG);
    if (vendor_id != 0x4F54280A) {
        LOG_ERROR ("USB vendor id does not match: recvd [%u] != [%u]", vendor_id, 0x4F54280A);
        return eUSB_STATUS_INIT_FAILED;
    }

    if (power_on_ () != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to power on");
        return eUSB_STATUS_POWERON_FAILED;
    }
    // Disable Interrupts for USB
    write32 (CORE_AHB_CFG_REG, read32 (CORE_AHB_CFG_REG) & ~(1 << 0));
    // set interrupt handler for USB
    gic_enable_interrupt (VC_GIC_USB_IRQ, &usb_irq_handler);

    u32 usb_config = read32 (CORE_USB_CFG_REG);
    usb_config &= ~(1 << 20); // ULPI_EXT_VBUS_DRV
    usb_config &= ~(1 << 22); // TERM_SEL_DL_PULSE
    write32 (CORE_USB_CFG_REG, usb_config);

    if (reset_ () != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to reset");
        return eUSB_STATUS_RESET_FAILED;
    }

    usb_config = read32 (CORE_USB_CFG_REG);
    usb_config &= ~(1 << 4); // select UTMI+
    usb_config &= ~(1 << 3); // select UTMI width to be 8
    write32 (CORE_USB_CFG_REG, usb_config);
    usb_config = read32 (CORE_USB_CFG_REG);

    u32 hw_config = read32 (CORE_HW_CFG2_REG);
    if (((hw_config >> 6) & 3) == (1 << 1) && ((hw_config >> 8) & 3) == (1 << 0)) {
        usb_config |= (1 << 17);
        usb_config |= (1 << 19);
    } else {
        usb_config &= ~(1 << 17);
        usb_config &= ~(1 << 19);
    }
    write32 (CORE_USB_CFG_REG, usb_config);

    u32 nchannels = ((hw_config >> 14) & 0xF) + 1;

    u32 ahb_config = read32 (CORE_AHB_CFG_REG);
    ahb_config |= (1 << 5);  // dmaenable
    ahb_config |= (1 << 4);  // wait axi writes
    ahb_config &= ~(3 << 1); // max axi burst mask
    write32 (CORE_AHB_CFG_REG, ahb_config);

    usb_config = read32 (CORE_USB_CFG_REG);
    usb_config &= ~(1 << 9); // disable hnp capable
    usb_config &= ~(1 << 8); // disable srp capable
    write32 (CORE_USB_CFG_REG, usb_config);

    // Clear all pending interrupts
    write32 (CORE_INT_STATUS_REG, 0xFFFFFFFF);
}
