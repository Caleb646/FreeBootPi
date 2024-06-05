#ifndef _USB_H
#define _USB_H

#include "base.h"

#define USB_CORE_BASE       (PERIPH_BASE + 0x02980000)
#define USB_HOST_BASE       (USB_CORE_BASE + 0x400)
#define USB_DEV_BASE        (USB_CORE_BASE + 0x800)
#define USB_POWER           (USB_CORE_BASE + 0xE00)

#define CORE_OTG_CTRL_REG   (USB_CORE_BASE + 0x000)
#define CORE_OTG_INT_REG    (USB_CORE_BASE + 0x004)
/*
 * Bit 0 = interrupt enable
 */
#define CORE_AHB_CFG_REG    (USB_CORE_BASE + 0x008)
#define CORE_USB_CFG_REG    (USB_CORE_BASE + 0x00C)
#define CORE_RESET_REG      (USB_CORE_BASE + 0x010)
#define CORE_INT_STATUS_REG (USB_CORE_BASE + 0x014)
#define CORE_VENDOR_ID_REG  (USB_CORE_BASE + 0x040)
#define CORE_HW_CFG2_REG    (USB_CORE_BASE + 0x048)

#define HOST_CFG_REG        (USB_HOST_BASE + 0x000)
#define HOST_PORT_REG       (USB_HOST_BASE + 0x040)

#ifndef __ASSEMBLER__

typedef enum usb_status_t {
    eUSB_STATUS_OK = 1,
    eUSB_STATUS_ERROR,
    eUSB_STATUS_INIT_FAILED,
    eUSB_STATUS_POWERON_FAILED,
    eUSB_STATUS_RESET_FAILED
} usb_status_t;


usb_status_t usb_init (void);
// usb_status_t hcd_init_ (void);
// usb_status_t power_on_ (void);

#endif // __ASSEMBLER__

#endif /*_USB_H */