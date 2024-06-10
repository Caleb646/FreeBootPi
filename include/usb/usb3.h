#ifndef _USB_3_H
#define _USB_3_H

#include "base.h"
#include "usb/pcie_hostbridge.h"

typedef struct mmio_space_t {
    uintptr_t base;     // Capability registers
    uintptr_t op_base;  // Operational registers
    uintptr_t db_base;  // Doorbell registers
    uintptr_t rt_base;  // Runtime registers
    uintptr_t pt_base;  // Port register set
    uintptr_t ecp_base; // Extended capabilities
    u32 hcx_params[4];  // Capability cache
} mmio_space_t;

typedef struct xhci_device_t {
    pcie_hostbridge_t* ppcie;
    mmio_space_t* pmmio;
    bool is_shutdown;
} xhci_device_t;

xhci_device_t* xhci_device_create (void);
bool xhci_device_init (xhci_device_t*);

#endif /*_USB_3_H */