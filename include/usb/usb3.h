#ifndef _USB_3_H
#define _USB_3_H

#include "base.h"
#include "usb/pcie_hostbridge.h"

#define XHCI_CONFIG_MAX_PORTS 3
#define XHCI_MAX_ENDPOINTS    31
#define XHCI_CONFIG_MAX_SLOTS 64
#define XHCI_PAGE_SIZE        4096

#define XHCI_PCI_CLASS_CODE   0xC0330
#define XHCI_PCIE_SLOT        0
#define XHCI_PCIE_FUNC        0

// typedef struct xhci_slot_manager_t;
// typedef struct xhci_roothub_t;
// typedef struct xhci_device_t;
// typedef struct xhci_usb_device_t;

typedef enum usb_speed_t {
    eUSB_SPEED_HIGH = 0,
    eUSB_SPEED_FULL = 1,
    eUSB_SPEED_LOW  = 2,
    eUSB_SPEED_UNKNOWN
} usb_speed_t;

typedef struct mmio_space_t {
    uintptr_t base;     // Capability registers
    uintptr_t op_base;  // Operational registers
    uintptr_t db_base;  // Doorbell registers
    uintptr_t rt_base;  // Runtime registers
    uintptr_t pt_base;  // Port register set
    uintptr_t ecp_base; // Extended capabilities
    u32 hcx_params[4];  // Capability cache
} mmio_space_t;

typedef struct xhci_usb_device_t {
    u8 slot_id;
} xhci_usb_device_t;

typedef struct xhci_slot_manager_t {
    u64* pdcbaa;
    xhci_usb_device_t* pusb_devs[XHCI_CONFIG_MAX_SLOTS];

} xhci_slot_manager_t;

typedef struct xhci_erst_entry_t {
    u64 ring_segment_base;
#define XHCI_ERST_ENTRY_RING_SEGMENT_BASE_LO__MASK 0xFFFFFFC0
    u32 ring_segment_size;
#define XHCI_ERST_ENTRY_RING_SEGMENT_SIZE__MASK 0xFFFF
    u32 reserved;
} xhci_erst_entry_t;

STATIC_ASSERT (sizeof (xhci_erst_entry_t) == 16);

typedef enum xhci_ring_type_t {
    eXHCI_RING_TYPE_TRANSFER,
    eXHCI_RING_TYPE_EVENT,
    eXHCI_RING_TYPE_COMMAND,
    eXHCI_RING_TYPE_UNKNOWN
} xhci_ring_type_t;

typedef struct xhci_trb_t {
    union {
        struct {
            u32 parameter1;
            u32 parameter2;
        };
        u64 parameter;
    };

    u32 status;
    u32 control;
#define XHCI_TRB_CONTROL_C               (1 << 0)
#define XHCI_TRB_CONTROL_TRB_TYPE__SHIFT 10
#define XHCI_TRB_CONTROL_TRB_TYPE__MASK  (0x3F << 10)
} xhci_trb_t;

STATIC_ASSERT (sizeof (xhci_trb_t) == 16);

typedef struct xhci_ring_t {
    xhci_trb_t* pfirst_trb;
    u32 trb_count;
    u32 enqueue_idx;
    u32 dequeue_idx;
    xhci_ring_type_t type;
    u32 cycle_state;
} xhci_ring_t;

typedef struct xhci_event_manager_t {
    xhci_erst_entry_t* perst;
    xhci_ring_t event_ring;
} xhci_event_manager_t;

typedef struct xhci_command_manager_t {
    xhci_trb_t* pcur_cmd_trb;
    xhci_ring_t cmd_ring;
    bool volatile is_cmd_completed;
    u8 completion_code;
    u8 slot_id;
} xhci_command_manager_t;

typedef struct xhci_endpoint_t {
    u32 for_compiler;
} xhci_endpoint_t;

typedef struct xhci_rootport_t {
    u32 port_idx;
    xhci_usb_device_t* pusb_dev;
} xhci_rootport_t;

typedef struct xhci_roothub_t {
    u32 nports;
    xhci_rootport_t rootports[XHCI_CONFIG_MAX_PORTS];
} xhci_roothub_t;

typedef struct xhci_device_t {
    pcie_hostbridge_t* ppcie;
    mmio_space_t* pmmio;
    xhci_roothub_t* proothub;
    xhci_slot_manager_t* pslot_man;
    xhci_event_manager_t* pevent_man;
    xhci_command_manager_t* pcommand_man;
    void* pscratchpad_buffers;
    u64* pscratchpad_buffer_array;
    bool is_shutdown;
    bool is_interrupt_connected;
} xhci_device_t;

xhci_device_t* xhci_device_create (void);
bool xhci_device_init (xhci_device_t*);

#endif /*_USB_3_H */