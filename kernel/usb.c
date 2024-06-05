#include "usb.h"
#include "irq.h"
#include "peripherals/timer.h"
#include "peripherals/vpu.h"

typedef enum usb_speed_t {
    eUSB_SPEED_HIGH = 0,
    eUSB_SPEED_FULL = 1,
    eUSB_SPEED_LOW  = 2,
    eUSB_SPEED_UNKNOWN
} usb_speed_t;

typedef enum request_type_t {
    eREQUEST_TYPE_OUT = 0,
    eREQUEST_TYPE_IN  = 0x80
} request_type_t;

typedef enum request_code_t {
    eREQUEST_CODE_GET_STATUS        = 0,
    eREQUEST_CODE_CLEAR_FEATURE     = 1,
    eREQUEST_CODE_SET_FEATURE       = 3,
    eREQUEST_CODE_SET_ADDRESS       = 5,
    eREQUEST_CODE_GET_DESCRIPTOR    = 6,
    eREQUEST_CODE_SET_CONFIGURATION = 9,
    eREQUEST_CODE_SET_INTERFACE     = 11
} request_code_t;

typedef enum descriptor_type_t {
    eDESCRIPTOR_TYPE_DEVICE        = 1,
    eDESCRIPTOR_TYPE_CONFIGURATION = 2,
    eDESCRIPTOR_TYPE_STRING        = 3,
    eDESCRIPTOR_TYPE_INTERFACE     = 4,
    eDESCRIPTOR_TYPE_ENDPOINT      = 5
} descriptor_type_t;

typedef struct usb_info_t {
    u32 nchannels;
} usb_info_t;

typedef enum endpoint_type_t {
    eEND_POINT_TYPE_CONTROL     = 0,
    eEND_POINT_TYPE_BULK        = 1,
    eEND_POINT_TYPE_INTERRUPT   = 2,
    eEND_POINT_TYPE_ISOCHRONOUS = 3,
} endpoint_type_t;

typedef struct endpoint_t {
    endpoint_type_t endpoint_type;
} endpoint_t;

typedef struct usb_request_t {

} usb_request_t;

typedef void (*request_completion_routine) (usb_request_t* req_buff);

#define ENDPOINT_INIT() \
    { .endpoint_type = eEND_POINT_TYPE_CONTROL }

typedef struct usb_device_descriptor_t {
    u16 length : 8, descriptor_type : 8;
    u16 bcd_usb;
    u16 device_class : 8, device_subclass : 8;
    u16 device_protocol : 8, max_packed_size : 8;
#define USB_DEFAULT_MAX_PACKET_SIZE 8
    u16 idVendor;
    u16 idProduct;
    u16 bcdDevice;
    u16 manufacturer : 8, product : 8;
    u16 serial_number : 8, num_configs : 8;
} usb_device_descriptor_t;

STATIC_ASSERT (sizeof (usb_device_descriptor_t) == 18);

typedef struct usb_device_t {
    endpoint_t endpoint;
    usb_speed_t speed;
} usb_device_t;


typedef struct root_port_t;

typedef struct hci_device_t {
    root_port_t* root_port;
    u32 nchannels;
    u32 volatile allocated_channels;
} hci_device_t;

typedef struct root_port_t {
    hci_device_t* host;
    usb_device_t* device;
} root_port_t;

static usb_info_t usb_info_   = { 0 };
static root_port_t root_port_ = { 0 };

static usb_status_t power_on_ (void) {
    /*
     * bit 0 = if 1 turn power on
     * bit 1 = if 1 wait for device to power on
     */
    VPU_CMB_BUFF_INIT (VPU_MBOX_TAG_SETPOWER, 8, 8, VPU_DEV_ID_USB_HCD, (1 << 0) | (1 << 1));
    vpu_status_t status = vpu_call (&cmd_buffer, VPU_MBOX_CH_PROP);
    u32 dev_state       = cmd_buffer[6];
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
    while ((read32 (CORE_RESET_REG) & (1 << 31)) == 0x0) {
        wait_ms (10);
        if (max_wait-- <= 0) {
            return eUSB_STATUS_RESET_FAILED;
        }
    }
    max_wait = 100;
    write32 (CORE_RESET_REG, read32 (CORE_RESET_REG) | (1 << 0));
    // wait for soft reset
    // while ((read32 (CORE_RESET_REG) & (1 << 0)) == 0x0) {
    //     LOG_INFO ("USB waiting %X", read32 (CORE_RESET_REG));
    //     wait_ms (100);
    //     if (max_wait-- <= 0) {
    //         return eUSB_STATUS_RESET_FAILED;
    //     }
    // }
    wait_ms (100);
    return eUSB_STATUS_OK;
}

static void flush_tx_fifo_ (u32 nfifo) {
    u32 reset = 0;
    reset |= (1 << 5);     // RESET_TX_FIFO_FLUSH
    reset |= (nfifo << 6); // TX_FIFO_NUM_MASK
    reset &= ~(0x1F << 6); // RESET_TX_FIFO_NUM_MASK
    write32 (CORE_RESET_REG, reset);
    s32 max_wait = 10;
    /*
     * RESET_TX_FIFO_FLUSH (1 << 0)
     */
    while ((read32 (CORE_RESET_REG) & (1 << 0)) == 0x0) {
        wait_ms (10);
        if (max_wait-- <= 0) {
            LOG_ERROR ("USB failed to flush TX FIFO");
        }
    }
    wait_us (1); // Wait for 3 PHY clocks
}

static void flush_rx_fifo_ (void) {
    u32 reset = 0;
    reset |= (1 << 4);
    write32 (CORE_RESET_REG, reset);
    s32 max_wait = 10;
    /*
     * RESET_RX_FIFO_FLUSH (1 << 4)
     */
    while ((read32 (CORE_RESET_REG) & (1 << 4)) == 0x0) {
        wait_ms (10);
        if (max_wait-- <= 0) {
            LOG_ERROR ("USB failed to flush RX FIFO");
        }
    }
    wait_us (1); // Wait for 3 PHY clocks
}

static void enable_global_interrupts_ (void) {
    u32 ahb = read32 (CORE_AHB_CFG_REG);
    ahb |= (1 << 0); // GLOBAL INT
    write32 (CORE_AHB_CFG_REG, ahb);
}

static void enable_common_interrupts_ (void) {
    // Clear all pending interrupts
    write32 (CORE_INT_STATUS_REG, 0xFFFFFFFF);
}

static void enable_host_interrupts_ (void) {
    u32 int_mask = 0;
    // Disable all interrupts
    write32 (CORE_INT_STATUS_REG, int_mask);

    enable_common_interrupts_ ();

    int_mask = read32 (CORE_INT_STATUS_REG);
    int_mask |= (1 << 25); // HC_INTR
    int_mask |= (1 << 24); // PORT_INTR
    int_mask |= (1 << 29); // DISCONNECT_INTR
    write32 (CORE_INT_STATUS_REG, int_mask);
}

static usb_status_t host_init_ (void) {
    // Restart the physical clock
    write32 (USB_POWER, 0x0);

    u32 host_config = read32 (HOST_CFG_REG);
    host_config &= ~(3 << 0); // FSLS PCLK SEL Mask

    u32 usb_config = read32 (CORE_USB_CFG_REG);
    u32 hw_config  = read32 (CORE_HW_CFG2_REG);
    if (((hw_config >> 6) & 3) == (1 << 1) && ((hw_config >> 8) & 3) == (1 << 0) && (usb_config & (1 << 17))) {
        host_config |= (1 << 0); // FSLS_PCLK_SEL_48_MHZ
    } else {
        host_config |= (0 << 0); // FSLS_PCLK_SEL_30_60_MHZ
    }
    write32 (HOST_CFG_REG, host_config);
    // Flush all TX FIFOs
    flush_tx_fifo_ (0x10);
    flush_rx_fifo_ ();

    u32 host_port = read32 (HOST_PORT_REG);
    host_port &= ~(1 << 1); // PORT_CONNECT_CHANGED
    host_port &= ~(1 << 2); // PORT_ENABLE
    host_port &= ~(1 << 3); // ENABLE_CHANGED
    host_port &= ~(1 << 5); // OVERCURRENT_CHANGED
    /*
     * PORT_POWER (1 << 12)
     */
    if (!(host_port & (1 << 12))) {
        host_port |= (1 << 12);
        write32 (HOST_PORT_REG, host_port);
    }
    enable_host_interrupts_ ();
    return eUSB_STATUS_OK;
}

static usb_status_t core_init (void) {
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

    u32 nchannels       = ((hw_config >> 14) & 0xF) + 1;
    usb_info_.nchannels = nchannels;

    u32 ahb_config = read32 (CORE_AHB_CFG_REG);
    ahb_config |= (1 << 5);  // dmaenable
    ahb_config |= (1 << 4);  // wait axi writes
    ahb_config &= ~(3 << 1); // max axi burst mask
    write32 (CORE_AHB_CFG_REG, ahb_config);

    usb_config = read32 (CORE_USB_CFG_REG);
    usb_config &= ~(1 << 9); // disable hnp capable
    usb_config &= ~(1 << 8); // disable srp capable
    write32 (CORE_USB_CFG_REG, usb_config);

    enable_common_interrupts_ ();
    return eUSB_STATUS_OK;
}

static void usb_irq_handler_ (u32 irq_id) {
}

usb_status_t usb_init (void) {
    usb_status_t status = eUSB_STATUS_OK;
    u32 vendor_id       = read32 (CORE_VENDOR_ID_REG);
    if (vendor_id != 0x4F54280A) {
        LOG_ERROR ("USB vendor id does not match: recvd [%X] != [%X]", vendor_id, 0x4F54280A);
        return eUSB_STATUS_INIT_FAILED;
    }
    status = power_on_ ();
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to power on");
        return status;
    }
    // Disable Interrupts for USB
    write32 (CORE_AHB_CFG_REG, read32 (CORE_AHB_CFG_REG) & ~(1 << 0));
    // set interrupt handler for USB
    gic_enable_interrupt (VC_GIC_USB_IRQ, &usb_irq_handler_);

    status = core_init ();
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to init core");
        return status;
    }
    enable_global_interrupts_ ();

    status = host_init_ ();
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to init core");
        return status;
    }
    LOG_INFO ("USB successfully setup Vendor ID [%X]", vendor_id);
    return eUSB_STATUS_OK;
}

static usb_speed_t usb_get_port_speed (void) {
    u32 host_port = read32 (HOST_PORT_REG);
    u32 speed     = ((read32 (HOST_PORT_REG)) >> 17) & 3;
    switch (speed) {
    case eUSB_SPEED_HIGH: return eUSB_SPEED_HIGH;
    case eUSB_SPEED_FULL: return eUSB_SPEED_FULL;
    case eUSB_SPEED_LOW: return eUSB_SPEED_LOW;
    default:
        LOG_ERROR ("USB Speed [0x%X] is unknown. Host Port [0x%X]", speed, host_port);
        return eUSB_SPEED_UNKNOWN;
    }
}

static usb_status_t hci_submit_blocking_request (usb_device_t* device) {

    if (device->endpoint.endpoint_type == eEND_POINT_TYPE_CONTROL) {
    }
}

static s32 hci_control_message (
usb_device_t* device,
request_type_t req_type,
request_code_t req_code,
u16 value,
u16 index,
void* data,
u16 data_size) {
}

static s32 hci_get_descriptor (
usb_device_t* device,
void* buffer,
u32 buff_size,
descriptor_type_t desc_type,
usb_device_descriptor_t* desc) {

    return hci_control_message (
    device, eREQUEST_TYPE_IN, eREQUEST_CODE_GET_DESCRIPTOR,
    (desc_type << 8) | 0, 0, buffer, buff_size);
}

static usb_status_t
usb_device_init (hci_device_t* host_device, usb_speed_t speed, usb_device_t* device) {
    endpoint_t endpoint = ENDPOINT_INIT ();
    device->endpoint    = endpoint;

    usb_device_descriptor_t desc = { 0 };

    return eUSB_STATUS_OK;
}

usb_status_t usb_root_init (hci_device_t* host_device) {
    usb_device_t root_device;

    usb_speed_t speed = usb_get_port_speed ();
    if (speed == eUSB_SPEED_UNKNOWN) {
        return eUSB_STATUS_ERROR;
    }

    usb_status_t status = usb_device_init (host_device, speed, &root_device);
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to setup root port");
        return status;
    }
}
