#include "usb.h"
#include "irq.h"
#include "peripherals/timer.h"
#include "peripherals/vpu.h"
#include "sync.h"

typedef enum usb_speed_t {
    eUSB_SPEED_HIGH = 0,
    eUSB_SPEED_FULL = 1,
    eUSB_SPEED_LOW  = 2,
    eUSB_SPEED_UNKNOWN
} usb_speed_t;

typedef enum pid_t { ePID_DATA_0 = 0, ePID_DATA_1 = 2, ePID_SETUP = 3 } pid_t;

typedef enum descriptor_type_t {
    eDESCRIPTOR_TYPE_DEVICE        = 1,
    eDESCRIPTOR_TYPE_CONFIGURATION = 2,
    eDESCRIPTOR_TYPE_STRING        = 3,
    eDESCRIPTOR_TYPE_INTERFACE     = 4,
    eDESCRIPTOR_TYPE_ENDPOINT      = 5
} descriptor_type_t;

typedef struct device_descriptor_t {
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
} device_descriptor_t;

STATIC_ASSERT (sizeof (device_descriptor_t) == 18);

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
    bool is_direction_in;
    endpoint_type_t type;
    u32 max_packet_size;
    u8 number;
    pid_t next_pid;
} endpoint_t;

#define ENDPOINT_INIT()                                             \
    {                                                               \
        .type = eEND_POINT_TYPE_CONTROL, .is_direction_in = false,  \
        .next_pid        = ePID_SETUP,                              \
        .max_packet_size = USB_DEFAULT_MAX_PACKET_SIZE, .number = 0 \
    }

// clang-format off
/*
*************************************************Standard Device Requests****************************************************************
* bmRequestType	bRequest	                wValue	                    wIndex	                wLength	            Data
* 1000 0000b	GET_STATUS (0x00)	        Zero	                    Zero	                Two	                Device Status
* 0000 0000b	CLEAR_FEATURE (0x01)	    Feature Selector	        Zero	                Zero	            None
* 0000 0000b	SET_FEATURE (0x03)	        Feature Selector	        Zero	                Zero	            None
* 0000 0000b	SET_ADDRESS (0x05)	        Device Address	            Zero	                Zero	            None
* 1000 0000b	GET_DESCRIPTOR (0x06)	    Descriptor Type & Index	    Zero or Language ID	    Descriptor Length	Descriptor
* 0000 0000b	SET_DESCRIPTOR (0x07)	    Descriptor Type & Index	    Zero or Language ID	    Descriptor Length	Descriptor
* 1000 0000b	GET_CONFIGURATION (0x08)	Zero	                    Zero	                1	                Configuration Value
* 0000 0000b	SET_CONFIGURATION (0x09)	Configuration Value	        Zero	                Zero	            None
*/
// clang-format on

typedef enum request_type_t {
    eREQUEST_TYPE_OUT = 0,
    eREQUEST_TYPE_IN  = 0x80
} request_type_t;

typedef enum request_code_t {
    eREQUEST_CODE_GET_STATUS    = 0,
    eREQUEST_CODE_CLEAR_FEATURE = 1,
    eREQUEST_CODE_SET_FEATURE   = 3,
    /*
     * Set Address is used during enumeration to assign a unique
     * address to the USB device. The address is specified in wValue
     * and can only be a maximum of 127.
     */
    eREQUEST_CODE_SET_ADDRESS = 5,
    /*
     * Set Descriptor/Get Descriptor is used to return the specified descriptor in wValue.
     * A request for the configuration descriptor will return the device descriptor
     * and all interface and endpoint descriptors in the one request.
     */
    eREQUEST_CODE_GET_DESCRIPTOR    = 6,
    eREQUEST_CODE_SET_CONFIGURATION = 9,
    eREQUEST_CODE_SET_INTERFACE     = 11
} request_code_t;

typedef struct setup_data_t {
    /*
     * bmrequest_type will determine the direction of the request, type of request
     * and designated recipient. brequest_code determines the request being made
     */
    u16 bmrequest_type : 8, brequest_code : 8;
    u16 wvalue;
    u16 windex;
    u16 wlength;
} setup_data_t;

STATIC_ASSERT ((sizeof (setup_data_t) == 8));

typedef struct usb_request_t {
    setup_data_t setup_data;
    endpoint_t* endpoint;
    void* data;
    u16 data_size;
} usb_request_t;

typedef struct stage_data_t {
    usb_request_t* request_data;
    u32 nchannel;
    /*
     * Direction can be IN: device (keyboard/mouse) sends data to the host (computer)
     * Direction can be OUT: host (computer) sends data to the device (keyboard/mouse)
     */
    bool is_dir_in;
    bool status_stage;
    void* transfer_buf;
    u32 transfer_size_nbytes;
    u32 transfer_size_npackets;
    u32 max_packet_size;
    usb_speed_t speed;
    u8 device_address;
    endpoint_type_t endpoint_type;
    u8 endpoint_number;
} stage_data_t;

typedef void (*request_completion_routine) (usb_request_t* req_buff);

// Forward declare
typedef struct hci_root_port_t hci_root_port_t;
typedef struct hci_device_t hci_device_t;

typedef struct usb_device_t {
    endpoint_t endpoint;
    usb_speed_t speed;
    device_descriptor_t desc;
    u8 address;
    u8 hub_address;
    u8 hub_port;
} usb_device_t;

typedef struct hci_root_port_t {
    hci_device_t* host;
    usb_device_t* device;
} hci_root_port_t;

typedef struct hci_device_t {
    hci_root_port_t root_port;
    usb_device_t usb_device;
    u32 nchannels;
    /*
     * 32 channels. if 1 channel is allocated
     */
    u32 volatile allocated_channels;
    bool volatile waiting;
#define MAX_NCHANNELS 16
    stage_data_t stage_data[MAX_NCHANNELS];
} hci_device_t;

#define MAX_NDEVICES 32
static bool volatile allocated_hci_devices_[MAX_NDEVICES];
static hci_device_t hci_devices_[MAX_NDEVICES];
// static usb_info_t usb_info_          = { 0 };
// static hci_device_t hci_root_device_ = { 0 };
// static usb_device_t usb_root_device_ = { 0 };
// static hci_root_port_t hci_root_port_ = { 0 };

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

static usb_status_t host_init_ (hci_device_t* host) {
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

static usb_status_t core_init (hci_device_t* host) {
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

    u32 nchannels   = ((hw_config >> 14) & 0xF) + 1;
    host->nchannels = nchannels;
    ASSERT ((nchannels >= 4 && nchannels <= MAX_NCHANNELS), "USB number of channels out of range [%u]", nchannels);

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

static usb_speed_t hci_get_port_speed (void) {
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

static bool hci_start_channel (hci_device_t* host, stage_data_t* stage_data) {
    u32 chan = stage_data->nchannel;
    // Reset all pending channel interrupts
    u32 interrupts = read32 (HOST_CHAN_INT_REG (chan));
    write32 (HOST_CHAN_INT_REG (chan), interrupts | 0xFFFFFFFF);
    // Set the transfer size in bytes, in number of packets, and the PID
    u32 transfer_info = read32 (HOST_CHAN_TRANSFER_INFO_REG (chan));
    transfer_info |= (stage_data->transfer_size_nbytes & 0x7FFFF);
    transfer_info |= (stage_data->transfer_size_npackets << 19) & (0x3FF << 19);
    transfer_info |= (stage_data->request_data->endpoint->next_pid << 29);
    write32 (HOST_CHAN_TRANSFER_INFO_REG (chan), transfer_info);
    // Convert stage_data's transfer buf address to a VPU address
    // Then give it to the DMA reg
    uintptr_t buf_addr = (uintptr_t)stage_data->transfer_buf;
    write32 (HOST_CHAN_DMA_ADDR_REG (chan), ARM_TO_VPU_BUS_ADDR (buf_addr));
    // Make transfer buffer coherent with DMA
    clean_invalidate_data_cache_vaddr (buf_addr, stage_data->transfer_size_nbytes);

    u32 character = read32 (HOST_CHAN_CHARACTER_REG (chan));
    character &= ~0x7FF;
    character |= (stage_data->max_packet_size & 0x7FF);
    character &= ~(3 << 20);
    character |= (1 << 20);
    // Set direction to IN
    if (stage_data->is_dir_in) {
        character |= (1 << 15);
    }
    // Set direction to OUT
    else {
        character &= ~(1 << 15);
    }
    // Set speed to LOW SPEED
    if (stage_data->speed == eUSB_SPEED_LOW) {
        character |= (1 << 17);
    }
    // Set speed to NOT LOW SPEED
    else {
        character &= ~(1 << 17);
    }
    // Set the device address
    character &= ~(0x7F << 22);
    character |= (stage_data->device_address << 22);
    // Set the end point type for the device
    character &= ~(3 << 18);
    character |= (stage_data->endpoint_type << 18);
    // Set the end point number for the device
    character &= ~(0xF << 11);
    character |= (0xF << 11);

    u32 chan_interrupts = read32 (HOST_CHAN_INT_REG (chan));
    // Clear complete bit
    chan_interrupts |= (1 << 0);
    // Clear halt bit
    chan_interrupts |= (1 << 1);
    // Clear error bits
    chan_interrupts |=
    ((1 << 2) | (1 << 3) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
    write32 (HOST_CHAN_INT_REG (chan), chan_interrupts);
    // Set character enable bit
    character |= (1 << 31);
    // Clear character disable bit
    character &= ~(1 << 30);
    write32 (HOST_CHAN_CHARACTER_REG (chan), character);
}

static void hci_start_transaction (hci_device_t* host, stage_data_t* stage_data) {
    u32 chan = stage_data->nchannel;
    // Check if the channel is enabled
    u32 character = read32 (HOST_CHAN_CHARACTER_REG (chan));
    if (character & (1 << 31)) {
        // Set channel enable to 0
        character &= ~(1 << 31);
        // Disable the channel
        character |= (1 << 30);
        write32 (HOST_CHAN_CHARACTER_REG (chan), character);

        u32 interrupt = HOST_CHAN_INT_REG (chan);
        // Halt any interrupts
        write32 (HOST_CHAN_INT_REG (chan), interrupt | (1 << 1));
    } else {
        hci_start_channel (host, stage_data);
    }
}

static void stage_data_init_ (
hci_device_t const* host,
stage_data_t* stage_data,
usb_request_t* request,
u32 chan,
bool is_dir_in,
bool is_status_stage) {
    usb_device_t const* device  = &host->usb_device;
    stage_data->request_data    = request;
    stage_data->is_dir_in       = is_dir_in;
    stage_data->status_stage    = is_status_stage;
    stage_data->nchannel        = chan;
    stage_data->max_packet_size = USB_DEFAULT_MAX_PACKET_SIZE;
    stage_data->speed           = device->speed;
    stage_data->device_address  = device->address;
    stage_data->endpoint_type   = device->endpoint.type;
    stage_data->endpoint_number = device->endpoint.number;

    if (!is_status_stage) {
        if (request->endpoint->next_pid == ePID_SETUP) {
            stage_data->transfer_size_nbytes = sizeof (setup_data_t);
            stage_data->transfer_buf = &stage_data->request_data->setup_data;
        } else {
            stage_data->transfer_size_nbytes = stage_data->request_data->data_size;
            stage_data->transfer_buf         = stage_data->request_data->data;
        }
    }
    stage_data->transfer_size_npackets =
    (stage_data->transfer_size_nbytes / stage_data->max_packet_size) + 1;
}

static s32 hci_allocate_device_channel (hci_device_t* host) {
    for (u32 i = 0; i < host->nchannels; ++i) {
        if ((host->allocated_channels & (1 << i)) == 0) {
            host->allocated_channels |= (1 << i);
            return i;
        }
    }
    return -1;
}

static bool
hci_transfer_stage (hci_device_t* host, usb_request_t* request, bool is_dir_in, bool status_stage) {

    stage_data_t stage_data = { 0 };
    s32 chan                = hci_allocate_device_channel (host);
    if (chan == -1) {
        return false;
    }
    stage_data_init_ (host, &stage_data, request, chan, is_dir_in, status_stage);
    hci_start_transaction (host, &stage_data);
    return true;
}

static bool hci_submit_blocking_request (hci_device_t* host_device, usb_request_t* request) {
    usb_device_t* device = host_device->root_port.device;
    if (device->endpoint.type == eEND_POINT_TYPE_CONTROL) {
        if (request->setup_data.bmrequest_type & eREQUEST_TYPE_IN) {
            // Send setup packet
            bool status = hci_transfer_stage (host_device, request, false, false);
            // Receive data from device
            status &= hci_transfer_stage (host_device, request, true, false);
            // Send ack to device
            status &= hci_transfer_stage (host_device, request, false, true);
            return status;
        } else {
            if (request->data_size == 0) {
                // Send setup packet
                bool status = hci_transfer_stage (host_device, request, false, false);
                // Receive ack from device
                status &= hci_transfer_stage (host_device, request, true, true);
                return status;
            } else {
                // Send setup packet
                bool status = hci_transfer_stage (host_device, request, false, false);
                // Send data to device
                status &= hci_transfer_stage (host_device, request, false, false);
                // Received ack from device
                status &= hci_transfer_stage (host_device, request, true, true);
                return status;
            }
        }
    } else {
        // Interrupt or Bulk Type Endpoint
        bool status =
        hci_transfer_stage (host_device, request, request->endpoint->is_direction_in, false);
        return status;
    }
}

static s32 hci_control_message (
hci_device_t* host,
request_type_t bmreq_type,
request_code_t breq_code,
u16 wvalue,
u16 windex,
u16 wlength,
void* data) {

    setup_data_t setup_data;
    setup_data.bmrequest_type = bmreq_type;
    setup_data.brequest_code  = breq_code;
    setup_data.wvalue         = wvalue;
    setup_data.windex         = windex;
    setup_data.wlength        = wlength;

    usb_device_t* device = &host->root_port.device;
    usb_request_t request;
    request.endpoint   = &device->endpoint;
    request.data       = data;
    request.data_size  = wlength;
    request.setup_data = setup_data;

    ASSERT (
    (device->endpoint.type == eEND_POINT_TYPE_CONTROL),
    "USB invalid end point type for control msg [%u]", device->endpoint.type);

    s32 result = -1;
    if (hci_submit_blocking_request (host, &request)) {
        result = wlength;
    }
    return result;
}

static s32 hci_get_descriptor (hci_device_t* host_device, void* buffer, u16 buff_size, descriptor_type_t desc_type) {

    return hci_control_message (
    host_device, eREQUEST_TYPE_IN, eREQUEST_CODE_GET_DESCRIPTOR,
    (desc_type << 8) | 0, 0, buff_size, buffer);
}

static usb_status_t
usb_device_init (hci_device_t* host, usb_device_t* device, usb_speed_t speed) {
    endpoint_t endpoint      = ENDPOINT_INIT ();
    device_descriptor_t desc = { 0 };

    device->endpoint    = endpoint;
    device->desc        = desc;
    device->speed       = speed;
    device->address     = 0;
    device->hub_address = 0;
    device->hub_port    = 1;

    s32 data_in =
    hci_get_descriptor (host, &device->desc, sizeof (device_descriptor_t), eDESCRIPTOR_TYPE_DEVICE);
    if (data_in != sizeof (device_descriptor_t)) {
        LOG_ERROR ("USB failed to get descriptor data [%d]", data_in);
        return eUSB_STATUS_ERROR;
    }
    return eUSB_STATUS_OK;
}

usb_status_t
hci_root_port_init (hci_device_t* host, usb_device_t* device, hci_root_port_t* port) {
    port->device = device;
    port->host   = host;

    usb_speed_t speed = hci_get_port_speed ();
    if (speed == eUSB_SPEED_UNKNOWN) {
        return eUSB_STATUS_ERROR;
    }

    usb_status_t status = usb_device_init (host, device, speed);
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to setup root port");
        return status;
    }
    return eUSB_STATUS_OK;
}

static void hci_device_init (hci_device_t* host) {
    if (hci_root_port_init (host, &host->usb_device, &host->root_port) != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to initialize Root Port");
    }
}

usb_status_t hci_init (hci_device_t* host) {
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

    status = core_init (host);
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to init core");
        return status;
    }
    enable_global_interrupts_ ();

    status = host_init_ (host);
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to init core");
        return status;
    }
    LOG_INFO ("USB successfully setup Vendor ID [%X]", vendor_id);

    hci_device_init (host);
    return eUSB_STATUS_OK;
}

hci_device_t* hci_device_create (void) {
    for (u32 i = 0; i < MAX_NDEVICES; ++i) {
        if (allocated_hci_devices_[i] == false) {
            allocated_hci_devices_[i] = true;
            hci_device_t* dev         = &hci_devices_[i];
            memset ((void*)dev, 0, sizeof (hci_device_t));
            return dev;
        }
    }
    return NULLPTR;
}
