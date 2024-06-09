#ifndef _USB_H
#define _USB_H

#include "base.h"

#define USB_CORE_BASE                  (PERIPH_BASE + 0x02980000)
#define USB_HOST_BASE                  (USB_CORE_BASE + 0x400)
#define USB_DEV_BASE                   (USB_CORE_BASE + 0x800)
#define USB_POWER                      (USB_CORE_BASE + 0xE00)

#define CORE_OTG_CTRL_REG              (USB_CORE_BASE + 0x000)
#define CORE_OTG_INT_REG               (USB_CORE_BASE + 0x004)
/*
 * Bit 0 = interrupt enable
 */
#define CORE_AHB_CFG_REG               (USB_CORE_BASE + 0x008)
#define CORE_USB_CFG_REG               (USB_CORE_BASE + 0x00C)
#define CORE_RESET_REG                 (USB_CORE_BASE + 0x010)
#define CORE_INT_STATUS_REG            (USB_CORE_BASE + 0x014)
#define CORE_INT_MASK_REG              (USB_CORE_BASE + 0x018)
#define CORE_RX_FIFO_SIZE_REG          (USB_CORE_BASE + 0x024)
#define CORE_NPER_TX_FIFO_SIZE_REG     (USB_CORE_BASE + 0x028)
#define CORE_VENDOR_ID_REG             (USB_CORE_BASE + 0x040)
#define CORE_HW_CFG2_REG               (USB_CORE_BASE + 0x048)
#define CORE_HOST_PER_TX_FIFO_SIZE_REG (USB_CORE_BASE + 0x100)

#define HOST_CFG_REG                   (USB_HOST_BASE + 0x000)
#define HOST_FRM_NUM                   (USB_HOST_BASE + 0x008)
#define HOST_ALLCHAN_INT_STATUS_REG    (USB_HOST_BASE + 0x014)
#define HOST_ALLCHAN_INT_MASK_REG      (USB_HOST_BASE + 0x018)
#define HOST_PORT_REG                  (USB_HOST_BASE + 0x040)
#define HOST_CHAN_CHARACTER_REG(chan_id) \
    (USB_HOST_BASE + 0x100 + (chan_id * 0x20))
#define HOST_CHAN_SPLIT_CTRL_REG(chan_id) \
    (USB_HOST_BASE + 0x104 + (chan_id * 0x20))
#define HOST_CHAN_INT_REG(chan_id) (USB_HOST_BASE + 0x108 + (chan_id * 0x20))
/*
 * INT_AHB_ERROR | STALL | XACT_ERROR | BABBLE_ERROR | FRAME_OVERRUN | DATA_TOGGLE_ERROR
 */
#define HOST_CHAN_INT_ERROR_MASK \
    ((1 << 2) | (1 << 3) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10))
#define HOST_CHAN_INT_NAK_BIT  (1 << 4)
#define HOST_CHAN_INT_NYET_BIT (1 << 6)

#define HOST_CHAN_INT_MASK_REG(chan_id) \
    (USB_HOST_BASE + 0x10C + (chan_id * 0x20))
#define HOST_CHAN_TRANSFER_INFO_REG(chan_id) \
    (USB_HOST_BASE + 0x110 + (chan_id * 0x20))
#define HOST_CHAN_DMA_ADDR_REG(chan_id) \
    (USB_HOST_BASE + 0x114 + (chan_id * 0x20))


#ifndef __ASSEMBLER__

typedef enum usb_status_t {
    eUSB_STATUS_OK = 1,
    eUSB_STATUS_ERROR,
    eUSB_STATUS_INIT_FAILED,
    eUSB_STATUS_POWERON_FAILED,
    eUSB_STATUS_RESET_FAILED,
    eUSB_STATUS_ROOTPORTINIT_FAILED
} usb_status_t;

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
    eEND_POINT_TYPE_ISOCHRONOUS = 1,
    eEND_POINT_TYPE_BULK        = 2,
    eEND_POINT_TYPE_INTERRUPT   = 3
} endpoint_type_t;

typedef struct endpoint_t {
    bool is_direction_in;
    endpoint_type_t type;
    u32 max_packet_size;
    u8 number;
    pid_t pid;
} endpoint_t;

/* #define ENDPOINT_INIT()                                             \
//     {                                                               \
//         .type = eEND_POINT_TYPE_CONTROL, .is_direction_in = false,  \
//         .next_pid        = ePID_SETUP,                              \
//         .max_packet_size = USB_DEFAULT_MAX_PACKET_SIZE, .number = 0 \
//     }*/

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
    endpoint_t* pendpoint;
    void* pdata;
    u16 data_size;
} usb_request_t;

// Forward declare
typedef struct hci_root_port_t hci_root_port_t;
typedef struct hci_device_t hci_device_t;
typedef struct usb_device_t usb_device_t;

typedef enum stage_state_t {
    eSTAGE_STATE_NO_SPLIT_TRANSFER,
    eSTAGE_STATE_START_SPLIT,
    eSTAGE_STATE_COMPLETE_SPLIT,
    eSTAGE_STATE_PERIODIC_DELAY,
    eSTAGE_STATE_UNKNOWN
} stage_state_t;

typedef enum stage_sub_state_t {
    eSTAGE_SUB_STATE_WAIT_FOR_CHAN_DISABLE,
    eSTAGE_SUB_STATE_WAIT_FOR_TRANSACTION_COMPLETE,
    eSTAGE_SUB_STATE_UNKNOWN
} stage_sub_state_t;

typedef struct stage_data_t {
    usb_device_t* pdevice;
    endpoint_t* pendpoint;
    usb_request_t* prequest;
    void* ptransfer_buf;
    u32 total_tsize_nbytes;
    u32 total_tsize_npackets;
    u32 nbytes_per_transaction;
    u32 npackets_per_transaction;
    u32 total_bytes_sent;
    u32 total_packets_sent;
    u32 max_packet_size;
    usb_speed_t speed;
    u32 dev_addr;
    u32 nchannel;
    /*
     * Direction can be IN: device (keyboard/mouse) sends data to the host (computer)
     * Direction can be OUT: host (computer) sends data to the device (keyboard/mouse)
     */
    bool is_dir_in;
    bool status_stage;
    stage_state_t state;
    stage_sub_state_t sub_state;
} stage_data_t;

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
    bool is_plugnplay;
    bool is_rootport_enabled;
    u32 nchannels;
    /*
     * 32 channels. if 1 channel is allocated
     */
    u32 volatile allocated_channels;
    bool volatile waiting;
#define MAX_NCHANNELS 16
    stage_data_t stage_data[MAX_NCHANNELS];
} hci_device_t;


usb_status_t hci_init (hci_device_t* host);
// usb_status_t hcd_init_ (void);
// usb_status_t power_on_ (void);

#endif // __ASSEMBLER__

#endif /*_USB_H */