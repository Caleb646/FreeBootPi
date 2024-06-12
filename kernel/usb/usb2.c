#include "usb/usb2.h"
#include "irq.h"
#include "peripherals/timer.h"
#include "peripherals/vpu.h"
#include "sync.h"

typedef void (*request_completion_routine) (hci_device_t* host, u32 chan);

#define MAX_NDEVICES 32
// static bool volatile allocated_hci_devices_[MAX_NDEVICES];
// static hci_device_t hci_devices_[MAX_NDEVICES];
// static hci_device_t hci_host_device_;
/*
 * DMA ring buffer for sending and receiving packets
 */
// static u8 GCC_CACHE_ALIGNED dma_ring_buf_[CACHE_LINE_NBYTES * MAX_NCHANNELS * 4];
// static u8* cur_ptr_dma_ring_buf_ = NULLPTR;
// static uintptr_t start_dma_ring_buf_;
// static uintptr_t end_dma_ring_buf_;
// STATIC_ASSERT ((sizeof (dma_ring_buf_) % CACHE_LINE_NBYTES == 0));
// static usb_info_t usb_info_          = { 0 };
// static hci_device_t hci_root_device_ = { 0 };
// static usb_device_t usb_root_device_ = { 0 };
// static hci_root_port_t hci_root_port_ = { 0 };

// static void* allocate_dma_buffer_ (u64 sz) {
//     if (cur_ptr_dma_ring_buf_ == NULLPTR) {
//         cur_ptr_dma_ring_buf_ = &dma_ring_buf_[0];
//         start_dma_ring_buf_   = (uintptr_t)&dma_ring_buf_[0];
//         end_dma_ring_buf_ = (uintptr_t)&dma_ring_buf_[sizeof (dma_ring_buf_) - 1];
//     }
//     u64 mod        = sz % CACHE_LINE_NBYTES;
//     u64 aligned_sz = sz + (sz * (mod != 0) - mod);
//     ASSERT (
//     (end_dma_ring_buf_ - start_dma_ring_buf_ > aligned_sz),
//     "USB failed to allocate buffer with cache aligned size [%u]", aligned_sz);

//     if ((uintptr_t)cur_ptr_dma_ring_buf_ + aligned_sz > end_dma_ring_buf_) {
//         cur_ptr_dma_ring_buf_ = &dma_ring_buf_[0];
//     }
//     void* ret = cur_ptr_dma_ring_buf_;
//     cur_ptr_dma_ring_buf_ += aligned_sz;
//     return ret;
// }

static bool wait_for_bit_ (u64 reg_addr, u32 mask, bool wait_until_set, u32 ms_timeout) {
    while ((read32 (reg_addr) & mask) ? !wait_until_set : wait_until_set) {
        wait_ms (1);
        if (--ms_timeout <= 0) {
            return false;
        }
    }
    return true;
}

static bool enable_root_port_ (hci_device_t* host) {
    if (wait_for_bit_ (HOST_PORT_REG, (1 << 0), true, 2000) == false) {
        LOG_ERROR ("[0x%X]", read32 (HOST_PORT_REG));
        return false;
    }
    wait_ms (100); // USB 2.0 spec

    u32 host_port = read32 (HOST_PORT_REG);
    // PORT_CONNECT_CHANGED | PORT_ENABLE | PORT_ENABLE_CHANGED | PORT_OVERCURRENT_CHANGED
    host_port &= ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 5));
    // Reset the port
    host_port |= (1 << 8);
    write32 (HOST_PORT_REG, host_port);

    wait_ms (50); // USB 2.0 spec (tDRSTR)

    host_port = read32 (HOST_PORT_REG);
    // PORT_CONNECT_CHANGED | PORT_ENABLE | PORT_ENABLE_CHANGED | PORT_OVERCURRENT_CHANGED
    host_port &= ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 5));
    host_port &= ~(1 << 8);
    write32 (HOST_PORT_REG, host_port);

    wait_ms (20); // USB 2.0 spec (tRSTRCY)
    return true;
}

// static void completion_routine (hci_device_t* host, u32 chan) {
//     host->waiting = false;
// }

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
    bool failed = (dev_state & 0x1) != 0x1;
    failed |= (dev_state & 0x2) == 0x2;
    if (failed) {
        return eUSB_STATUS_POWERON_FAILED;
    }
    return eUSB_STATUS_OK;
}

static usb_status_t reset_ (void) {
    // Wait for AHB master idle state
    if (wait_for_bit_ (CORE_RESET_REG, (1 << 31), true, 100) == false) {
        LOG_ERROR ("USB failed to reset AHB");
        return eUSB_STATUS_RESET_FAILED;
    }
    write32 (CORE_RESET_REG, read32 (CORE_RESET_REG) | (1 << 0));
    // wait for soft reset
    if (wait_for_bit_ (CORE_RESET_REG, (1 << 0), false, 10) == false) {
        LOG_ERROR ("USB failed to soft reset");
        return eUSB_STATUS_RESET_FAILED;
    }
    wait_ms (100);
    return eUSB_STATUS_OK;
}

static void flush_tx_fifo_ (u32 nfifo) {
    u32 reset = (1 << 5);  // RESET_TX_FIFO_FLUSH
    reset |= (nfifo << 6); // TX_FIFO_NUM_MASK
    reset &= ~(0x1F << 6); // RESET_TX_FIFO_NUM_MASK
    write32 (CORE_RESET_REG, reset);
    /*
     * RESET_TX_FIFO_FLUSH (1 << 0)
     */
    if (wait_for_bit_ (CORE_RESET_REG, (1 << 5), false, 10) == false) {
        LOG_ERROR ("USB failed to flush TX FIFO");
        return;
    }
    wait_us (1); // Wait for 3 PHY clocks
}

static void flush_rx_fifo_ (void) {
    u32 reset = (1 << 4); // RX_FIFO_FLUSH
    write32 (CORE_RESET_REG, reset);
    /*
     * RESET_RX_FIFO_FLUSH (1 << 4)
     */
    if (wait_for_bit_ (CORE_RESET_REG, (1 << 4), false, 10) == false) {
        LOG_ERROR ("USB failed to flush RX FIFO");
        return;
    }
    wait_us (1); // Wait for 3 PHY clocks
}

static void enable_channel_interrupts (u32 chan) {
    u32 all_chan_interrupt = read32 (HOST_ALLCHAN_INT_MASK_REG);
    all_chan_interrupt |= (1 << chan);
    write32 (HOST_ALLCHAN_INT_MASK_REG, all_chan_interrupt);
}

// static void disable_channel_interrupts (u32 chan) {
//     u32 all_chan_interrupt = read32 (HOST_ALLCHAN_INT_MASK_REG);
//     all_chan_interrupt &= ~(1 << chan);
//     write32 (HOST_ALLCHAN_INT_MASK_REG, all_chan_interrupt);
// }

static void enable_global_interrupts_ (void) {
    u32 ahb = read32 (CORE_AHB_CFG_REG);
    ahb |= (1 << 0); // GLOBAL INT
    write32 (CORE_AHB_CFG_REG, ahb);
}

static void enable_common_interrupts_ (void) {
    // Clear all pending interrupts
    write32 (CORE_INT_STATUS_REG, U32_MAX);
}

static void enable_host_interrupts_ (void) {
    // Disable all interrupts
    write32 (CORE_INT_MASK_REG, 0);

    enable_common_interrupts_ ();

    u32 int_mask = read32 (CORE_INT_MASK_REG);
    int_mask |= (1 << 25); // HC_INTR
    int_mask |= (1 << 24); // PORT_INTR
    int_mask |= (1 << 29); // DISCONNECT_INTR
    write32 (CORE_INT_MASK_REG, int_mask);
}

static usb_status_t host_init_ (hci_device_t* host) {
    // Restart the physical clock
    write32 (USB_POWER, 0x0);

    u32 host_config = read32 (HOST_CFG_REG);
    host_config &= ~(3 << 0); // FSLS PCLK SEL Mask

    u32 hw_config  = read32 (CORE_HW_CFG2_REG);
    u32 usb_config = read32 (CORE_USB_CFG_REG);
    if (((hw_config >> 6) & 3) == (1 << 1) && ((hw_config >> 8) & 3) == (1 << 0) && (usb_config & (1 << 17))) {
        host_config |= (1 << 0); // FSLS_PCLK_SEL_48_MHZ
    } else {
        host_config |= (0 << 0); // FSLS_PCLK_SEL_30_60_MHZ
    }
    write32 (HOST_CFG_REG, host_config);
    /*
     * Configure the dynamic FIFO buffers
     */
    write32 (CORE_RX_FIFO_SIZE_REG, 1024);
    write32 (CORE_NPER_TX_FIFO_SIZE_REG, 1024 | (1024 << 16));
    write32 (CORE_HOST_PER_TX_FIFO_SIZE_REG, 2048 | (1024 << 16));

    // Flush all TX FIFOs
    flush_tx_fifo_ (16);
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
    /*
     * Set full speed. CFG_PHY_SEL_FS
     */
    usb_config |= (1 << 6);
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


    u32 hw_config = read32 (CORE_HW_CFG2_REG);
    ASSERT ((((hw_config >> 3) & 3) == 2), "USB invalid hardware config");

    usb_config = read32 (CORE_USB_CFG_REG);
    if (((hw_config >> 6) & 3) == (1 << 1) && ((hw_config >> 8) & 3) == (1 << 0)) {
        usb_config |= (1 << 17);
        usb_config |= (1 << 19);
    } else {
        usb_config &= ~(1 << 17);
        usb_config &= ~(1 << 19);
    }
    write32 (CORE_USB_CFG_REG, usb_config);

    host->nchannels = ((hw_config >> 14) & 0xF) + 1;
    LOG_INFO ("USB number of channels [%u]", host->nchannels);
    ASSERT (
    (host->nchannels >= 4 && host->nchannels <= MAX_NCHANNELS),
    "USB number of channels out of range [%u]", host->nchannels);

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

static void hci_start_channel (hci_device_t* host, stage_data_t* stage_data) {
    u32 chan              = stage_data->nchannel;
    stage_data->sub_state = eSTAGE_SUB_STATE_WAIT_FOR_TRANSACTION_COMPLETE;
    // Reset all pending channel interrupts
    write32 (HOST_CHAN_INT_REG (chan), U32_MAX);
    // Set the transfer size in bytes, in number of packets, and the PID
    u32 bytes_to_transfer   = stage_data->nbytes_per_transaction;
    u32 packets_to_transfer = stage_data->npackets_per_transaction;
    u32 transfer_info       = 0;
    transfer_info |= (bytes_to_transfer & 0x7FFFF);
    transfer_info |= (packets_to_transfer << 19) & (0x3FF << 19);
    transfer_info |= (stage_data->pendpoint->pid << 29);
    write32 (HOST_CHAN_TRANSFER_INFO_REG (chan), transfer_info);
    // Allocate a dma buffer that is aligned to the cache line size in
    // address and size
    // void* dma_buffer = allocate_dma_buffer_ (bytes_to_transfer);
    // memcpy (stage_data->ptransfer_buf, bytes_to_transfer, dma_buffer);
    // stage_data->ptransfer_buf = dma_buffer;
    // Convert stage_data's transfer buf address to a VPU address
    // Then give it to the DMA reg
    uintptr_t buf_addr = (uintptr_t)stage_data->ptransfer_buf;
    write32 (HOST_CHAN_DMA_ADDR_REG (chan), ARM_TO_VPU_BUS_ADDR (buf_addr));
    // Make transfer buffer coherent with DMA
    clean_invalidate_data_cache_vaddr (buf_addr, bytes_to_transfer);

    // TODO: do I need split control???
    write32 (HOST_CHAN_SPLIT_CTRL_REG (chan), 0);

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
    character |= (stage_data->dev_addr << 22);
    // Set the end point type for the device
    character &= ~(3 << 18);
    character |= (stage_data->pendpoint->type << 18);
    // Set the end point number for the device
    character &= ~(0xF << 11);
    character |= (stage_data->pendpoint->number << 11);

    u32 frame_number = read32 (HOST_FRM_NUM) & 0xFFFF;
    /*
     * If the frame number is odd then make it as odd
     */
    if (frame_number & 1) {
        character |= (1 << 29);
    } else {
        character &= ~(1 << 29);
    }
    /*
     * Clear complete, halt and error bits in the host interrupt
     * mask register.
     */
    u32 chan_interrupt_mask = read32 (HOST_CHAN_INT_MASK_REG (chan));
    // Clear complete bit
    chan_interrupt_mask |= (1 << 0);
    // Clear halt bit
    chan_interrupt_mask |= (1 << 1);
    // Clear error bits
    chan_interrupt_mask |= HOST_CHAN_INT_ERROR_MASK;
    write32 (HOST_CHAN_INT_MASK_REG (chan), chan_interrupt_mask);
    /*
     * Begin the transaction
     */
    // Set character enable bit
    character |= (1 << 31);
    // Clear character disable bit
    character &= ~(1 << 30);
    write32 (HOST_CHAN_CHARACTER_REG (chan), character);
}

static void hci_start_transaction (hci_device_t* host, stage_data_t* stage_data) {
    u32 chan = stage_data->nchannel;
    // Check if the channel is enabled if it is disable it
    u32 character = read32 (HOST_CHAN_CHARACTER_REG (chan));
    if (character & (1 << 31)) {
        LOG_WARN ("USB channel was NOT disabled");
        // Set the sub state to wait for channel disable so the start_channel
        // can be called when the interrupt is received
        stage_data->sub_state = eSTAGE_SUB_STATE_WAIT_FOR_CHAN_DISABLE;
        // Set channel enable to 0
        character &= ~(1 << 31);
        // Disable the channel
        character |= (1 << 30);
        write32 (HOST_CHAN_CHARACTER_REG (chan), character);
        // Halt any interrupts
        write32 (HOST_CHAN_INT_MASK_REG (chan), (1 << 1));
    } else {
        hci_start_channel (host, stage_data);
    }
}

static void stage_data_init_ (
hci_device_t const* host,
stage_data_t* stage_data,
usb_device_t* device,
endpoint_t* endpoint,
usb_request_t* request,
u32 chan,
bool is_dir_in,
bool is_status_stage) {
    memset (stage_data, 0, sizeof (stage_data_t));
    stage_data->pdevice         = device;
    stage_data->pendpoint       = endpoint;
    stage_data->prequest        = request;
    stage_data->is_dir_in       = is_dir_in;
    stage_data->status_stage    = is_status_stage;
    stage_data->nchannel        = chan;
    stage_data->max_packet_size = endpoint->max_packet_size;
    stage_data->speed           = device->speed;
    stage_data->dev_addr        = device->address;

    if (is_status_stage == false) {
        if (endpoint->pid == ePID_SETUP) {
            stage_data->total_tsize_nbytes = sizeof (setup_data_t);
            stage_data->ptransfer_buf      = &request->setup_data;
        } else {
            stage_data->total_tsize_nbytes = request->data_size;
            stage_data->ptransfer_buf      = request->pdata;
        }
        stage_data->total_tsize_npackets =
        (stage_data->total_tsize_nbytes / stage_data->max_packet_size) + 1;

        stage_data->nbytes_per_transaction   = stage_data->total_tsize_nbytes;
        stage_data->npackets_per_transaction = stage_data->total_tsize_npackets;
    } else {
        stage_data->total_tsize_nbytes   = 0;
        stage_data->total_tsize_npackets = 1;

        stage_data->nbytes_per_transaction   = 0;
        stage_data->npackets_per_transaction = 1;
    }
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

// static void hci_free_device_channel (hci_device_t* host, u32 chan) {
//     host->allocated_channels &= ~(1 << chan);
// }

static bool
hci_transfer_stage (hci_device_t* host, usb_request_t* request, bool is_dir_in, bool status_stage) {
    s32 chan = hci_allocate_device_channel (host);
    if (chan == -1) {
        LOG_ERROR ("USB unable to allocate channel");
        return false;
    }
    usb_device_t* device = &host->usb_device;
    endpoint_t* endpoint = &host->usb_device.endpoint;
    ASSERT ((device != NULLPTR), "USB device was null");
    ASSERT ((endpoint != NULLPTR), "USB endpoint was null");
    stage_data_t* stage_data = &host->stage_data[chan];
    stage_data_init_ (host, stage_data, device, endpoint, request, chan, is_dir_in, status_stage);
    host->waiting = true;

    stage_data->state = eSTAGE_STATE_NO_SPLIT_TRANSFER;
    // Enable interrupts for this channel
    enable_channel_interrupts (chan);

    hci_start_transaction (host, stage_data);
    // Block until transaction is complete.
    // host->waiting is set to false in interrupt handler
    while (host->waiting) {
    }
    return true;
}

static bool hci_submit_blocking_request (hci_device_t* host, usb_request_t* request) {
    endpoint_t* endpoint = request->pendpoint;
    if (endpoint->type == eEND_POINT_TYPE_CONTROL) {
        if (request->setup_data.bmrequest_type & eREQUEST_TYPE_IN) {
            // Send setup packet
            bool status = hci_transfer_stage (host, request, false, false);
            // Receive data from device
            status &= hci_transfer_stage (host, request, true, false);
            // Send ack to device
            status &= hci_transfer_stage (host, request, false, true);
            return status;
        } else {
            if (request->data_size == 0) {
                // Send setup packet
                bool status = hci_transfer_stage (host, request, false, false);
                // Receive ack from device
                status &= hci_transfer_stage (host, request, true, true);
                return status;
            } else {
                // Send setup packet
                bool status = hci_transfer_stage (host, request, false, false);
                // Send data to device
                status &= hci_transfer_stage (host, request, false, false);
                // Received ack from device
                status &= hci_transfer_stage (host, request, true, true);
                return status;
            }
        }
    } else {
        // Interrupt or Bulk Type Endpoint
        bool status =
        hci_transfer_stage (host, request, request->pendpoint->is_direction_in, false);
        return status;
    }
}

static s32 hci_control_message (
hci_device_t* host,
endpoint_t* endpoint,
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

    usb_request_t request;
    request.pendpoint  = endpoint;
    request.pdata      = data;
    request.data_size  = wlength;
    request.setup_data = setup_data;

    ASSERT (
    (endpoint->type == eEND_POINT_TYPE_CONTROL),
    "USB invalid end point type for control msg [%u]", endpoint->type);

    s32 result = -1;
    if (hci_submit_blocking_request (host, &request)) {
        result = wlength;
    }
    return result;
}

static s32
hci_get_descriptor (hci_device_t* host, void* desc_receive_buf, u16 desc_size_nbytes, descriptor_type_t desc_type) {

    return hci_control_message (
    host, &host->usb_device.endpoint, eREQUEST_TYPE_IN, eREQUEST_CODE_GET_DESCRIPTOR,
    (desc_type << 8) | 0, 0, desc_size_nbytes, desc_receive_buf);
}

static void usb_endpoint_init_ (hci_device_t* host, endpoint_t* endpoint) {
    memset (endpoint, 0, sizeof (endpoint_t));
    endpoint->type            = eEND_POINT_TYPE_CONTROL;
    endpoint->is_direction_in = false;
    endpoint->pid             = ePID_SETUP;
    endpoint->max_packet_size = USB_DEFAULT_MAX_PACKET_SIZE;
    endpoint->number          = 0;
}

static usb_status_t
usb_device_init (hci_device_t* host, usb_device_t* device, usb_speed_t speed) {
    memset (device, 0, sizeof (usb_device_t));
    usb_endpoint_init_ (host, &device->endpoint);
    // device->desc        = 0;
    device->address     = 0; // Default device address
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

static bool
hci_root_port_init_ (hci_device_t* host, usb_device_t* device, hci_root_port_t* port) {
    port->device = device;
    port->host   = host;

    usb_speed_t speed = hci_get_port_speed ();
    if (speed == eUSB_SPEED_UNKNOWN) {
        return false;
    }

    usb_status_t status = usb_device_init (host, device, speed);
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to setup root port");
        return status;
    }
    return true;
}

static void hci_channel_irq_handler_ (hci_device_t* host, u32 chan) {
    ASSERT ((host != NULLPTR), "USB interrupt was called on channel [%u] with null host", chan);
    stage_data_t* stage_data = &host->stage_data[chan];
    // LOG_INFO ("USB channel interrupt called for channel [%u]", chan);
    switch (stage_data->sub_state) {
    case eSTAGE_SUB_STATE_WAIT_FOR_CHAN_DISABLE:
        hci_start_channel (host, stage_data);
        return;
    case eSTAGE_SUB_STATE_WAIT_FOR_TRANSACTION_COMPLETE:
        u32 transfer_size  = read32 (HOST_CHAN_TRANSFER_INFO_REG (chan));
        u32 chan_interrupt = read32 (HOST_CHAN_INT_REG (chan));
        /*
         * If the transaction is halted (1 << 1) then restart it
         */
        if (chan_interrupt == (1 << 1)) {
            // LOG_DEBUG ("USB transaction was halted [0x%X]", chan_interrupt);
            hci_start_transaction (host, stage_data);
            return;
        }
        invalidate_data_cache_vaddr (
        (uintptr_t)stage_data->ptransfer_buf, stage_data->nbytes_per_transaction);
        u32 npackets_left = (transfer_size >> 19) & 0x3FF;
        u32 nbytes_left   = transfer_size & 0x7FFFF;
        /*
         * Check for errors, NAKs, or NYET
         */
        if (chan_interrupt & (HOST_CHAN_INT_ERROR_MASK | HOST_CHAN_INT_NAK_BIT | HOST_CHAN_INT_NYET_BIT)) {
            /*
             * If NAK was received
             */
            // if(chan_interrupt & HOST_CHAN_INT_NAK_BIT) {
            //     LOG_ERROR("USB transaction on channel [%u] received a NAK", chan);
            // }
            LOG_ERROR ("USB transaction on channel [%u] has errors [0x%X]", chan, chan_interrupt);
        }
        u32 npackets_sent = stage_data->total_tsize_npackets - npackets_left;
        u32 nbytes_sent   = stage_data->total_tsize_nbytes - nbytes_left;
        pid_t* pid        = &stage_data->pendpoint->pid;
        LOG_INFO (
        "USB packets/bytes left [%u]/[%u] packets/bytes sent [%u]/[%u]",
        npackets_left, nbytes_left, npackets_sent, nbytes_sent);
        if (stage_data->status_stage == false) {
            switch (*pid) {
            case ePID_SETUP: *pid = ePID_DATA_1; break;
            case ePID_DATA_0:
                if (npackets_sent & 1) {
                    *pid = ePID_DATA_1;
                }
                break;
            case ePID_DATA_1:
                if (npackets_sent & 1) {
                    *pid = ePID_DATA_0;
                }
                break;
            }
        }
        stage_data->total_packets_sent += npackets_sent;
        stage_data->total_bytes_sent += nbytes_sent;
        stage_data->ptransfer_buf = ((u8*)stage_data->ptransfer_buf) + nbytes_sent;
        break;
    default:
        ASSERT (false, "USB stage_data has invalid substate [%u]", stage_data->sub_state);
        return;
    }
}

static void hci_irq_handler_ (u32 irq_id, void* context) {
    u32 int_status = read32 (CORE_INT_STATUS_REG);
    // LOG_INFO ("USB inside interrupt handler");
    if (context == NULLPTR) {
        LOG_ERROR ("USB interrupt handler device is null");
        return;
    }
    hci_device_t* host = (hci_device_t*)context;
    /*
     * If interrupt is intended for the host controller (1 << 25)
     * call interrupt handler for every allocated channel
     */
    if (int_status & (1 << 25)) {
        u32 all_chan_interrupt = read32 (HOST_ALLCHAN_INT_STATUS_REG);
        // Acknowledge interrupts
        write32 (HOST_ALLCHAN_INT_STATUS_REG, all_chan_interrupt);

        for (u32 i = 0; i < host->nchannels; ++i) {
            if (all_chan_interrupt & (1 << i)) {
                // Acknowledge channel interrupt
                write32 (HOST_CHAN_INT_REG (i), 0);
                hci_channel_irq_handler_ (host, i);
            }
        }
    }
    /*
     * If the host device supports detecting when a device
     * connects or disconnects see if the interrupt is a connect
     * or disconnect interrupt
     */
    if (host->is_plugnplay) {
        /*
         * Check if the status of the port has changed
         */
        if (int_status & (1 << 24)) {
            u32 host_port = read32 (HOST_PORT_REG);
            /*
             * Check if a device was CONNECTED
             */
            if (host_port & (1 << 1)) {
                LOG_INFO ("USB detected that a device was recently CONNECTED");
            }
        }
        /*
         * Check if a device was DISCONNECTED
         */
        if (int_status & (1 << 29)) {
            LOG_INFO ("USB detected that a device was recently DISCONNECTED");
        }
    }
}

void rescan_devices (hci_device_t* host) {
    while (host->is_rootport_enabled == false) {
        if (host->is_rootport_enabled == false) {
            if (enable_root_port_ (host)) {
                host->is_rootport_enabled = true;
                if (hci_root_port_init_ (host, &host->usb_device, &host->root_port) == false) {
                    LOG_ERROR ("USB failed to initialize root port");
                }
            } else {
                LOG_ERROR (
                "USB failed to ENABLE root port. No device connected to "
                "the root port");
            }
        }
        wait_ms (1000);
    }
}

usb_status_t hci_init (hci_device_t* host) {
    memset (host, 0, sizeof (hci_device_t));
    host->is_plugnplay = true;

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
    gic_enable_interrupt (VC_GIC_USB_IRQ, &hci_irq_handler_, (void*)host);

    status = core_init (host);
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to init core");
        return status;
    }
    enable_global_interrupts_ ();

    status = host_init_ (host);
    if (status != eUSB_STATUS_OK) {
        LOG_ERROR ("USB failed to init host");
        return status;
    }
    LOG_INFO ("USB successfully initialized core and host with Vendor ID [%X]", vendor_id);
    rescan_devices (host);
    return eUSB_STATUS_OK;
}
