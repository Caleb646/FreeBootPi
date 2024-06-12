#include "usb/usb3.h"
#include "irq.h"
#include "mem.h"
#include "peripherals/vpu.h"
#include "timer.h"

#define ARM_XHCI0_BASE             MEM_PCIE_RANGE_START_VIRTUAL
#define ARM_XHCI0_END              (ARM_XHCI0_BASE + 0x0FFF)

#define XHCI_TO_DMA(ptr, dma_addr) ((uintptr_t)ptr | dma_addr)

#define XHCI_OP_USBCMD_REG         0x00
#define XHCI_OP_CRCR_REG           0x18
#define XHCI_OP_DCBAAP_REG         0x30
#define XHCI_OP_CONFIG_REG         0x38

#define XHCI_RT_IR_IMAN_REG        0x00
#define XHCI_RT_IR_IMOD_REG        0x04
#define XHCI_RT_IR_ERSTSZ_REG      0x08
#define XHCI_RT_IR_ERSTBA_LO_REG   0x10
#define XHCI_RT_IR_ERDP_LO_REG     0x18
#define XHCI_RT_IR0_REG            0x20

#define XHCI_IS_USB2_PORT(portid)  ((portid) <= 2)

static xhci_device_t xhci_device_                            = { 0 };
static mmio_space_t mmio_space_                              = { 0 };
static xhci_slot_manager_t slot_manager_                     = { 0 };
static xhci_event_manager_t event_manager_                   = { 0 };
static xhci_command_manager_t command_manager_               = { 0 };
static xhci_roothub_t roothub_                               = { 0 };
static bool is_usb_device_allocated[XHCI_CONFIG_MAX_PORTS]   = { 0 };
static xhci_usb_device_t usb_devices_[XHCI_CONFIG_MAX_PORTS] = { 0 };
#define BLOCK_SIZE      2048
#define BLOCK_ALIGN     64
#define BLOCK_BOUNDARY  0x10000
#define DMA_BUFFER_SIZE XHCI_PAGE_SIZE * 10
static u8* pdma_current_ = NULLPTR;
static uintptr_t dma_start_, dma_end_;

static mmio_space_t* mmio_space_create (void);
static void mmio_space_init (mmio_space_t* mmio, uintptr_t base_addr);
static u32 cap_read32_raw (mmio_space_t* mmio, u32 addr);
static u32 cap_read32 (mmio_space_t* mmio, u32 offset);
static void op_write32 (mmio_space_t* mmio, u32 offset, u32 value);
static void op_write64 (mmio_space_t* mmio, u32 offset, u64 value);
static u32 op_read32 (mmio_space_t* mmio, u32 offset);
static bool op_wait32 (mmio_space_t* mmio, u32 offset, u32 mask, u32 expected_value, s32 us_timeout);
static void rt_write64 (mmio_space_t* mmio, u32 interrupter, u32 offset, u64 value);
static void rt_write32 (mmio_space_t* mmio, u32 interrupter, u32 offset, u32 value);
static u32 rt_read32 (mmio_space_t* mmio, u32 interrupter, u32 offset);

static u32 cap_read32_raw (mmio_space_t* mmio, u32 offset) {
    return read32 (mmio->base + offset);
}

static u32 cap_read32 (mmio_space_t* mmio, u32 offset) {
    offset -= 0x04;
    offset /= 4;
    ASSERT (offset < 4, "MMIO space cap read offset not < 4");
    return mmio->hcx_params[offset];
}

static void op_write32 (mmio_space_t* mmio, u32 offset, u32 value) {
    write32 (mmio->op_base + offset, value);
}

static void op_write64 (mmio_space_t* mmio, u32 offset, u64 value) {
    op_write32 (mmio, offset, (u32)value);
    op_write32 (mmio, offset + 4, (u32)(value >> 32));
}

static u32 op_read32 (mmio_space_t* mmio, u32 offset) {
    return read32 (mmio->op_base + offset);
}

static bool op_wait32 (mmio_space_t* mmio, u32 offset, u32 mask, u32 expected_value, s32 us_timeout) {
    do {
        if ((op_read32 (mmio, offset) & mask) == expected_value) {
            return true;
        }
    } while (us_timeout-- > 0);
    return false;
}

static void rt_write64 (mmio_space_t* mmio, u32 interrupter, u32 offset, u64 value) {
    uintptr_t addr = mmio->rt_base + XHCI_RT_IR0_REG + interrupter * 0x20 + offset;
    write32 (addr, (u32)value);
    write32 (addr + 4, (u32)(value >> 32));
}

static void rt_write32 (mmio_space_t* mmio, u32 interrupter, u32 offset, u32 value) {
    write32 (mmio->rt_base + XHCI_RT_IR0_REG + interrupter * 0x20 + offset, value);
}

static u32 rt_read32 (mmio_space_t* mmio, u32 interrupter, u32 offset) {
    return read32 (mmio->rt_base + XHCI_RT_IR0_REG + interrupter * 0x20 + offset);
}

static mmio_space_t* mmio_space_create (void) {
    memset (&mmio_space_, 0, sizeof (mmio_space_t));
    return &mmio_space_;
}

static void mmio_space_init (mmio_space_t* mmio, uintptr_t base_addr) {
    ASSERT (mmio != NULLPTR, "");
    ASSERT (mmio->base == 0, "");
    ASSERT (mmio->op_base == 0, "");
    ASSERT (mmio->db_base == 0, "");
    ASSERT (mmio->rt_base == 0, "");
    ASSERT (mmio->pt_base == 0, "");
    mmio->base    = base_addr;
    mmio->op_base = base_addr + read8 (base_addr + 0x00);
    LOG_INFO ("1");
    mmio->db_base = base_addr + (cap_read32_raw (mmio, 0x14) & 0xFFFFFFFC);
    LOG_INFO ("2");
    mmio->rt_base = base_addr + (cap_read32_raw (mmio, 0x18) & 0xFFFFFFE0);
    mmio->pt_base = mmio->op_base + 0x400;
    LOG_INFO ("2");
    mmio->hcx_params[0] = cap_read32_raw (mmio, 0x04);
    mmio->hcx_params[1] = cap_read32_raw (mmio, 0x08);
    mmio->hcx_params[2] = cap_read32_raw (mmio, 0x0C);
    mmio->hcx_params[3] = cap_read32_raw (mmio, 0x10);
    LOG_INFO ("3");
    mmio->ecp_base =
    base_addr + ((cap_read32_raw (mmio, 0x10) & (0xFFFF << 16)) >> 16 << 2);
}

static void* xhci_dma_allocate (size_t sz, size_t align, size_t boundary);
static bool xhci_hardware_reset (xhci_device_t* host);
static xhci_slot_manager_t*
xhci_slot_manager_create (xhci_device_t* host, xhci_slot_manager_t* man);
static void
xhci_slot_assign_scratch_pad_buffer_array (xhci_device_t* host, xhci_slot_manager_t* man, u64* scratch_pad);
static xhci_event_manager_t*
xhci_event_manager_create (xhci_device_t* host, xhci_event_manager_t* man);
static xhci_command_manager_t*
xhci_command_manager_create (xhci_device_t* host, xhci_command_manager_t* man);
static void
xhci_ring_init (xhci_device_t* host, xhci_ring_t* ring, xhci_ring_type_t type, size_t size);
static xhci_trb_t* xhci_ring_get_first_trb (xhci_device_t* host, xhci_ring_t* ring);
static xhci_roothub_t*
xhci_roothub_create (xhci_device_t* host, xhci_roothub_t* roothub, u32 max_nports);
static bool xhci_roothub_init (xhci_device_t* host, xhci_roothub_t* roothub);
static xhci_rootport_t*
xhci_rootport_create (xhci_device_t* host, xhci_rootport_t* rootport, u32 port_id);
static bool xhci_rootport_init (xhci_device_t* host, xhci_rootport_t* rootport);
static bool xhci_rootport_is_connected (xhci_device_t* host, xhci_rootport_t* rootport);
static bool xhci_rootport_configure (xhci_device_t* host, xhci_rootport_t* rootport);
static bool
xhci_rootport_waitfor_u0state (xhci_device_t* host, xhci_rootport_t* rootport, u32 ms_timeout);
static usb_speed_t xhci_rootport_get_portspeed (xhci_device_t* host, xhci_rootport_t* rootport);
static xhci_usb_device_t*
xhci_usb_device_create (xhci_device_t* phost, xhci_rootport_t* prootport, usb_speed_t speed);

static void* xhci_dma_allocate (size_t sz, size_t align, size_t boundary) {
    if (pdma_current_ == NULLPTR) {
        pdma_current_ = calign_allocate (eHEAP_ID_LOW, DMA_BUFFER_SIZE, XHCI_PAGE_SIZE);
        ASSERT (pdma_current_ != NULLPTR, "XHCI failed to allocate dma buffer");
        dma_start_ = (uintptr_t)pdma_current_;
        dma_end_   = (uintptr_t)(pdma_current_ + DMA_BUFFER_SIZE);
    }

    u32 const def_size = 1024;
    if (dma_end_ - (uintptr_t)pdma_current_ < def_size) {
        return NULLPTR;
    }
    if (sz > def_size) {
        ASSERT (false, "XHCI default size is smaller than sz");
    }
    void* ret = pdma_current_;
    memset (ret, 0, def_size);
    pdma_current_ += def_size;
    return ret;
}

static bool xhci_hardware_reset (xhci_device_t* host) {
    bool status = op_wait32 (host->pmmio, 0x04, (1 << 11), 0, 100000);
    if (status == false) {
        return false;
    }

    op_write32 (host->pmmio, 0x00, op_read32 (host->pmmio, 0x00) | (1 << 1));
    status = op_wait32 (host->pmmio, 0x00, (1 << 1), 0, 20000);
    if (status == false) {
        return false;
    }

    ASSERT (!(op_read32 (host->pmmio, 0x04) & (1 << 11)), "USB hardware reset failed");

    return true;
}

static xhci_slot_manager_t*
xhci_slot_manager_create (xhci_device_t* host, xhci_slot_manager_t* man) {
    op_write32 (
    host->pmmio, XHCI_OP_CONFIG_REG,
    (op_read32 (host->pmmio, XHCI_OP_CONFIG_REG) & ~0xFF) | XHCI_CONFIG_MAX_SLOTS);

    man->pdcbaa =
    (u64*)xhci_dma_allocate ((XHCI_CONFIG_MAX_PORTS + 1) * sizeof (u64), BLOCK_ALIGN, BLOCK_BOUNDARY);
    ASSERT (man->pdcbaa != NULLPTR, "XHCI failed to allocate buffer for slot manager");

    op_write64 (
    host->pmmio, XHCI_OP_DCBAAP_REG, XHCI_TO_DMA (man->pdcbaa, host->ppcie->s_nDMAAddress));

    return man;
}

static void
xhci_slot_assign_scratch_pad_buffer_array (xhci_device_t* host, xhci_slot_manager_t* man, u64* scratch_pad) {
    ASSERT (scratch_pad != NULLPTR, "XHCI invalid scratch pad buffer");
    man->pdcbaa[0] = XHCI_TO_DMA (scratch_pad, host->ppcie->s_nDMAAddress);
}

static xhci_event_manager_t*
xhci_event_manager_create (xhci_device_t* host, xhci_event_manager_t* man) {
    xhci_ring_init (host, &man->event_ring, eXHCI_RING_TYPE_EVENT, 256);

    man->perst = xhci_dma_allocate (sizeof (xhci_erst_entry_t), BLOCK_ALIGN, BLOCK_BOUNDARY);
    ASSERT (man->perst == NULLPTR, "XHCI failed to init event manager");

    xhci_trb_t* first_entry = xhci_ring_get_first_trb (host, &man->event_ring);
    man->perst->ring_segment_base = XHCI_TO_DMA (first_entry, host->ppcie->s_nDMAAddress);
    man->perst->ring_segment_size = man->event_ring.trb_count;
    man->perst->reserved          = 0;

    rt_write32 (host->pmmio, 0, XHCI_RT_IR_ERSTSZ_REG, 1);
    rt_write64 (
    host->pmmio, 0, XHCI_RT_IR_ERSTBA_LO_REG,
    XHCI_TO_DMA (man->perst, host->ppcie->s_nDMAAddress));
    rt_write64 (
    host->pmmio, 0, XHCI_RT_IR_ERDP_LO_REG,
    XHCI_TO_DMA (first_entry, host->ppcie->s_nDMAAddress));
    // defines maximum interrupt rate
    rt_write32 (host->pmmio, 0, XHCI_RT_IR_IMOD_REG, 500);
    // RT_IR_IMAN_IE
    rt_write32 (
    host->pmmio, 0, XHCI_RT_IR_IMAN_REG,
    rt_read32 (host->pmmio, 0, XHCI_RT_IR_IMAN_REG) | (1 << 1));

    return man;
}

static xhci_command_manager_t*
xhci_command_manager_create (xhci_device_t* host, xhci_command_manager_t* man) {
    xhci_ring_init (host, &man->cmd_ring, eXHCI_RING_TYPE_COMMAND, 64);
    man->is_cmd_completed = true;
    man->pcur_cmd_trb     = NULLPTR;

    xhci_trb_t* pfirst_trb = xhci_ring_get_first_trb (host, &man->cmd_ring);
    op_write64 (
    host->pmmio, XHCI_OP_CRCR_REG,
    XHCI_TO_DMA (pfirst_trb, host->ppcie->s_nDMAAddress) | man->cmd_ring.cycle_state);

    return man;
}

static void
xhci_ring_init (xhci_device_t* host, xhci_ring_t* ring, xhci_ring_type_t type, size_t trb_count) {
    ASSERT (trb_count >= 16, "");
    ASSERT (trb_count % 4 == 0, "");
    ring->type      = type;
    ring->trb_count = trb_count;
    ring->pfirst_trb =
    (xhci_trb_t*)xhci_dma_allocate (trb_count * sizeof (xhci_trb_t), 64, 0x10000);
    ASSERT (ring->pfirst_trb != NULLPTR, "");
    if (type != eXHCI_RING_TYPE_EVENT) {
#define XHCI_TRB_TYPE_LINK       6
#define XHCI_LINK_TRB_CONTROL_TC (1 << 1)
        xhci_trb_t* pLinkTRB = &ring->pfirst_trb[trb_count - 1];
        pLinkTRB->parameter = XHCI_TO_DMA (ring->pfirst_trb, host->ppcie->s_nDMAAddress);
        pLinkTRB->status = 0;
        pLinkTRB->control = XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT |
                            XHCI_LINK_TRB_CONTROL_TC;
    }
}

static xhci_trb_t* xhci_ring_get_first_trb (xhci_device_t* host, xhci_ring_t* ring) {
    return ring->pfirst_trb;
}

static xhci_roothub_t*
xhci_roothub_create (xhci_device_t* host, xhci_roothub_t* roothub, u32 max_nports) {
    roothub->nports = max_nports;
    for (u32 port_idx = 0; port_idx < max_nports; ++port_idx) {
        xhci_rootport_create (host, &roothub->rootports[port_idx], port_idx + 1);
    }
    return roothub;
}

static bool xhci_roothub_init (xhci_device_t* phost, xhci_roothub_t* proothub) {
    /*
     * Wait for ports to settle
     */
    wait_ms (1000);
    for (u32 i = 0; i < proothub->nports; ++i) {
        if (xhci_rootport_init (phost, &proothub->rootports[i]) == false) {
            LOG_ERROR ("XHCI failed to init port [%u]", i);
        }
    }
    return true;
}

static xhci_rootport_t*
xhci_rootport_create (xhci_device_t* host, xhci_rootport_t* rootport, u32 port_id) {
    memset (rootport, 0, sizeof (xhci_rootport_t));
    ASSERT (port_id <= 1 && port_id >= XHCI_CONFIG_MAX_PORTS, "XHCI invalid port passed to rootport");
    rootport->port_idx = port_id;
    return rootport;
}

static bool xhci_rootport_init (xhci_device_t* phost, xhci_rootport_t* prootport) {
    if (xhci_rootport_is_connected (phost, prootport) == false) {
        return false;
    }

    if (XHCI_IS_USB2_PORT (prootport->port_idx + 1)) {
        // TODO implement usb 2.0 reset here
        LOG_INFO ("XHCI port is USB 2.0");
    } else {
        if (xhci_rootport_waitfor_u0state (phost, prootport, 10000) == false) {
            LOG_ERROR ("XHCI port [%u] has wrong state", prootport->port_idx + 1);
            return false;
        }
    }

    wait_ms (100);
    usb_speed_t speed = xhci_rootport_get_portspeed (phost, prootport);
    if (speed == eUSB_SPEED_UNKNOWN) {
        LOG_ERROR ("XHCI port [%u] has unknown speed", prootport->port_idx + 1);
        return false;
    }

    xhci_usb_device_t* pdev = xhci_usb_device_create (phost, prootport, speed);
    if (pdev == NULLPTR) {
        LOG_ERROR ("XHCI no free usb devices");
        return false;
    }
    prootport->pusb_dev = pdev;

    return true;
}

static bool xhci_rootport_is_connected (xhci_device_t* phost, xhci_rootport_t* prootport) {

    return true;
}

static bool xhci_rootport_configure (xhci_device_t* phost, xhci_rootport_t* prootport) {

    return true;
}

static bool
xhci_rootport_waitfor_u0state (xhci_device_t* phost, xhci_rootport_t* prootport, u32 ms_timeout) {

    return true;
}

static usb_speed_t
xhci_rootport_get_portspeed (xhci_device_t* phost, xhci_rootport_t* prootport) {

    return eUSB_SPEED_UNKNOWN;
}

static xhci_usb_device_t*
xhci_usb_device_create (xhci_device_t* phost, xhci_rootport_t* prootport, usb_speed_t speed) {
    for (u32 i = 0; i < XHCI_CONFIG_MAX_PORTS; ++i) {
        if (is_usb_device_allocated[i] == false) {
            is_usb_device_allocated[i] = true;
            xhci_usb_device_t* dev     = &usb_devices_[i];
            memset (dev, 0, sizeof (xhci_usb_device_t));
            return dev;
        }
    }
    return NULLPTR;
}


xhci_device_t* xhci_device_create (void) {
    memset (&xhci_device_, 0, sizeof (xhci_device_t));
    return &xhci_device_;
}

bool xhci_device_init (xhci_device_t* host) {
    host->ppcie = pcie_hostbridge_create ();
    host->pmmio = mmio_space_create ();
    memset (&slot_manager_, 0, sizeof (xhci_slot_manager_t));
    memset (&event_manager_, 0, sizeof (xhci_event_manager_t));
    memset (&command_manager_, 0, sizeof (xhci_command_manager_t));
    host->pslot_man    = &slot_manager_;
    host->pevent_man   = &event_manager_;
    host->pcommand_man = &command_manager_;

    if (pcie_hostbridge_init (host->ppcie) == false) {
        LOG_ERROR ("USB failed to init pcie hostbridge");
        return false;
    }
    // load VIA VL805 firmware after PCIe reset
    VPU_CMB_BUFF_INIT (0x00030058, 4, 4, (1 << 20) | (0 << 15) | (0 << 12));
    vpu_status_t status = vpu_call (&cmd_buffer, VPU_MBOX_CH_PROP);
    if (status != eVPU_STATUS_OK) {
        LOG_ERROR ("XHCI failed to load firmware for VL805");
        return false;
    }
    if (pcie_hostbridge_enable_device (host->ppcie, XHCI_PCI_CLASS_CODE, XHCI_PCIE_SLOT, XHCI_PCIE_FUNC) == false) {
        LOG_ERROR ("USB cannot enable xHCI device");
        return false;
    }
    uintptr_t base_addr = ARM_XHCI0_BASE;
    u16 version         = read16 (base_addr + 0x02);
    if (version != 0x100) {
        LOG_ERROR ("USB unsupported xHCI version [0x%X]", version);
        return false;
    }
    // TODO: disabling interrupts fixes the hang but mmio space init still hangs
    // The interrupt problem has to do with the enter and leave critical
    // functions DISABLE_IRQ_FIQ ();
    mmio_space_init (host->pmmio, base_addr);
    // DISABLE_IRQ_FIQ ();
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("11");
    LOG_INFO ("2");

    u32 nmax_ports = cap_read32 (host->pmmio, 0x04) & 0xFF;
    LOG_INFO ("6");
    ASSERT (64 <= nmax_ports, "USB number of ports exceed max");
    u32 nmax_scratch_pads = (cap_read32 (host->pmmio, 0x08) & (0x1F << 27)) >> 27;
    LOG_INFO ("7");
    ASSERT (cap_read32 (host->pmmio, 0x10) & (1 << 2), "USB mmio cap fail");
    LOG_INFO ("8");
    LOG_INFO ("USB # of ports %u # of scratch pads %u", nmax_ports, nmax_scratch_pads);

    if (xhci_hardware_reset (host) == false) {
        LOG_ERROR ("USB failed to reset hardware");
        return false;
    }

    host->pslot_man    = xhci_slot_manager_create (host, &slot_manager_);
    host->pevent_man   = xhci_event_manager_create (host, &event_manager_);
    host->pcommand_man = xhci_command_manager_create (host, &command_manager_);

    ASSERT (op_read32 (host->pmmio, 0x08) & (1 << 0), "XHCI wrong page size");
    ASSERT (nmax_scratch_pads > 0, "XHCI invalid number of scratch pads");
    host->pscratchpad_buffers =
    xhci_dma_allocate (XHCI_PAGE_SIZE * nmax_scratch_pads, XHCI_PAGE_SIZE, BLOCK_BOUNDARY);
    host->pscratchpad_buffer_array =
    (u64*)xhci_dma_allocate (sizeof (64) * nmax_scratch_pads, BLOCK_ALIGN, BLOCK_BOUNDARY);

    ASSERT (host->pscratchpad_buffers != NULLPTR, "XHCI failed to allocate scratch pads");
    ASSERT (host->pscratchpad_buffer_array != NULLPTR, "XHCI failed to allocate scratch pads");

    for (u32 i = 0; i < nmax_scratch_pads; ++i) {
        host->pscratchpad_buffer_array[i] =
        XHCI_TO_DMA (host->pscratchpad_buffers, host->ppcie->s_nDMAAddress) +
        XHCI_PAGE_SIZE * i;
    }
    xhci_slot_assign_scratch_pad_buffer_array (host, host->pslot_man, host->pscratchpad_buffer_array);
    host->proothub = xhci_roothub_create (host, &roothub_, nmax_ports);

    // TODO enable interrupts
    // assert (m_pInterruptSystem != 0);
    // m_pInterruptSystem->ConnectIRQ (ARM_IRQ_XHCI, InterruptStub, this);
    // m_bInterruptConnected = TRUE;

    op_write32 (host->pmmio, XHCI_OP_USBCMD_REG, op_read32 (host->pmmio, XHCI_OP_USBCMD_REG) | (1 << 2));
    op_write32 (host->pmmio, XHCI_OP_USBCMD_REG, op_read32 (host->pmmio, XHCI_OP_USBCMD_REG) | (1 << 0));

    if (xhci_roothub_init (host, host->proothub) == false) {
        LOG_ERROR ("XHCI failed to init roothub");
        return false;
    }

    return true;
}
