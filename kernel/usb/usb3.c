#include "usb/usb3.h"
#include "irq.h"
#include "mem.h"
#include "peripherals/vpu.h"
#include "sync.h"
#include "timer.h"

#define ARM_XHCI0_BASE                                    MEM_PCIE_RANGE_START_VIRTUAL
#define ARM_XHCI0_END                                     (ARM_XHCI0_BASE + 0x0FFF)

// TRB Completion Codes
#define XHCI_TRB_COMPLETION_CODE_SUCCESS                  1
#define XHCI_TRB_COMPLETION_CODE_NO_SLOTS_AVAILABLE_ERROR 9
#define XHCI_TRB_COMPLETION_CODE_SHORT_PACKET             13
#define XHCI_TRB_COMPLETION_CODE_RING_UNDERRUN            14
#define XHCI_TRB_COMPLETION_CODE_RING_OVERRUN             15
#define XHCI_TRB_COMPLETION_CODE_EVENT_RING_FULL_ERROR    21
#define XHCI_TRB_COMPLETION_CODE_MISSED_SERVICE_ERROR     23
//
// Private Completion Codes
//
#define XHCI_PRIV_COMPLETION_CODE_ERROR                   -1
#define XHCI_PRIV_COMPLETION_CODE_TIMEOUT                 -2

#define XHCI_TO_DMA(ptr, dma_addr)                        ((uintptr_t)ptr | dma_addr)
#define XHCI_TO_DMA_LO(ptr, dma_addr)                     ((u32)XHCI_TO_DMA (ptr, dma_addr))
#define XHCI_TO_DMA_HI(ptr, dma_addr)                     ((u32)(XHCI_TO_DMA (ptr, dma_addr) >> 32))
#define XHCI_IS_USB2_PORT(portid)                         ((portid) == 1)
#define XHCI_TRB_SUCCESS(com_code) \
    ((com_code) == XHCI_TRB_COMPLETION_CODE_SUCCESS)
#define XHCI_CMD_SUCCESS    XHCI_TRB_SUCCESS
#define XHCI_IS_SLOTID(num) (1 <= (num) && (num) <= XHCI_CONFIG_MAX_SLOTS)
#define XHCI_USB_SPEED_TO_PSI(speed) \
    ((unsigned)((speed) < eUSB_SPEED_HIGH ? ((speed) ^ 1) + 1 : (speed) + 1))

#define XHCI_OP_USBCMD_REG            0x00
#define XHCI_OP_CRCR_REG              0x18
#define XHCI_OP_DCBAAP_REG            0x30
#define XHCI_OP_CONFIG_REG            0x38
#define XHCI_OP_PORTS_BASE_REG        0x400

#define XHCI_PT_PORTS_PORTSC_REG      0x00

#define XHCI_RT_IR_IMAN_REG           0x00
#define XHCI_RT_IR_IMOD_REG           0x04
#define XHCI_RT_IR_ERSTSZ_REG         0x08
#define XHCI_RT_IR_ERSTBA_LO_REG      0x10
#define XHCI_RT_IR_ERDP_LO_REG        0x18
#define XHCI_RT_IR0_REG               0x20

#define XHCI_REG_DB_HC_COMMAND_TARGET 0
#define XHCI_REG_DB_EP0_TARGET        1
#define XHCI_REG_DB_EP1_OUT_TARGET    2
#define XHCI_REG_DB_EP1_IN_TARGET     3
#define XHCI_REG_DB_EP15_OUT_TARGET   30
#define XHCI_REG_DB_EP15_IN_TARGET    31

#define XHCI_DB_HOST_CONTROLLER_REG   0
#define XHCI_DB_DEVICE_CONTEXT1_REG   1
#define XHCI_DB_DEVICE_CONTEXT255_REG 255

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

static u32 pt_read32 (mmio_space_t* mmio, u32 port, u32 offset);
static void pt_write32 (mmio_space_t* mmio, u32 port, u32 offset, u32 value);

static void db_write32 (mmio_space_t* mmio, u32 slot, u32 value);

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
        wait_us (1);
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

static u32 pt_read32 (mmio_space_t* mmio, u32 port, u32 offset) {
    return read32 (mmio->pt_base + port * (4 * 4) + offset);
}

static void pt_write32 (mmio_space_t* pmmio, u32 port, u32 offset, u32 value) {
    write32 (pmmio->pt_base + port * (4 * 4) + offset, value);
}

static void db_write32 (mmio_space_t* pmmio, u32 slot, u32 value) {
    write32 (pmmio->db_base + slot * 4, value);
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
    mmio->db_base = base_addr + (cap_read32_raw (mmio, 0x14) & 0xFFFFFFFC);
    mmio->rt_base = base_addr + (cap_read32_raw (mmio, 0x18) & 0xFFFFFFE0);
    mmio->pt_base = mmio->op_base + 0x400;
    mmio->hcx_params[0] = cap_read32_raw (mmio, 0x04);
    mmio->hcx_params[1] = cap_read32_raw (mmio, 0x08);
    mmio->hcx_params[2] = cap_read32_raw (mmio, 0x0C);
    mmio->hcx_params[3] = cap_read32_raw (mmio, 0x10);
    mmio->ecp_base = base_addr + ((cap_read32 (mmio, 0x10) & (0xFFFF << 16)) >> 16 << 2);
}
/*
 * Utils
 */
static void* xhci_dma_allocate (size_t sz, size_t align, size_t boundary);
static bool xhci_hardware_reset (xhci_device_t* host);
/*
 * Slot Manager
 */
static xhci_slot_manager_t*
xhci_slot_manager_create (xhci_device_t* phost, xhci_slot_manager_t* pman);
static void xhci_slot_manager_assign_scratch_pad_buffer_array (
xhci_device_t* phost,
xhci_slot_manager_t* pman,
u64* pscratch_pad);
static void
xhci_slot_manager_assign_device (xhci_device_t* phost, xhci_slot_manager_t* pman, xhci_usb_device_t* pdev, u8 slot_id);
/*
 * Event Manager
 */
static xhci_event_manager_t*
xhci_event_manager_create (xhci_device_t* host, xhci_event_manager_t* man);
/*
 * Command Manager
 */
static xhci_command_manager_t*
xhci_command_manager_create (xhci_device_t* host, xhci_command_manager_t* man);
static s32
xhci_command_manager_enable_slot (xhci_device_t* host, xhci_command_manager_t* man, u8* pslot_id);
static s32 xhci_command_manager_do_cmd (
xhci_device_t* host,
xhci_command_manager_t* man,
u32 control,
u32 param1,
u32 param2,
u32 status,
u8* pslot_id);
static s32 xhci_command_manager_address_device (
xhci_device_t* host,
xhci_command_manager_t* man,
xhci_input_context_t* pinput_context_buffer,
u8 slot_id,
bool should_set_addr);
/*
 * Ring
 */
static void
xhci_ring_init (xhci_device_t* host, xhci_ring_t* ring, xhci_ring_type_t type, size_t size);
static xhci_trb_t* xhci_ring_get_first_trb (xhci_device_t* host, xhci_ring_t* ring);
static xhci_trb_t* xhci_ring_get_enqueue_trb (xhci_device_t* host, xhci_ring_t* ring);
static void xhci_ring_inc_enqueue (xhci_device_t* host, xhci_ring_t* ring);
/*
 * Root Hub
 */
static xhci_roothub_t*
xhci_roothub_create (xhci_device_t* host, xhci_roothub_t* roothub, u32 max_nports);
static bool xhci_roothub_init (xhci_device_t* host, xhci_roothub_t* roothub);
/*
 * Root Port
 */
static xhci_rootport_t*
xhci_rootport_create (xhci_device_t* host, xhci_rootport_t* rootport, u32 port_id);
static bool xhci_rootport_init (xhci_device_t* host, xhci_rootport_t* rootport);
static bool xhci_rootport_reset (xhci_device_t* host, xhci_rootport_t* rootport, u32 us_timeout);
static bool xhci_rootport_is_connected (xhci_device_t* host, xhci_rootport_t* rootport);
static bool xhci_rootport_configure (xhci_device_t* host, xhci_rootport_t* rootport);
static bool
xhci_rootport_waitfor_u0state (xhci_device_t* host, xhci_rootport_t* rootport, u32 us_timeout);
static usb_speed_t xhci_rootport_get_portspeed (xhci_device_t* host, xhci_rootport_t* rootport);
/*
 * USB Device
 */
static xhci_usb_device_t*
xhci_usb_device_create (xhci_device_t* phost, xhci_rootport_t* prootport, usb_speed_t speed);
static bool xhci_usb_device_init (xhci_device_t* phost, xhci_usb_device_t* pdev);
static xhci_input_context_t*
xhci_usb_device_get_input_context_address_device (xhci_device_t* phost, xhci_usb_device_t* pdev);

static void* xhci_dma_allocate (size_t sz, size_t align, size_t boundary) {
    if (pdma_current_ == NULLPTR) {
        pdma_current_ = npage_coherent_allocate (40); // 15 * PAGE_SIZE (4096)
        ASSERT (pdma_current_ != NULLPTR, "XHCI failed to allocate dma buffer");
        dma_start_ = (uintptr_t)pdma_current_;
        dma_end_   = (uintptr_t)(pdma_current_ + 40 * PAGE_SIZE);
    }

    if (sz <= BLOCK_SIZE && align <= BLOCK_ALIGN && boundary <= BLOCK_BOUNDARY) {
        sz       = BLOCK_SIZE;
        align    = BLOCK_ALIGN;
        boundary = BLOCK_BOUNDARY;
    }

    uintptr_t start = (uintptr_t)pdma_current_;
    size_t mask     = align - 1;
    // LOG_INFO ("0x%X -- 0x%X", (u32)align, (u32)start);
    if ((start & mask) > 0) {
        start = (start + mask) & ~mask;
    }
    // LOG_INFO ("0x%X -- 0x%X", (u32)boundary, (u32)start);
    mask = boundary - 1;
    if (((start + (sz - 1)) & ~mask) != (start & ~mask)) {
        start = (start + mask) & ~mask;
    }
    // LOG_INFO ("0x%X < 0x%X -- 0x%X", (u32)(dma_end_ - start), (u32)sz, (u32)start);
    if (dma_end_ - start < sz) {
        return NULLPTR;
    }
    pdma_current_ = (u8*)start;
    void* ret     = pdma_current_;
    memset (ret, 0, sz);
    pdma_current_ += sz;
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
    (u64*)xhci_dma_allocate ((XHCI_CONFIG_MAX_PORTS + 1) * sizeof (u64), BLOCK_ALIGN, PAGE_SIZE);
    ASSERT (man->pdcbaa != NULLPTR, "XHCI failed to allocate buffer for slot manager");

    op_write64 (
    host->pmmio, XHCI_OP_DCBAAP_REG, XHCI_TO_DMA (man->pdcbaa, host->ppcie->s_nDMAAddress));

    return man;
}

static void xhci_slot_manager_assign_scratch_pad_buffer_array (
xhci_device_t* host,
xhci_slot_manager_t* man,
u64* scratch_pad) {
    ASSERT (scratch_pad != NULLPTR, "XHCI invalid scratch pad buffer");
    man->pdcbaa[0] = XHCI_TO_DMA (scratch_pad, host->ppcie->s_nDMAAddress);
}

static void
xhci_slot_manager_assign_device (xhci_device_t* phost, xhci_slot_manager_t* pman, xhci_usb_device_t* pdev, u8 slot_id) {
    ASSERT (XHCI_IS_SLOTID (slot_id), "");
    ASSERT (pdev != NULLPTR, "");
    ASSERT (pman->pusb_devs[slot_id] == NULLPTR, "");
    ASSERT (pdev->pdev_context != NULLPTR, "");
    ASSERT (pman->pdcbaa != NULLPTR, "");
    ASSERT (pman->pdcbaa[slot_id] == 0, "");
    pman->pdcbaa[slot_id] = XHCI_TO_DMA (pdev->pdev_context, phost->ppcie->s_nDMAAddress);
}

static xhci_event_manager_t*
xhci_event_manager_create (xhci_device_t* host, xhci_event_manager_t* man) {
    xhci_ring_init (host, &man->event_ring, eXHCI_RING_TYPE_EVENT, 256);

    man->perst = xhci_dma_allocate (sizeof (xhci_erst_entry_t), BLOCK_ALIGN, PAGE_SIZE);
    ASSERT (man->perst != NULLPTR, "XHCI failed to init event manager");

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

static s32
xhci_command_manager_enable_slot (xhci_device_t* phost, xhci_command_manager_t* pman, u8* pslot_id) {
    u8 slot;
    s32 result = xhci_command_manager_do_cmd (phost, pman, 9 << 10, 0, 0, 0, &slot);
    if (XHCI_CMD_SUCCESS (result) == false) {
        return false;
    }
    if (XHCI_IS_SLOTID (slot) == false) {
        return XHCI_TRB_COMPLETION_CODE_NO_SLOTS_AVAILABLE_ERROR;
    }
    ASSERT (pslot_id == NULLPTR, "");
    *pslot_id = slot;
    return result;
}

static s32 xhci_command_manager_do_cmd (
xhci_device_t* host,
xhci_command_manager_t* man,
u32 control,
u32 param1,
u32 param2,
u32 status,
u8* pslot_id) {
    ASSERT (man->is_cmd_completed, "");
    xhci_trb_t* pcmd_trb = xhci_ring_get_enqueue_trb (host, &man->cmd_ring);
    if (pcmd_trb == NULLPTR) {
        return XHCI_TRB_COMPLETION_CODE_RING_OVERRUN;
    }
    pcmd_trb->parameter1 = param1;
    pcmd_trb->parameter2 = param2;
    pcmd_trb->status     = status;

    ASSERT ((control & XHCI_TRB_CONTROL_C) == false, "");
    pcmd_trb->control     = control | pcmd_trb->control;
    man->pcur_cmd_trb     = pcmd_trb;
    man->is_cmd_completed = false;

    xhci_ring_inc_enqueue (host, &man->cmd_ring);
    DATA_SYNC_BARRIER_FS_ANY ();

    db_write32 (host->pmmio, XHCI_DB_HOST_CONTROLLER_REG, XHCI_REG_DB_HC_COMMAND_TARGET);
    u64 start_time = get_sys_time_s ();
    while (man->is_cmd_completed == false) {
        if (get_sys_time_s () - start_time >= 3) {
            man->is_cmd_completed = true;
            man->pcur_cmd_trb     = NULLPTR;
            return XHCI_PRIV_COMPLETION_CODE_TIMEOUT;
        }
    }
    DATA_MEMORY_BARRIER_FS_ANY ();
    if (pslot_id != NULLPTR) {
        *pslot_id = man->slot_id;
    }
    return man->completion_code;
}

static s32 xhci_command_manager_address_device (
xhci_device_t* phost,
xhci_command_manager_t* pman,
xhci_input_context_t* pinput_context_buffer,
u8 slot_id,
bool should_set_addr) {
    ASSERT (XHCI_IS_SLOTID (slot_id), "");
    u32 control = 11 << 10 | (u32)slot_id << 24;
    if (should_set_addr == false) {
        control |= (1 << 9);
    }
    ASSERT (pinput_context_buffer != NULLPTR, "");
    u32 high = XHCI_TO_DMA_HI (pinput_context_buffer, phost->ppcie->s_nDMAAddress);
    u32 low = XHCI_TO_DMA_LO (pinput_context_buffer, phost->ppcie->s_nDMAAddress);
    return xhci_command_manager_do_cmd (phost, pman, control, low, high, 0, NULLPTR);
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

static xhci_trb_t* xhci_ring_get_first_trb (xhci_device_t* phost, xhci_ring_t* pring) {
    return pring->pfirst_trb;
}

static xhci_trb_t* xhci_ring_get_enqueue_trb (xhci_device_t* phost, xhci_ring_t* pring) {
    ASSERT (pring->pfirst_trb != NULLPTR, "");
    ASSERT (pring->enqueue_idx < pring->trb_count, "");
    if (
    (pring->pfirst_trb[pring->enqueue_idx].control & XHCI_TRB_CONTROL_C) ==
    pring->cycle_state) {
        return NULLPTR;
    }
    return &pring->pfirst_trb[pring->enqueue_idx];
}

static void xhci_ring_inc_enqueue (xhci_device_t* phost, xhci_ring_t* pring) {
    ASSERT (pring->pfirst_trb != NULLPTR, "");
    ASSERT (pring->type == eXHCI_RING_TYPE_EVENT, "");
    ASSERT (pring->enqueue_idx < pring->trb_count, "");
    ASSERT (
    (pring->pfirst_trb[pring->enqueue_idx].control & XHCI_TRB_CONTROL_C) ==
    pring->cycle_state,
    "");

    if (++pring->enqueue_idx == pring->trb_count - 1) {
        xhci_trb_t* ptrb = &pring->pfirst_trb[pring->enqueue_idx];
        ptrb->control ^= XHCI_TRB_CONTROL_C;
        if (ptrb->control & XHCI_TRB_CONTROL_C) {
            pring->cycle_state ^= XHCI_TRB_CONTROL_C;
        }
        pring->enqueue_idx = 0;
    }
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
    // while (true) {
    wait_ms (1000);
    for (u32 i = 0; i < proothub->nports; ++i) {
        if (xhci_rootport_init (phost, &proothub->rootports[i]) == false) {
            // LOG_ERROR ("XHCI failed to init port [%u]", i + 1);
        }
    }
    //}
    return true;
}

static xhci_rootport_t*
xhci_rootport_create (xhci_device_t* host, xhci_rootport_t* rootport, u32 port_id) {
    memset (rootport, 0, sizeof (xhci_rootport_t));
    ASSERT (port_id >= 1 && port_id <= XHCI_CONFIG_MAX_PORTS, "XHCI invalid port passed to rootport");
    rootport->port_idx = port_id - 1;
    return rootport;
}

static bool xhci_rootport_init (xhci_device_t* phost, xhci_rootport_t* prootport) {
    if (xhci_rootport_is_connected (phost, prootport) == false) {
        LOG_ERROR ("XHCI port [%u] no device is connected", prootport->port_idx + 1);
        return false;
    }
    // return true;
    if (XHCI_IS_USB2_PORT (prootport->port_idx + 1)) {
        if (xhci_rootport_reset (phost, prootport, 100000) == false) {
            LOG_ERROR ("XHCI port [%u] failed to reset ", prootport->port_idx + 1);
            return false;
        }
    } else {
        if (xhci_rootport_waitfor_u0state (phost, prootport, 100000) == false) {
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
    ASSERT (prootport->pusb_dev == NULLPTR, "");
    prootport->pusb_dev = pdev;
    if (xhci_usb_device_init (phost, pdev) == false) {
        LOG_ERROR ("USB device failed to init");
        return false;
    }

    return true;
}

static bool xhci_rootport_reset (xhci_device_t* phost, xhci_rootport_t* prootport, u32 us_timeout) {
    ASSERT (prootport->port_idx < XHCI_CONFIG_MAX_PORTS, "XHCI invalid port index");
    u32 port_sc = pt_read32 (phost->pmmio, prootport->port_idx, XHCI_PT_PORTS_PORTSC_REG);
    port_sc |= (1 << 4);
    pt_write32 (phost->pmmio, prootport->port_idx, XHCI_PT_PORTS_PORTSC_REG, port_sc & ~(1 << 1));
    return op_wait32 (
    phost->pmmio, XHCI_OP_PORTS_BASE_REG + prootport->port_idx * (4 * 4) + XHCI_PT_PORTS_PORTSC_REG,
    (1 << 4), 0, us_timeout);
}

static bool xhci_rootport_is_connected (xhci_device_t* phost, xhci_rootport_t* prootport) {
    ASSERT (prootport->port_idx < XHCI_CONFIG_MAX_PORTS, "");
    u32 port = pt_read32 (phost->pmmio, prootport->port_idx, XHCI_PT_PORTS_PORTSC_REG);
    // LOG_INFO ("[%u] [0x%X]", prootport->port_idx + 1, port);
    return !!(port & (1 << 0));
}

static bool xhci_rootport_configure (xhci_device_t* phost, xhci_rootport_t* prootport) {

    return true;
}

static bool
xhci_rootport_waitfor_u0state (xhci_device_t* phost, xhci_rootport_t* prootport, u32 us_timeout) {
    return op_wait32 (
    phost->pmmio, XHCI_OP_PORTS_BASE_REG + prootport->port_idx * (4 * 4) + XHCI_PT_PORTS_PORTSC_REG,
    (0xF << 5), 0 << 5, us_timeout);
}

static usb_speed_t
xhci_rootport_get_portspeed (xhci_device_t* phost, xhci_rootport_t* prootport) {
    ASSERT (prootport->port_idx < XHCI_CONFIG_MAX_PORTS, "XHCI invalid port index");
    u32 port_sc = pt_read32 (phost->pmmio, prootport->port_idx, XHCI_PT_PORTS_PORTSC_REG);
    u8 speed = (port_sc & (0xF << 10)) >> 10;
    // LOG_INFO ("Speed [0x%X] [%u]", port_sc, (u32)speed);
    if (speed == 0 || (speed >= 1 && speed <= 4) == false) {
        return eUSB_SPEED_UNKNOWN;
    }
    return (usb_speed_t)((speed) < 3 ? ((speed)-1) ^ 1 : (speed)-1);
}

static xhci_usb_device_t*
xhci_usb_device_create (xhci_device_t* phost, xhci_rootport_t* prootport, usb_speed_t speed) {
    for (u32 i = 0; i < XHCI_CONFIG_MAX_PORTS; ++i) {
        if (is_usb_device_allocated[i] == false) {
            is_usb_device_allocated[i] = true;
            xhci_usb_device_t* dev     = &usb_devices_[i];
            memset (dev, 0, sizeof (xhci_usb_device_t));
            dev->pdev_context =
            xhci_dma_allocate (sizeof (xhci_device_context_t), BLOCK_ALIGN, PAGE_SIZE);
            return dev;
        }
    }
    return NULLPTR;
}

static bool xhci_usb_device_init (xhci_device_t* phost, xhci_usb_device_t* pdev) {
    if (pdev->pdev_context == NULLPTR) {
        return false;
    }
    s32 result =
    xhci_command_manager_enable_slot (phost, phost->pcommand_man, &pdev->slot_id);
    if (XHCI_CMD_SUCCESS (result) == false) {
        LOG_ERROR ("USB Device failed to enable slot");
        pdev->slot_id = 0;
        return false;
    }
    ASSERT (XHCI_IS_SLOTID (pdev->slot_id), "");
    xhci_slot_manager_assign_device (phost, phost->pslot_man, pdev, pdev->slot_id);
    xhci_input_context_t* pinput_context =
    xhci_usb_device_get_input_context_address_device (phost, pdev);
    ASSERT (pinput_context != NULLPTR, "");
    result = xhci_command_manager_address_device (
    phost, phost->pcommand_man, pinput_context, pdev->slot_id, true);
    if (XHCI_CMD_SUCCESS (result) == false) {
        LOG_ERROR ("USB Device cannot set address");
        return false;
    }
    wait_ms (50);
    pdev->dev_address = pdev->slot_id;
    LOG_INFO ("USB device address [%u]", (u32)pdev->dev_address);

    return true;
}

static xhci_input_context_t*
xhci_usb_device_get_input_context_address_device (xhci_device_t* phost, xhci_usb_device_t* pdev) {
    ASSERT (pdev->pinput_context_buffer == NULLPTR, "");
    // pdev->pinput_context_buf =
    // (u8*)align_allocate_set (sizeof (xhci_input_context_t) + XHCI_PAGE_SIZE - 1, 0, PAGE_SIZE);
    pdev->pinput_context_buffer =
    (u8*)align_allocate_set (sizeof (xhci_input_context_t), 0, PAGE_SIZE);
    ASSERT (pdev->pinput_context_buffer != NULLPTR, "");
    xhci_input_context_t* pinput_context = (xhci_input_context_t*)pdev->pinput_context_buffer;
    // set input control context
    pinput_context->control_context.add_context_flags = 3; // set A0 and A1
    // Set slot context
    pinput_context->dev_context.slot.roothub_port_num = pdev->pdev_context->slot.roothub_port_num;
    pinput_context->dev_context.slot.route_string = pdev->pdev_context->slot.route_string;
    pinput_context->dev_context.slot.speed =
    XHCI_USB_SPEED_TO_PSI (pdev->pdev_context->slot.speed);
    pinput_context->dev_context.slot.context_entries = 1;

    xhci_endpoint_context_t* pep0_context =
    &pinput_context->dev_context.endpoint_contexts[0];
    ASSERT (pep0_context != NULLPTR, "");
    xhci_ring_t* ptransfer_ring = pdev->endpoints[0].ptransfer_ring;
    ASSERT (ptransfer_ring != NULLPTR, "");
    pep0_context->trdequeue_ptr =
    XHCI_TO_DMA (ptransfer_ring->pfirst_trb, phost->ppcie->s_nDMAAddress) | (1 << 0);
    pep0_context->mult           = 0;
    pep0_context->max_ps_streams = 0;
    pep0_context->interval       = 0;
    pep0_context->cerr           = 0;
    pep0_context->eptype         = XHCI_EP_CONTEXT_EP_TYPE_CONTROL;
    pep0_context->max_burst_size = 0;
    pep0_context->avg_trb_length = 8;

    switch (pdev->pdev_context->slot.speed) {
    case eUSB_SPEED_LOW:
    case eUSB_SPEED_FULL: pep0_context->max_packet_size = 8; break;
    case eUSB_SPEED_HIGH: pep0_context->max_packet_size = 64; break;
    case eUSB_SPEED_SUPER: pep0_context->max_packet_size = 512; break;
    default: ASSERT (false, ""); break;
    }
    clean_invalidate_data_cache_vaddr ((uintptr_t)pinput_context, sizeof (*pinput_context));
    return pinput_context;
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
    mmio_space_init (host->pmmio, base_addr);
    u32 nmax_ports = (cap_read32 (host->pmmio, 0x04) & (0xFF << 24)) >> 24;
    ASSERT (nmax_ports > 0 && nmax_ports <= XHCI_CONFIG_MAX_PORTS, "USB number of ports exceed max");
    u32 nmax_scratch_pads = (cap_read32 (host->pmmio, 0x08) & (0x1F << 27)) >> 27;
    ASSERT (!(cap_read32 (host->pmmio, 0x10) & (1 << 2)), "USB mmio cap fail");
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
    xhci_dma_allocate (XHCI_PAGE_SIZE * nmax_scratch_pads, XHCI_PAGE_SIZE, PAGE_SIZE);
    host->pscratchpad_buffer_array =
    (u64*)xhci_dma_allocate (sizeof (64) * nmax_scratch_pads, BLOCK_ALIGN, PAGE_SIZE);

    ASSERT (host->pscratchpad_buffers != NULLPTR, "XHCI failed to allocate scratch pads");
    ASSERT (host->pscratchpad_buffer_array != NULLPTR, "XHCI failed to allocate scratch pads");

    for (u32 i = 0; i < nmax_scratch_pads; ++i) {
        host->pscratchpad_buffer_array[i] =
        XHCI_TO_DMA (host->pscratchpad_buffers, host->ppcie->s_nDMAAddress) +
        XHCI_PAGE_SIZE * i;
    }
    xhci_slot_manager_assign_scratch_pad_buffer_array (
    host, host->pslot_man, host->pscratchpad_buffer_array);
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
