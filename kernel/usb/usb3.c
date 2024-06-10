#include "usb/usb3.h"


#include "peripherals/vpu.h"

#define MEM_PCIE_RANGE_START_VIRTUAL MEM_PCIE_RANGE_START
#define MEM_PCIE_RANGE_END_VIRTUAL \
    (MEM_PCIE_RANGE_START_VIRTUAL + MEM_PCIE_RANGE_SIZE - 1UL)
#define ARM_XHCI0_BASE      MEM_PCIE_RANGE_START_VIRTUAL
#define ARM_XHCI0_END       (ARM_XHCI0_BASE + 0x0FFF)

#define XHCI_PCI_CLASS_CODE 0xC0330
#define XHCI_PCIE_SLOT      0
#define XHCI_PCIE_FUNC      0

static xhci_device_t xhci_device_ = { 0 };
static mmio_space_t mmio_space_   = { 0 };

static mmio_space_t* mmio_space_create (void);
static void mmio_space_init (mmio_space_t* mmio, uintptr_t base_addr);
static u32 cap_read32_raw (mmio_space_t* mmio, u32 addr);
static u32 cap_read32 (mmio_space_t* mmio, u32 offset);
static void op_write32 (mmio_space_t* mmio, u32 offset, u32 value);
static u32 op_read32 (mmio_space_t* mmio, u32 offset);
static bool op_wait32 (mmio_space_t* mmio, u32 offset, u32 mask, u32 expected_value, s32 us_timeout);

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

static bool op_wait32 (mmio_space_t* mmio, u32 offset, u32 mask, u32 expected_value, s32 us_timeout) {
    do {
        if ((op_read32 (mmio, offset) & mask) == expected_value) {
            return true;
        }
    } while (us_timeout-- > 0);
    return false;
}

static u32 op_read32 (mmio_space_t* mmio, u32 offset) {
    return read32 (mmio->op_base + offset);
}

static mmio_space_t* mmio_space_create (void) {
    memset (&mmio_space_, 0, sizeof (mmio_space_t));
    return &mmio_space_;
}

static void mmio_space_init (mmio_space_t* mmio, uintptr_t base_addr) {
    // TODO: raspberry pi hangs somewhere is this function
    mmio->base    = base_addr;
    mmio->op_base = base_addr + read8 (base_addr + 0x00);
    mmio->db_base = base_addr + (cap_read32_raw (mmio, 0x14) & 0xFFFFFFFC);
    mmio->rt_base = base_addr + (cap_read32_raw (mmio, 0x18) & 0xFFFFFFE0);
    mmio->pt_base = mmio->op_base + 0x400;

    mmio->hcx_params[0] = cap_read32_raw (mmio, 0x04);
    mmio->hcx_params[1] = cap_read32_raw (mmio, 0x08);
    mmio->hcx_params[2] = cap_read32_raw (mmio, 0x0C);
    mmio->hcx_params[3] = cap_read32_raw (mmio, 0x10);

    mmio->ecp_base =
    base_addr + ((cap_read32_raw (mmio, 0x10) & (0xFFFF << 16)) >> 16 << 2);
}

static bool hardware_reset (xhci_device_t* host);


static bool hardware_reset (xhci_device_t* host) {
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


xhci_device_t* xhci_device_create (void) {
    memset (&xhci_device_, 0, sizeof (xhci_device_t));
    return &xhci_device_;
}

bool xhci_device_init (xhci_device_t* host) {
    host->ppcie = pcie_hostbridge_create ();
    host->pmmio = mmio_space_create ();

    if (pcie_hostbridge_init (host->ppcie) == false) {
        LOG_ERROR ("USB failed to init pcie hostbridge");
        return false;
    }
    // load VIA VL805 firmware after PCIe reset
    VPU_CMB_BUFF_INIT (0x00030058, 4, 4, (1 << 20) | (0 << 15) | (0 << 12));
    vpu_status_t status = vpu_call (&cmd_buffer, VPU_MBOX_CH_PROP);
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
    LOG_INFO ("1");
    mmio_space_init (host->pmmio, base_addr);
    LOG_INFO ("2");

    u32 nmax_ports = cap_read32 (host->pmmio, 0x04) & 0xFF;
    LOG_INFO ("6");
    ASSERT (64 <= nmax_ports, "USB number of ports exceed max");
    u32 nmax_scratch_pads = (cap_read32 (host->pmmio, 0x08) & (0x1F << 27)) >> 27;
    LOG_INFO ("7");
    ASSERT (cap_read32 (host->pmmio, 0x10) & (1 << 2), "USB mmio cap fail");
    LOG_INFO ("8");
    LOG_INFO ("USB # of ports %u # of scratch pads %u", nmax_ports, nmax_scratch_pads);

    if (hardware_reset (host) == false) {
        LOG_ERROR ("USB failed to reset hardware");
        return false;
    }

    return true;
}
