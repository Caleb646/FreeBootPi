//
// bcmpciehostbridge.cpp
//
// This driver has been ported from the Linux driver which is:
//	drivers/pci/controller/pcie-brcmstb.c
//	Copyright (C) 2009 - 2019 Broadcom
//	Licensed under GPL-2.0+
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2019-2023  R. Stange <rsta2@o2online.de>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "usb/pcie_hostbridge.h"
#include "peripherals/timer.h"

#define PCIE_GEN                                 2

/* BRCM_PCIE_CAP_REGS - Offset for the mandatory capability config regs */
#define BRCM_PCIE_CAP_REGS                       0x00ac

/*
 * Broadcom Settop Box PCIe Register Offsets. The names are from
 * the chip's RDB and we use them here so that a script can correlate
 * this code and the RDB to prevent discrepancies.
 */
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1  0x0188
#define PCIE_RC_CFG_PRIV1_ID_VAL3                0x043c
#define PCIE_RC_DL_MDIO_ADDR                     0x1100
#define PCIE_RC_DL_MDIO_WR_DATA                  0x1104
#define PCIE_RC_DL_MDIO_RD_DATA                  0x1108
#define PCIE_MISC_MISC_CTRL                      0x4008
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO         0x400c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI         0x4010
#define PCIE_MISC_RC_BAR1_CONFIG_LO              0x402c
#define PCIE_MISC_RC_BAR1_CONFIG_HI              0x4030
#define PCIE_MISC_RC_BAR2_CONFIG_LO              0x4034
#define PCIE_MISC_RC_BAR2_CONFIG_HI              0x4038
#define PCIE_MISC_RC_BAR3_CONFIG_LO              0x403c
#define PCIE_MISC_MSI_BAR_CONFIG_LO              0x4044
#define PCIE_MISC_MSI_BAR_CONFIG_HI              0x4048
#define PCIE_MISC_MSI_DATA_CONFIG                0x404c
#define PCIE_MISC_EOI_CTRL                       0x4060
#define PCIE_MISC_PCIE_CTRL                      0x4064
#define PCIE_MISC_PCIE_STATUS                    0x4068
#define PCIE_MISC_REVISION                       0x406c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT 0x4070
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI    0x4080
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI   0x4084
#if RASPPI == 4
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG 0x4204
#define PCIE_INTR2_CPU_BASE            0x4300
#else
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG 0x4304
#define PCIE_INTR2_CPU_BASE            0x4400
#endif
#define PCIE_MSI_INTR2_BASE                                            0x4500
/*
 * Broadcom Settop Box PCIe Register Field shift and mask info. The
 * names are from the chip's RDB and we use them here so that a script
 * can correlate this code and the RDB to prevent discrepancies.
 */
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK  0xc
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_SHIFT 0x2
#define PCIE_RC_CFG_PRIV1_ID_VAL3_CLASS_CODE_MASK                      0xffffff
#define PCIE_RC_CFG_PRIV1_ID_VAL3_CLASS_CODE_SHIFT                     0x0
#define PCIE_MISC_MISC_CTRL_RCB_MPS_MODE_MASK                          0x400
#define PCIE_MISC_MISC_CTRL_RCB_MPS_MODE_SHIFT                         0xa
#define PCIE_MISC_MISC_CTRL_SCB_ACCESS_EN_MASK                         0x1000
#define PCIE_MISC_MISC_CTRL_SCB_ACCESS_EN_SHIFT                        0xc
#define PCIE_MISC_MISC_CTRL_CFG_READ_UR_MODE_MASK                      0x2000
#define PCIE_MISC_MISC_CTRL_CFG_READ_UR_MODE_SHIFT                     0xd
#define PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_MASK                        0x300000
#define PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_SHIFT                       0x14
#define PCIE_MISC_MISC_CTRL_SCB0_SIZE_MASK                             0xf8000000
#define PCIE_MISC_MISC_CTRL_SCB0_SIZE_SHIFT                            0x1b
#define PCIE_MISC_MISC_CTRL_SCB1_SIZE_MASK                             0x7c00000
#define PCIE_MISC_MISC_CTRL_SCB1_SIZE_SHIFT                            0x16
#define PCIE_MISC_MISC_CTRL_SCB2_SIZE_MASK                             0x1f
#define PCIE_MISC_MISC_CTRL_SCB2_SIZE_SHIFT                            0x0
#define PCIE_MISC_RC_BAR1_CONFIG_LO_SIZE_MASK                          0x1f
#define PCIE_MISC_RC_BAR1_CONFIG_LO_SIZE_SHIFT                         0x0
#define PCIE_MISC_RC_BAR2_CONFIG_LO_SIZE_MASK                          0x1f
#define PCIE_MISC_RC_BAR2_CONFIG_LO_SIZE_SHIFT                         0x0
#define PCIE_MISC_RC_BAR3_CONFIG_LO_SIZE_MASK                          0x1f
#define PCIE_MISC_RC_BAR3_CONFIG_LO_SIZE_SHIFT                         0x0
#define PCIE_MISC_PCIE_CTRL_PCIE_PERSTB_MASK                           0x4
#define PCIE_MISC_PCIE_CTRL_PCIE_PERSTB_SHIFT                          0x2
#define PCIE_MISC_PCIE_CTRL_PCIE_L23_REQUEST_MASK                      0x1
#define PCIE_MISC_PCIE_CTRL_PCIE_L23_REQUEST_SHIFT                     0x0
#define PCIE_MISC_PCIE_STATUS_PCIE_PORT_MASK                           0x80
#define PCIE_MISC_PCIE_STATUS_PCIE_PORT_SHIFT                          0x7
#define PCIE_MISC_PCIE_STATUS_PCIE_DL_ACTIVE_MASK                      0x20
#define PCIE_MISC_PCIE_STATUS_PCIE_DL_ACTIVE_SHIFT                     0x5
#define PCIE_MISC_PCIE_STATUS_PCIE_PHYLINKUP_MASK                      0x10
#define PCIE_MISC_PCIE_STATUS_PCIE_PHYLINKUP_SHIFT                     0x4
#define PCIE_MISC_PCIE_STATUS_PCIE_LINK_IN_L23_MASK                    0x40
#define PCIE_MISC_PCIE_STATUS_PCIE_LINK_IN_L23_SHIFT                   0x6
#define PCIE_MISC_REVISION_MAJMIN_MASK                                 0xffff
#define PCIE_MISC_REVISION_MAJMIN_SHIFT                                0
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_LIMIT_MASK            0xfff00000
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_LIMIT_SHIFT           0x14
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_MASK             0xfff0
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_SHIFT            0x4
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_NUM_MASK_BITS         0xc
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI_BASE_MASK                0xff
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI_BASE_SHIFT               0x0
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI_LIMIT_MASK              0xff
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI_LIMIT_SHIFT             0x0
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_CLKREQ_DEBUG_ENABLE_MASK        0x2
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_CLKREQ_DEBUG_ENABLE_SHIFT       0x1
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_SERDES_IDDQ_MASK                0x08000000
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_SERDES_IDDQ_SHIFT               0x1b
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_CLKREQ_L1SS_ENABLE_MASK         0x00200000
#define PCIE_RGR1_SW_INIT_1_PERST_MASK                                 0x1
#define PCIE_RGR1_SW_INIT_1_PERST_SHIFT                                0x0

#define BRCM_INT_PCI_MSI_NR                                            32
#define BRCM_PCIE_HW_REV_33                                            0x0303

#define BRCM_MSI_TARGET_ADDR_LT_4GB                                    0x0fffffffcULL
#define BRCM_MSI_TARGET_ADDR_GT_4GB                                    0xffffffffcULL

#define BURST_SIZE_128                                                 0
#define BURST_SIZE_256                                                 1
#define BURST_SIZE_512                                                 2

/* Offsets from PCIE_INTR2_CPU_BASE */
#define STATUS                                                         0x0
#define SET                                                            0x4
#define CLR                                                            0x8
#define MASK_STATUS                                                    0xc
#define MASK_SET                                                       0x10
#define MASK_CLR                                                       0x14

#define PCIE_BUSNUM_SHIFT                                              20
#define PCIE_SLOT_SHIFT                                                15
#define PCIE_FUNC_SHIFT                                                12

#define DATA_ENDIAN                                                    0
#define MMIO_ENDIAN                                                    0

#define MDIO_PORT0                                                     0x0
#define MDIO_DATA_MASK                                                 0x7fffffff
#define MDIO_DATA_SHIFT                                                0x0
#define MDIO_PORT_MASK                                                 0xf0000
#define MDIO_PORT_SHIFT                                                0x16
#define MDIO_REGAD_MASK                                                0xffff
#define MDIO_REGAD_SHIFT                                               0x0
#define MDIO_CMD_MASK                                                  0xfff00000
#define MDIO_CMD_SHIFT                                                 0x14
#define MDIO_CMD_READ                                                  0x1
#define MDIO_CMD_WRITE                                                 0x0
#define MDIO_DATA_DONE_MASK                                            0x80000000
#define MDIO_RD_DONE(x)                                                (((x)&MDIO_DATA_DONE_MASK) ? 1 : 0)
#define MDIO_WT_DONE(x)                                                (((x)&MDIO_DATA_DONE_MASK) ? 0 : 1)
#define SSC_REGS_ADDR                                                  0x1100
#define SET_ADDR_OFFSET                                                0x1f
#define SSC_CNTL_OFFSET                                                0x2
#define SSC_CNTL_OVRD_EN_MASK                                          0x8000
#define SSC_CNTL_OVRD_EN_SHIFT                                         0xf
#define SSC_CNTL_OVRD_VAL_MASK                                         0x4000
#define SSC_CNTL_OVRD_VAL_SHIFT                                        0xe
#define SSC_STATUS_OFFSET                                              0x1
#define SSC_STATUS_SSC_MASK                                            0x400
#define SSC_STATUS_SSC_SHIFT                                           0xa
#define SSC_STATUS_PLL_LOCK_MASK                                       0x800
#define SSC_STATUS_PLL_LOCK_SHIFT                                      0xb

#define PCIE_RGR1_SW_INIT_1                                            0x9210
#define RGR1_SW_INIT_1_INIT_GENERIC_MASK                               0x2
#define RGR1_SW_INIT_1_INIT_GENERIC_SHIFT                              0x1
#define PCIE_EXT_CFG_INDEX                                             0x9000
#define PCIE_EXT_CFG_DATA                                              0x8000

#define bcm_readl(a)                                                   read32 (a)
#define bcm_writel(d, a)                                               write32 (a, d)
#define bcm_readw(a)                                                   read16 (a)
#define bcm_writew(d, a)                                               write16 (a, d)

/* These macros extract/insert fields to host controller's register set. */
#define RD_FLD(m_base, reg, field) \
    rd_fld (m_base + reg, reg##_##field##_MASK, reg##_##field##_SHIFT)
#define WR_FLD(m_base, reg, field, val) \
    wr_fld (m_base + reg, reg##_##field##_MASK, reg##_##field##_SHIFT, val)
#define WR_FLD_RB(m_base, reg, field, val) \
    wr_fld_rb (m_base + reg, reg##_##field##_MASK, reg##_##field##_SHIFT, val)
#define WR_FLD_WITH_OFFSET(m_base, off, reg, field, val) \
    wr_fld (m_base + reg + off, reg##_##field##_MASK, reg##_##field##_SHIFT, val)
#define EXTRACT_FIELD(val, reg, field) \
    ((val & reg##_##field##_MASK) >> reg##_##field##_SHIFT)
#define INSERT_FIELD(val, reg, field, field_val) \
    ((val & ~reg##_##field##_MASK) | (reg##_##field##_MASK & (field_val << reg##_##field##_SHIFT)))

#define PCI_BUS(n)            (n)

#define PCI_DEVFN(slot, func) ((((slot)&0x1f) << 3) | ((func)&0x07))
#define PCI_SLOT(devfn)       (((devfn) >> 3) & 0x1f)
#define PCI_FUNC(devfn)       ((devfn)&0x07)

#define lower_32_bits(v)      (v & 0xFFFFFFFFU)
#define upper_32_bits(v)      (v >> 32)

static pcie_hostbridge_t host_bridge_ = { 0 };
static pcie_msi_data_t msi_data_      = { 0 };

#define ARM_PCIE_HOST_BASE 0xFD500000
#define ARM_PCIE_HOST_END  (ARM_PCIE_HOST_BASE + 0x930F)

static int pcie_probe (pcie_hostbridge_t* host);
static int pcie_setup (pcie_hostbridge_t* host);
static int enable_bridge (pcie_hostbridge_t* host);
static int enable_device (pcie_hostbridge_t* host, u32 nClassCode, unsigned nSlot, unsigned nFunc);
static int pcie_set_pci_ranges (pcie_hostbridge_t* host);
static int pcie_set_dma_ranges (pcie_hostbridge_t* host);
static void
pcie_set_outbound_win (pcie_hostbridge_t* host, unsigned win, u64 cpu_addr, u64 pcie_addr, u64 size);
static uintptr_t
pcie_map_conf (pcie_hostbridge_t* host, unsigned busnr, unsigned devfn, int where);
static uintptr_t find_pci_capability (uintptr_t nPCIConfig, u8 uchCapID);
static void pcie_bridge_sw_init_set (pcie_hostbridge_t* host, unsigned val);
static void pcie_perst_set (pcie_hostbridge_t* host, unsigned int val);
static bool pcie_link_up (pcie_hostbridge_t* host);
static bool pcie_rc_mode (pcie_hostbridge_t* host);
static int pcie_enable_msi (pcie_hostbridge_t* host, pcie_msi_handler_t* pHandler, void* pParam);
static void msi_set_regs (pcie_msi_data_t* msi);
static int cfg_index (int busnr, int devfn, int reg);
static void set_gen (uintptr_t base, int gen);
static const char* link_speed_to_str (int s);
static int encode_ibar_size (u64 size);
static u32 rd_fld (uintptr_t p, u32 mask, int shift);
static void wr_fld (uintptr_t p, u32 mask, int shift, u32 val);
static void wr_fld_rb (uintptr_t p, u32 mask, int shift, u32 val);
// static void InterruptHandler (void* pParam);
static void usleep_range (unsigned min, unsigned max);
static void msleep (unsigned ms);
static int ilog2 (u64 v);


pcie_hostbridge_t* pcie_hostbridge_create (void) {
    memset (&host_bridge_, 0, sizeof (pcie_hostbridge_t));
    host_bridge_.m_base = ARM_PCIE_HOST_BASE;
    return &host_bridge_;
}

// CBcmPCIeHostBridge::~CBcmPCIeHostBridge (void)
// {
// 	if (m_msi != 0)
// 	{
// 		DisconnectMSI ();
// 	}

// 	m_pInterrupt = 0;
// }

bool pcie_hostbridge_init (pcie_hostbridge_t* host) {
    int ret = pcie_probe (host);
    if (ret) {
        LOG_ERROR ("PCIe cannot initialize PCIe bridge");
        return false;
    }
    ret = enable_bridge (host);
    if (ret) {
        LOG_ERROR ("PCIe Cannot enable PCIe bridge");
        return false;
    }
    return true;
}

bool pcie_hostbridge_enable_device (pcie_hostbridge_t* host, u32 nClassCode, unsigned nSlot, unsigned nFunc) {
    return !enable_device (host, nClassCode, nSlot, nFunc);
}

bool pcie_hostbridge_connect_msi (pcie_hostbridge_t* host, pcie_msi_handler_t* pHandler, void* pParam) {
    return !pcie_enable_msi (host, pHandler, pParam);
}

void pcie_hostbridge_disconnect_msi (pcie_hostbridge_t* host) {
    bcm_writel (0xffffffff, host->m_msi->intr_base + MASK_SET);
    bcm_writel (0, host->m_msi->base + PCIE_MISC_MSI_BAR_CONFIG_LO);

    // assert (m_pInterrupt != 0);
    // m_pInterrupt->DisconnectIRQ (ARM_IRQ_PCIE_HOST_MSI);

    // delete m_msi;
    host->m_msi = NULLPTR;
}

static int pcie_probe (pcie_hostbridge_t* host) {
    int ret = pcie_set_pci_ranges (host);
    if (ret)
        return ret;

    ret = pcie_set_dma_ranges (host);
    if (ret)
        return ret;

    return pcie_setup (host);
}

static int pcie_setup (pcie_hostbridge_t* host) {
    unsigned int scb_size_val;
    u64 rc_bar2_offset, rc_bar2_size, total_mem_size = 0;
    u32 tmp;
    int i, j, limit;
    /* Reset the bridge */
    pcie_bridge_sw_init_set (host, 1);
    /* Ensure that the fundamental reset is asserted */
    pcie_perst_set (host, 1);
    usleep_range (100, 200);
    /* Take the bridge out of reset */
    pcie_bridge_sw_init_set (host, 0);

    WR_FLD_RB (host->m_base, PCIE_MISC_HARD_PCIE_HARD_DEBUG, SERDES_IDDQ, 0);
    /* Wait for SerDes to be stable */
    usleep_range (100, 200);

    /* Grab the PCIe hw revision number */
    tmp         = bcm_readl (host->m_base + PCIE_MISC_REVISION);
    host->m_rev = EXTRACT_FIELD (tmp, PCIE_MISC_REVISION, MAJMIN);

    /* Set SCB_MAX_BURST_SIZE, CFG_READ_UR_MODE, SCB_ACCESS_EN, RCB_MPS_MODE */
    tmp = bcm_readl (host->m_base + PCIE_MISC_MISC_CTRL);
    tmp = INSERT_FIELD (tmp, PCIE_MISC_MISC_CTRL, SCB_ACCESS_EN, 1);
    tmp = INSERT_FIELD (tmp, PCIE_MISC_MISC_CTRL, CFG_READ_UR_MODE, 1);

    tmp = INSERT_FIELD (tmp, PCIE_MISC_MISC_CTRL, MAX_BURST_SIZE, BURST_SIZE_128);

    tmp = INSERT_FIELD (tmp, PCIE_MISC_MISC_CTRL, MAX_BURST_SIZE, BURST_SIZE_256);
    tmp = INSERT_FIELD (tmp, PCIE_MISC_MISC_CTRL, RCB_MPS_MODE, 1);

    bcm_writel (tmp, host->m_base + PCIE_MISC_MISC_CTRL);

    /*
     * Set up inbound memory view for the EP (called RC_BAR2,
     * not to be confused with the BARs that are advertised by
     * the EP).
     *
     * The PCIe host controller by design must set the inbound
     * viewport to be a contiguous arrangement of all of the
     * system's memory.  In addition, its size mut be a power of
     * two.  Further, the MSI target address must NOT be placed
     * inside this region, as the decoding logic will consider its
     * address to be inbound memory traffic.  To further
     * complicate matters, the viewport must start on a
     * pcie-address that is aligned on a multiple of its size.
     * If a portion of the viewport does not represent system
     * memory -- e.g. 3GB of memory requires a 4GB viewport --
     * we can map the outbound memory in or after 3GB and even
     * though the viewport will overlap the outbound memory
     * the controller will know to send outbound memory downstream
     * and everything else upstream.
     */
    ASSERT (host->m_num_dma_ranges == 1, "PCIe invalid dma ranges");
    host->m_scb_size[0] = host->m_dma_ranges[0].size; // must be a power of 2
    host->m_num_scbs    = host->m_num_dma_ranges;
    rc_bar2_offset      = host->m_dma_ranges[0].pcie_addr;

    ASSERT (host->m_num_scbs == 1, "PCIe invalid num scbs");
    total_mem_size = host->m_scb_size[0];
    rc_bar2_size   = total_mem_size; // must be a power of 2

    /* Verify the alignment is correct */
    ASSERT (!(rc_bar2_offset & (rc_bar2_size - 1)), "PCIe invalid");
    /*
     * Position the MSI target low if possible.
     *
     * TO DO: Consider outbound window when choosing MSI target and
     * verifying configuration.
     */
    u64 msi_target_addr = BRCM_MSI_TARGET_ADDR_LT_4GB;
    if (rc_bar2_offset <= msi_target_addr && rc_bar2_offset + rc_bar2_size > msi_target_addr)
        msi_target_addr = BRCM_MSI_TARGET_ADDR_GT_4GB;

    host->m_msi_target_addr = msi_target_addr;


    tmp = lower_32_bits (rc_bar2_offset);
    tmp = INSERT_FIELD (tmp, PCIE_MISC_RC_BAR2_CONFIG_LO, SIZE, encode_ibar_size (rc_bar2_size));
    bcm_writel (tmp, host->m_base + PCIE_MISC_RC_BAR2_CONFIG_LO);
    bcm_writel (upper_32_bits (rc_bar2_offset), host->m_base + PCIE_MISC_RC_BAR2_CONFIG_HI);

    scb_size_val = host->m_scb_size[0] ? ilog2 (host->m_scb_size[0]) - 15 : 0xf; /* 0xf is 1GB */
    WR_FLD (host->m_base, PCIE_MISC_MISC_CTRL, SCB0_SIZE, scb_size_val);
    ASSERT (host->m_num_scbs == 1, "PCI num scbs is 1"); // do not set fields SCB1_SIZE and SCB2_SIZE

    /* disable the PCIe->GISB memory window (RC_BAR1) */
    WR_FLD (host->m_base, PCIE_MISC_RC_BAR1_CONFIG_LO, SIZE, 0);

    /* disable the PCIe->SCB memory window (RC_BAR3) */
    WR_FLD (host->m_base, PCIE_MISC_RC_BAR3_CONFIG_LO, SIZE, 0);

    /* clear any interrupts we find on boot */
    bcm_writel (0xffffffff, host->m_base + PCIE_INTR2_CPU_BASE + CLR);
    (void)bcm_readl (host->m_base + PCIE_INTR2_CPU_BASE + CLR);

    /* Mask all interrupts since we are not handling any yet */
    bcm_writel (0xffffffff, host->m_base + PCIE_INTR2_CPU_BASE + MASK_SET);
    (void)bcm_readl (host->m_base + PCIE_INTR2_CPU_BASE + MASK_SET);

    set_gen (host->m_base, PCIE_GEN);
    // Start link follows:

    /* Unassert the fundamental reset */
    pcie_perst_set (host, 0);

    /*
     * Wait for 100ms after PERST# deassertion; see PCIe CEM specification
     * sections 2.2, PCIe r5.0, 6.6.1.
     */
    msleep (100);

    /*
     * Give the RC/EP time to wake up, before trying to configure RC.
     * Intermittently check status for link-up, up to a total of 100ms
     * when we don't know if the device is there, and up to 1000ms if
     * we do know the device is there.
     */
    limit = 100;
    for (i = 1, j = 0; j < limit && !pcie_link_up (host); j += i, i = i * 2)
        msleep (i + j > limit ? limit - j : i);

    if (!pcie_link_up (host)) {
        LOG_ERROR ("PCIe link down");
        return -1;
    }

    if (!pcie_rc_mode (host)) {
        LOG_ERROR ("PCIe misconfigured; is in EP mode");
        return -1;
    }

    // This was done earlier for RASPPI >= 5
    for (i = 0; i < host->m_num_out_wins; i++)
        pcie_set_outbound_win (
        host, i, host->m_out_wins[i].cpu_addr, host->m_out_wins[i].pcie_addr,
        host->m_out_wins[i].size);

    /*
     * For config space accesses on the RC, show the right class for
     * a PCIe-PCIe bridge (the default setting is to be EP mode).
     */
    WR_FLD_RB (host->m_base, PCIE_RC_CFG_PRIV1_ID_VAL3, CLASS_CODE, 0x060400);
#define PCI_EXP_LNKSTA           18
#define PCI_EXP_LNKSTA_CLS       0x000f /* Current Link Speed */
#define PCI_EXP_LNKSTA_NLW       0x03f0 /* Negotiated Link Width */
#define PCI_EXP_LNKSTA_NLW_SHIFT 4      /* start of NLW mask in link status */
    u16 lnksta = bcm_readw (host->m_base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKSTA);
    u16 cls    = lnksta & PCI_EXP_LNKSTA_CLS;
    u16 nlw    = (lnksta & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
    LOG_INFO ("PCIe link up, %s Gbps x%u", link_speed_to_str (cls), nlw);
    /* PCIe->SCB endian mode for BAR */
    /* field ENDIAN_MODE_BAR2 = DATA_ENDIAN */
    WR_FLD_RB (host->m_base, PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1, ENDIAN_MODE_BAR2, DATA_ENDIAN);
    /*
     * Refclk from RC should be gated with CLKREQ# input when ASPM L0s,L1
     * is enabled =>  setting the CLKREQ_DEBUG_ENABLE field to 1.
     */
    WR_FLD_RB (host->m_base, PCIE_MISC_HARD_PCIE_HARD_DEBUG, CLKREQ_DEBUG_ENABLE, 1);
    return 0;
}

static int enable_bridge (pcie_hostbridge_t* host) {
    uintptr_t conf = pcie_map_conf (host, PCI_BUS (0), PCI_DEVFN (0, 0), 0);
    if (!conf)
        return -1;
#define PCI_CLASS_REVISION     0x08 /* High 24 bits are class, low 8 revision */
#define PCI_HEADER_TYPE        0x0e /* 8 bits */
#define PCI_HEADER_TYPE_BRIDGE 1
#define PCI_CACHE_LINE_SIZE    0x0c /* 8 bits */
#define PCI_SECONDARY_BUS      0x19 /* Secondary bus number */
#define PCI_SUBORDINATE_BUS    0x1a /* Highest bus number behind the bridge */
#define PCI_MEMORY_BASE        0x20 /* Memory range behind */
#define PCI_MEMORY_LIMIT       0x22
#define PCI_BRIDGE_CONTROL     0x3e
/* Enable parity detection on secondary interface */
#define PCI_BRIDGE_CTL_PARITY  0x01
#define PCI_CAP_LIST_ID        0      /* Capability ID */
#define PCI_CAP_ID_EXP         0x10   /* PCI Express */
#define PCI_EXP_RTCTL          28     /* Root Control */
#define PCI_EXP_RTCTL_CRSSVE   0x0010 /* CRS Software Visibility Enable */
#define PCI_COMMAND            0x04   /* 16 bits */
#define PCI_COMMAND_MEMORY     0x2    /* Enable response in Memory space */
#define PCI_COMMAND_MASTER     0x4    /* Enable bus mastering */
#define PCI_COMMAND_PARITY     0x40   /* Enable parity checking */
#define PCI_COMMAND_SERR       0x100  /* Enable SERR */


    if (read32 (conf + PCI_CLASS_REVISION) >> 8 != 0x060400 || read8 (conf + PCI_HEADER_TYPE) != PCI_HEADER_TYPE_BRIDGE)
        return -1;
    write8 (conf + PCI_CACHE_LINE_SIZE, 64 / 4); // TODO: get this from cache config

    write8 (conf + PCI_SECONDARY_BUS, PCI_BUS (1));
    write8 (conf + PCI_SUBORDINATE_BUS, PCI_BUS (1));

    write16 (conf + PCI_MEMORY_BASE, MEM_PCIE_RANGE_PCIE_START >> 16);
    write16 (conf + PCI_MEMORY_LIMIT, MEM_PCIE_RANGE_PCIE_START >> 16);

    write8 (conf + PCI_BRIDGE_CONTROL, PCI_BRIDGE_CTL_PARITY);

    ASSERT (read8 (conf + BRCM_PCIE_CAP_REGS + PCI_CAP_LIST_ID) == PCI_CAP_ID_EXP, "PCIe cap id not equal to expected value");
    write8 (conf + BRCM_PCIE_CAP_REGS + PCI_EXP_RTCTL, PCI_EXP_RTCTL_CRSSVE);

    write16 (conf + PCI_COMMAND, PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_PARITY | PCI_COMMAND_SERR);
    return 0;
}

static int enable_device (pcie_hostbridge_t* host, u32 nClassCode, unsigned nSlot, unsigned nFunc) {
    ASSERT (nClassCode >> 24 == 0, "PCIe");
    ASSERT (nSlot < 32, "PCIe number of slots is not < 32");
    ASSERT (nFunc < 8, "PCIe number of functions is not < 8");

    uintptr_t conf = pcie_map_conf (host, PCI_BUS (1), PCI_DEVFN (nSlot, nFunc), 0);
    if (!conf)
        return -1;

#define PCI_HEADER_TYPE_NORMAL 0

    if (read32 (conf + PCI_CLASS_REVISION) >> 8 != nClassCode || read8 (conf + PCI_HEADER_TYPE) != PCI_HEADER_TYPE_NORMAL)
        return -1;

    write8 (conf + PCI_CACHE_LINE_SIZE, 64 / 4); // TODO: get this from cache config

#define PCI_BASE_ADDRESS_0           0x10 /* 32 bits */
#define PCI_BASE_ADDRESS_1           0x14 /* 32 bits [htype 0,1 only] */
#define PCI_BASE_ADDRESS_MEM_TYPE_64 0x04 /* 64 bit address */
#define PCI_INTERRUPT_PIN            0x3d /* 8 bits */

    write32 (conf + PCI_BASE_ADDRESS_0, lower_32_bits (MEM_PCIE_RANGE_PCIE_START) | PCI_BASE_ADDRESS_MEM_TYPE_64);
    write32 (conf + PCI_BASE_ADDRESS_1, upper_32_bits (MEM_PCIE_RANGE_PCIE_START));

    // Ensure, that we can use INTA.
    u8 uchIntPin = read8 (conf + PCI_INTERRUPT_PIN);
    if (uchIntPin != 1) {
        LOG_WARN ("PCIe INTA was NOT enabled");
        write8 (conf + PCI_INTERRUPT_PIN, 1);
    }

    write16 (conf + PCI_COMMAND, PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_PARITY | PCI_COMMAND_SERR);

    return 0;
}

static int pcie_set_pci_ranges (pcie_hostbridge_t* host) {
    ASSERT (host->m_num_out_wins == 0, "PCIe num out wins");

    host->m_out_wins[0].cpu_addr  = MEM_PCIE_RANGE_START;
    host->m_out_wins[0].pcie_addr = MEM_PCIE_RANGE_PCIE_START;
    host->m_out_wins[0].size      = MEM_PCIE_RANGE_SIZE;
    host->m_num_out_wins          = 1;

    return 0;
}

static int pcie_set_dma_ranges (pcie_hostbridge_t* host) {
    ASSERT (host->m_num_dma_ranges == 0, "PCIe num dma ranges is 0");
#define MEM_PCIE_DMA_RANGE_PCIE_START 0UL // mapping on PCIe side
#define MEM_PCIE_DMA_RANGE_START      0UL
#define MEGABYTE                      0x100000
    host->m_dma_ranges[0].pcie_addr = MEM_PCIE_DMA_RANGE_PCIE_START;
    host->m_dma_ranges[0].cpu_addr  = MEM_PCIE_DMA_RANGE_START;
    host->m_dma_ranges[0].size      = (u64)(4096 * 2) * MEGABYTE;
    host->m_num_dma_ranges          = 1;
    host->s_nDMAAddress             = MEM_PCIE_DMA_RANGE_PCIE_START;

    return 0;
}

static void
pcie_set_outbound_win (pcie_hostbridge_t* host, unsigned win, u64 cpu_addr, u64 pcie_addr, u64 size) {
    u64 cpu_addr_mb, limit_addr_mb;
    u32 tmp;

    /* Set the m_base of the pcie_addr window */
    bcm_writel (
    lower_32_bits (pcie_addr) + MMIO_ENDIAN,
    host->m_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO + (win * 8));
    bcm_writel (upper_32_bits (pcie_addr), host->m_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI + (win * 8));

    cpu_addr_mb   = cpu_addr >> 20;
    limit_addr_mb = (cpu_addr + size - 1) >> 20;

    /* Write the addr m_base low register */
    WR_FLD_WITH_OFFSET (host->m_base, (win * 4), PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT, BASE, cpu_addr_mb);
    /* Write the addr limit low register */
    WR_FLD_WITH_OFFSET (host->m_base, (win * 4), PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT, LIMIT, limit_addr_mb);

    /* Write the cpu addr high register */
    tmp = (u32)(cpu_addr_mb >> PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_NUM_MASK_BITS);
    WR_FLD_WITH_OFFSET (host->m_base, (win * 8), PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI, BASE, tmp);
    /* Write the cpu limit high register */
    tmp = (u32)(limit_addr_mb >> PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_NUM_MASK_BITS);
    WR_FLD_WITH_OFFSET (host->m_base, (win * 8), PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI, LIMIT, tmp);
}

static uintptr_t
pcie_map_conf (pcie_hostbridge_t* host, unsigned busnr, unsigned devfn, int where) {
    /* Accesses to the RC go right to the RC registers if slot==0 */
    if (busnr == 0)
        return PCI_SLOT (devfn) ? 0 : host->m_base + where;

    /* For devices, write to the config space index register */
    int idx = cfg_index (busnr, devfn, 0);
    bcm_writel (idx, host->m_base + PCIE_EXT_CFG_INDEX);
    return host->m_base + PCIE_EXT_CFG_DATA + where;
}


static uintptr_t find_pci_capability (uintptr_t nPCIConfig, u8 uchCapID) {
    ASSERT (nPCIConfig != 0, "PCIe config is 0");
    ASSERT (uchCapID > 0, "PCIe cap id is 0");

#define PCI_STATUS          0x06 /* 16 bits */
#define PCI_STATUS_CAP_LIST 0x10 /* Support Capability List */
#define PCI_CAPABILITY_LIST 0x34 /* Offset of first capability list entry */
#define PCI_CAP_LIST_NEXT   1    /* Next capability in the list */

    if (!(read16 (nPCIConfig + PCI_STATUS) & PCI_STATUS_CAP_LIST)) {
        return 0;
    }

    u8 uchCapPtr = read8 (nPCIConfig + PCI_CAPABILITY_LIST);
    while (uchCapPtr != 0) {
        if (read8 (nPCIConfig + uchCapPtr + PCI_CAP_LIST_ID) == uchCapID) {
            return nPCIConfig + uchCapPtr;
        }

        uchCapPtr = read8 (nPCIConfig + uchCapPtr + PCI_CAP_LIST_NEXT);
    }

    return 0;
}

static void pcie_bridge_sw_init_set (pcie_hostbridge_t* host, unsigned val) {
    unsigned shift = RGR1_SW_INIT_1_INIT_GENERIC_SHIFT;
    u32 mask       = RGR1_SW_INIT_1_INIT_GENERIC_MASK;
    wr_fld_rb (host->m_base + PCIE_RGR1_SW_INIT_1, mask, shift, val);
}

static void pcie_perst_set (pcie_hostbridge_t* host, unsigned int val) {
    wr_fld_rb (host->m_base + PCIE_RGR1_SW_INIT_1, PCIE_RGR1_SW_INIT_1_PERST_MASK, PCIE_RGR1_SW_INIT_1_PERST_SHIFT, val);
}

static bool pcie_link_up (pcie_hostbridge_t* host) {
    u32 val = bcm_readl (host->m_base + PCIE_MISC_PCIE_STATUS);
    u32 dla = EXTRACT_FIELD (val, PCIE_MISC_PCIE_STATUS, PCIE_DL_ACTIVE);
    u32 plu = EXTRACT_FIELD (val, PCIE_MISC_PCIE_STATUS, PCIE_PHYLINKUP);

    return (dla && plu) ? true : false;
}

/* The controller is capable of serving in both RC and EP roles */
static bool pcie_rc_mode (pcie_hostbridge_t* host) {
    u32 val = bcm_readl (host->m_base + PCIE_MISC_PCIE_STATUS);
    return !!EXTRACT_FIELD (val, PCIE_MISC_PCIE_STATUS, PCIE_PORT);
}

static int pcie_enable_msi (pcie_hostbridge_t* host, pcie_msi_handler_t* pHandler, void* pParam) {
    ASSERT (pHandler != NULLPTR, "PCIe msi handler is null");
    memset (&msi_data_, 0, sizeof (pcie_msi_data_t));

    host->m_msi = &msi_data_;
    if (!host->m_msi)
        return -1;
    memset (host->m_msi, 0, sizeof *host->m_msi);

    host->m_msi->base        = host->m_base;
    host->m_msi->rev         = host->m_rev;
    host->m_msi->target_addr = host->m_msi_target_addr;
    host->m_msi->handler     = pHandler;
    host->m_msi->param       = pParam;

    // TODO: setup interrupt system
    // assert (m_pInterrupt != 0);
    // m_pInterrupt->ConnectIRQ (ARM_IRQ_PCIE_HOST_MSI, InterruptHandler, m_msi);

    ASSERT (host->m_rev >= BRCM_PCIE_HW_REV_33, "PCIe invalid rev");
    host->m_msi->intr_base = host->m_msi->base + PCIE_MSI_INTR2_BASE;

    msi_set_regs (host->m_msi);

    return 0;
}

static void msi_set_regs (pcie_msi_data_t* msi) {
    ASSERT (msi->rev >= BRCM_PCIE_HW_REV_33, "PCIe not equal to rev 33");
    /*
     * ffe0 -- least sig 5 bits are 0 indicating 32 msgs
     * 6540 -- this is our arbitrary unique data value
     */
    u32 data_val = 0xffe06540;

    /*
     * Make sure we are not masking MSIs. Note that MSIs can be masked,
     * but that occurs on the PCIe EP device
     */
    bcm_writel (0xffffffff, msi->intr_base + MASK_CLR);

    u32 msi_lo = lower_32_bits (msi->target_addr);
    u32 msi_hi = upper_32_bits (msi->target_addr);
    /*
     * The 0 bit of PCIE_MISC_MSI_BAR_CONFIG_LO is repurposed to MSI
     * enable, which we set to 1.
     */
    bcm_writel (msi_lo | 1, msi->base + PCIE_MISC_MSI_BAR_CONFIG_LO);
    bcm_writel (msi_hi, msi->base + PCIE_MISC_MSI_BAR_CONFIG_HI);
    bcm_writel (data_val, msi->base + PCIE_MISC_MSI_DATA_CONFIG);
}

/* Configuration space read/write support */
static int cfg_index (int busnr, int devfn, int reg) {
    return ((PCI_SLOT (devfn) & 0x1f) << PCIE_SLOT_SHIFT) |
           ((PCI_FUNC (devfn) & 0x07) << PCIE_FUNC_SHIFT) |
           (busnr << PCIE_BUSNUM_SHIFT) | (reg & ~3);
}

/* Limits operation to a specific generation (1, 2, or 3) */
static void set_gen (uintptr_t base, int gen) {
#define PCI_EXP_LNKCAP     12         /* Link Capabilities */
#define PCI_EXP_LNKCTL2    48         /* Link Control 2 */
#define PCI_EXP_LNKCAP_SLS 0x0000000f /* Supported Link Speeds */

    u32 lnkcap  = bcm_readl (base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCAP);
    u16 lnkctl2 = bcm_readw (base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCTL2);

    lnkcap = (lnkcap & ~PCI_EXP_LNKCAP_SLS) | gen;
    bcm_writel (lnkcap, base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCAP);

    lnkctl2 = (lnkctl2 & ~0xf) | gen;
    bcm_writew (lnkctl2, base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCTL2);
}

static const char* link_speed_to_str (int s) {
    switch (s) {
    case 1: return "2.5";
    case 2: return "5.0";
    case 3: return "8.0";
    default: break;
    }
    return "???";
}

/*
 * This is to convert the size of the inbound "BAR" region to the
 * non-linear values of PCIE_X_MISC_RC_BAR[123]_CONFIG_LO.SIZE
 */
static int encode_ibar_size (u64 size) {
    int log2_in = ilog2 (size);

    if (log2_in >= 12 && log2_in <= 15)
        /* Covers 4KB to 32KB (inclusive) */
        return (log2_in - 12) + 0x1c;
    else if (log2_in >= 16 && log2_in <= 37)
        /* Covers 64KB to 32GB, (inclusive) */
        return log2_in - 15;
    /* Something is awry so disable */
    return 0;
}

static u32 rd_fld (uintptr_t p, u32 mask, int shift) {
    return (bcm_readl (p) & mask) >> shift;
}

static void wr_fld (uintptr_t p, u32 mask, int shift, u32 val) {
    u32 reg = bcm_readl (p);

    reg = (reg & ~mask) | ((val << shift) & mask);
    bcm_writel (reg, p);
}

static void wr_fld_rb (uintptr_t p, u32 mask, int shift, u32 val) {
    wr_fld (p, mask, shift, val);
    (void)bcm_readl (p);
}

// TODO: implement interrupt handler
// static void InterruptHandler (void* pParam) {
//     pcie_msi_data_t* msi = (pcie_msi_data_t*)pParam;
//     ASSERT (msi != NULLPTR, "PCIe msi is null");

//     u32 status;
//     while ((status = bcm_readl (msi->intr_base + STATUS)) != 0) {
//         for (unsigned vector = 0; status && vector < BRCM_INT_PCI_MSI_NR; vector++) {
//             u32 mask = 1 << vector;

//             if (!(status & mask)) {
//                 continue;
//             }

//             /* clear the interrupt */
//             bcm_writel (mask, msi->intr_base + CLR);

//             ASSERT (msi->handler != NULLPTR, "PCIe msi handler is null");
//             (*msi->handler) (vector, msi->param);

//             status &= ~mask;
//         }
//     }

//     bcm_writel (1, msi->base + PCIE_MISC_EOI_CTRL);
// }

static void usleep_range (unsigned min, unsigned max) {
    wait_us (min);
}

static void msleep (unsigned ms) {
    wait_ms (ms);
}

static int ilog2 (u64 v) {
    int l = 0;
    while (((u64)1 << l) < v)
        l++;
    return l;
}