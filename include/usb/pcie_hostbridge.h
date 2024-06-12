#ifndef _PCIE_HOSTBRIDGE_H
#define _PCIE_HOSTBRIDGE_H

#include "base.h"
#include "mem.h"

typedef struct pcie_memory_window_t {
    u64 pcie_addr;
    u64 cpu_addr;
    u64 size;
} pcie_memory_window_t;

typedef void pcie_msi_handler_t (unsigned nVector, void* pParam);

typedef struct pcie_msi_data_t {
    uintptr_t base;
    u64 target_addr;
    uintptr_t intr_base; // base of interrupt status/set/clr regs
    u32 rev;
    pcie_msi_handler_t* handler;
    void* param;
} pcie_msi_data_t;

typedef struct pcie_hostbridge_t {
    uintptr_t m_base; // mmio base address
    u32 m_rev;        // controller revision

    pcie_memory_window_t m_out_wins[1]; // outbound window
    s32 m_num_out_wins;

    pcie_memory_window_t m_dma_ranges[1]; // inbound window
    s32 m_num_dma_ranges;
    u64 m_scb_size[1];
    s32 m_num_scbs;

    u64 m_msi_target_addr;
    pcie_msi_data_t* m_msi;

    u64 s_nDMAAddress;

} pcie_hostbridge_t;

pcie_hostbridge_t* pcie_hostbridge_create (void);
bool pcie_hostbridge_init (pcie_hostbridge_t* host);
bool pcie_hostbridge_enable_device (pcie_hostbridge_t* host, u32 nClassCode, unsigned nSlot, unsigned nFunc);
bool pcie_hostbridge_connect_msi (pcie_hostbridge_t* host, pcie_msi_handler_t* pHandler, void* pParam);
void pcie_hostbridge_disconnect_msi (pcie_hostbridge_t* host);

#endif /*_PCIE_HOSTBRIDGE_H */