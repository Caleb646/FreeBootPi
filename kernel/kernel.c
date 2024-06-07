#include "kernel.h"
#include "arm/sysregs.h"
#include "base.h"
#include "dma.h"
#include "irq.h"
#include "mem.h"
#include "peripherals/timer.h"
#include "peripherals/uart.h"
#include "screen.h"
#include "sync.h"
#include "usb.h"

const char* entry_error_messages[] = {
    "SYNC_INVALID_EL1t",   "IRQ_INVALID_EL1t",
    "FIQ_INVALID_EL1t",    "ERROR_INVALID_EL1t",

    "SYNC_INVALID_EL1h",   "IRQ_INVALID_EL1h",
    "FIQ_INVALID_EL1h",    "ERROR_INVALID_EL1h",

    "SYNC_INVALID_EL0_64", "IRQ_INVALID_EL0_64",
    "FIQ_INVALID_EL0_64",  "ERROR_INVALID_EL0_64",

    "SYNC_INVALID_EL0_32", "IRQ_INVALID_EL0_32",
    "FIQ_INVALID_EL0_32",  "ERROR_INVALID_EL0_32"
};

void show_invalid_entry_message (s32 type, u32 esr, u32 address) {
    LOG_ERROR ("[%s] ESR: [%x], address [%x]", entry_error_messages[type], esr, address);
}

void kernel_main (void) {
    init_printf (0, putc);
    LOG_DEBUG ("Exception Level: [0x%X]", get_arm_exception_lvl ());

    irq_init ();
    timer_init (VC_GIC_SYSTEM_TIMER_IRQ_3);

    // Data Cache should be invalidated on reset and startup.
    // L1 cache for secondary cores should be as well.
    cache_invalidate ();
    mem_init (NULLPTR);

    // wait_ms (100);

    hci_device_t* host = hci_device_create ();
    hci_init (host);

    dma_init (NULLPTR);
    screen_init ();
    screen_draw_string (512, 512, WHITE, "Hello World");
    screen_draw_rect (0, 0, 200, 200, WHITE);
    while (1) {
        // u8* ptr = (u8*)malloc(sizeof(size_t));
        // u8* ptr2 = (u8*)malloc(sizeof(size_t));
        // LOG_DEBUG("Address: [%X]  [%X] [%u]", ptr, ptr2, sizeof(node_t));
        // free(ptr);
        // screen_draw_rect (500, 500, 50, 50, WHITE);
        LOG_DEBUG ("Tick");
        screen_update ();
        wait_ms (1000); //
    }
}
