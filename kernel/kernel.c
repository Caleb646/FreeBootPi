#include "kernel.h"
#include "arm/sysregs.h"
#include "base.h"
#include "dma.h"
#include "irq.h"
#include "mem.h"
#include "peripherals/timer.h"
#include "peripherals/uart.h"
#include "printf.h"
#include "screen.h"
#include "sync.h"

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
    LOG_ERROR ("%s, ESR: %x, address, %x", entry_error_messages[type], esr, address);
}

void kernel_main (void) {
    // Cache should be invalidated  on reset and startup.
    // For secondary cores as well.
    cache_invalidate ();

    init_printf (0, putc);
    LOG_DEBUG ("Hello from Kernel");
    irq_init ();
    timer_init (VC_GIC_SYSTEM_TIMER_IRQ_3);
    mem_init (NULLPTR);
    dma_init (NULLPTR);
    screen_init ();
    screen_draw_string (512, 512, "Hello World");
    while (1) {
        // u8* ptr = (u8*)malloc(sizeof(size_t));
        // u8* ptr2 = (u8*)malloc(sizeof(size_t));
        // LOG_DEBUG("Address: [%X]  [%X] [%u]", ptr, ptr2, sizeof(node_s));
        // free(ptr);
        wait_ms (1000); //
    }
}
