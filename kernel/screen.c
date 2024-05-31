#include "screen.h"
#include "dma.h"
#include "mem.h"
#include "peripherals/vpu.h"
#include "sync.h"

static u8* volatile vpu_frame_buffer_ = NULLPTR;
static u8* frame_buffer_              = NULLPTR;
static u32 fb_size, screen_width, screen_height, bytes_per_line, pix_order;

void screen_init (void) {
    u32 cmd_buffer[VPU_CMD_BUFFER_SIZE];

    cmd_buffer[0] = 35 * sizeof (u32); // - sizeof (u32); // Length of message in bytes
    cmd_buffer[1] = VPU_MBOX_REQUEST_CODE;

    cmd_buffer[2] = VPU_MBOX_TAG_SETPHYWH;
    cmd_buffer[3] = 8;    // Request size in bytes
    cmd_buffer[4] = 8;    // Response size in bytes
    cmd_buffer[5] = 1920; // Screen width
    cmd_buffer[6] = 1080; // Screen height

    cmd_buffer[7]  = VPU_MBOX_TAG_SETVIRTWH;
    cmd_buffer[8]  = 8;
    cmd_buffer[9]  = 8;
    cmd_buffer[10] = 1920;
    cmd_buffer[11] = 1080;

    cmd_buffer[12] = VPU_MBOX_TAG_SETVIRTOFF;
    cmd_buffer[13] = 8;
    cmd_buffer[14] = 8;
    cmd_buffer[15] = 0;
    cmd_buffer[16] = 0;

    cmd_buffer[17] = VPU_MBOX_TAG_SETDEPTH;
    cmd_buffer[18] = 4;
    cmd_buffer[19] = 4;
    cmd_buffer[20] = 32; // 32 bit color

    cmd_buffer[21] = VPU_MBOX_TAG_SETPXLORDR;
    cmd_buffer[22] = 4;
    cmd_buffer[23] = 4;
    cmd_buffer[24] = 1; // RGB

    cmd_buffer[25] = VPU_MBOX_TAG_GETFB;
    cmd_buffer[26] = 8;
    cmd_buffer[27] = 8;
    cmd_buffer[28] = 4096; // Aligned on 4096 boundary
    cmd_buffer[29] = 0;    // Frame buffer size in bytes

    cmd_buffer[30] = VPU_MBOX_TAG_GETPITCH;
    cmd_buffer[31] = 4;
    cmd_buffer[32] = 4;
    cmd_buffer[33] = 0; // Bytes per line

    cmd_buffer[34] = VPU_MBOX_TAG_LAST;

    // Check call is successful and have a pointer with depth 32
    s32 status = vpu_call (&cmd_buffer, VPU_MBOX_CH_PROP);
    if (status > 0 && cmd_buffer[20] == 32 && cmd_buffer[28] != 0 && cmd_buffer[29] != 0) {
        // phy_width      = cmd_buffer[5];  // Physical width
        // phy_height     = cmd_buffer[6];  // Physical height
        // TODO: seems like width and height values are flipped
        screen_width   = cmd_buffer[10]; // Virtual width
        screen_height  = cmd_buffer[11]; // Virtual height
        bytes_per_line = cmd_buffer[33]; // Number of bytes per line
        pix_order      = cmd_buffer[24]; // Pixel order
        // cast to uintptr_t first to remove compiler warning
        vpu_frame_buffer_ = (u8*)(uintptr_t)VPU_BUS_TO_ARM_ADDR (cmd_buffer[28]);
        // Buffer size: 1920 x 1080 x 4 (32 bit color)
        fb_size = cmd_buffer[29];
        LOG_INFO (
        "VPU Frame Buffer ARM Addr [0x%X] BUS Addr [0x%X] Size [%u]",
        (u64)vpu_frame_buffer_, cmd_buffer[28], fb_size);
        LOG_INFO ("Virtual Height [%u] Width [%u]", screen_height, screen_width);
        LOG_INFO ("Pixel Order [%u]", pix_order);
        LOG_INFO ("Bytes Per Line [%u]", bytes_per_line);

        frame_buffer_ = (u8*)callocate (MEM_LOW_HEAP_ID, fb_size);

        LOG_INFO (
        "Local Frame Buffer Addr [0x%X] End [0x%X] Kernel Start [0x%X]",
        (u64)frame_buffer_, (u64)(frame_buffer_ + fb_size), KERNEL_START_ADDR);
        if (frame_buffer_ == NULLPTR) {
            LOG_ERROR ("Failed to initialize local frame buffer for screen");
        }

    } else {
        LOG_ERROR ("Failed to initialize screen");
    }
}

void screen_draw_pixel (u32 x, u32 y, u32 pixel_color) {
    if (frame_buffer_ == NULLPTR) {
        // LOG_ERROR ("Failed to draw pixel because FB is invalid");
        return;
    }
    u32 offset = (y * bytes_per_line) + (x * sizeof (u32));
    *((u32*)(frame_buffer_ + offset)) = pixel_color;
}

void screen_draw_rect (u32 x, u32 y, u32 width, u32 height, u32 color) {
    for (u32 row = y; row < y + height; ++row) {
        for (u32 col = x; col < x + width; ++col) {
            screen_draw_pixel (col, row, color);
        }
    }
}

void screen_draw_char (u32 x, u32 y, u32 color, char c) {
    s8 idx = (s8)c;
    if (idx > 0 && idx < NUM_GLYPHS) {
        for (u32 i = 0; i < BYTES_PER_GLYPH; ++i) {
            s8 glyph = font[idx][i];
            for (u32 j = 0; j < 8; ++j) {
                if ((glyph >> j) & 1) {
                    screen_draw_pixel (x + j, y + i, color);
                }
            }
        }
    }
}

void screen_draw_string (u32 x, u32 y, u32 color, char* str) {
    while (*str) {
        if (*str == '\r') {
            x = 0;
        } else if (*str == '\n') {
            x = 0;
            y += GLYPH_HEIGHT;
        } else {
            screen_draw_char (x, y, color, *str);
            x += GLYPH_WIDTH;
        }
        ++str;
    }
}

void screen_update (void) {
    DATA_MEMORY_BARRIER_OUTER_STORES ();
    // memcpy (frame_buffer_, fb_size, vpu_frame_buffer_);
    if (frame_buffer_ == NULLPTR || vpu_frame_buffer_ == NULLPTR) {
        LOG_ERROR (
        "Screen can NOT update frame buffer or vpu framer buffer is NULL");
        return;
    }
    clean_invalidate_data_cache_vaddr ((uintptr_t)frame_buffer_, fb_size);
    dma_status_t status =
    dma_memcpy ((uintptr_t)frame_buffer_, (uintptr_t)vpu_frame_buffer_, fb_size, DMA_STANDARD);
    if (status != DMA_OK) {
        LOG_ERROR ("DMA transfer to VPU frame buffer failed because [%u]", status);
    }
}