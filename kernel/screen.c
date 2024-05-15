#include "screen.h"
#include "peripherals/vidcore.h"
#include "printf.h"

static u32* frame_buffer;
static u32 phy_width, phy_height, virt_width, virt_height, bytes_per_line,
pix_order /* RGBA or BGRA ....*/;

void screen_init (void) {
    vid_core_buffer[0] = sizeof (vid_core_buffer) - sizeof (u32); // Length of message in bytes
    vid_core_buffer[1] = VID_CORE_MBOX_REQUEST_CODE;

    vid_core_buffer[2] = VID_CORE_MBOX_TAG_SETPHYWH;
    vid_core_buffer[3] = 8;
    vid_core_buffer[4] = 0;
    vid_core_buffer[5] = 1920; // Screen width
    vid_core_buffer[6] = 1080; // Screen height

    vid_core_buffer[7]  = VID_CORE_MBOX_TAG_SETVIRTWH;
    vid_core_buffer[8]  = 8;
    vid_core_buffer[9]  = 8;
    vid_core_buffer[10] = 1920;
    vid_core_buffer[11] = 1080;

    vid_core_buffer[12] = VID_CORE_MBOX_TAG_SETVIRTOFF;
    vid_core_buffer[13] = 8;
    vid_core_buffer[14] = 8;
    vid_core_buffer[15] = 0; // Value(x)
    vid_core_buffer[16] = 0; // Value(y)

    vid_core_buffer[17] = VID_CORE_MBOX_TAG_SETDEPTH;
    vid_core_buffer[18] = 4;
    vid_core_buffer[19] = 4;
    vid_core_buffer[20] = 32; // Bits per pixel

    vid_core_buffer[21] = VID_CORE_MBOX_TAG_SETPXLORDR;
    vid_core_buffer[22] = 4;
    vid_core_buffer[23] = 4;
    vid_core_buffer[24] = 1; // RGB

    vid_core_buffer[25] = VID_CORE_MBOX_TAG_GETFB;
    vid_core_buffer[26] = 8;
    vid_core_buffer[27] = 8;
    vid_core_buffer[28] = 4096; // Aligned on 4096 boundary
    vid_core_buffer[29] = 0;    // Frame buffer size in bytes

    vid_core_buffer[30] = VID_CORE_MBOX_TAG_GETPITCH;
    vid_core_buffer[31] = 4;
    vid_core_buffer[32] = 4;
    vid_core_buffer[33] = 0; // Bytes per line

    vid_core_buffer[34] = VID_CORE_MBOX_TAG_LAST;

    // Check call is successful and we have a pointer with depth 32
    s32 status = vid_core_call (vid_core_buffer, VID_CORE_MBOX_CHANNEL_PROP);
    if (status > 0 && vid_core_buffer[20] == 32 && vid_core_buffer[28] != 0) {
        // vid_core_buffer[28] &= 0x3FFFFFFF;    // Convert GPU address to ARM address
        vid_core_buffer[28] =
        VPU_BUS_TO_ARM_ADDR (vid_core_buffer[28]); // Convert GPU address to ARM address
        phy_width      = vid_core_buffer[5];  // Physical width
        phy_height     = vid_core_buffer[6];  // Physical height
        virt_width     = vid_core_buffer[10]; // Virtual width
        virt_height    = vid_core_buffer[11]; // Virtual height
        bytes_per_line = vid_core_buffer[33]; // Number of bytes per line
        pix_order      = vid_core_buffer[24]; // Pixel order
        frame_buffer   = (u32*)vid_core_buffer[28];
        // Buffer size: 1920 x 1080 x 32
    } else {
        LOG_ERROR ("Failed to intialize screen");
    }
}

void screen_draw_pixel (u32 x, u32 y, u32 pixel_color) {
    u32 offset = (y * bytes_per_line) + (x * sizeof (u32));
    *((u32*)(frame_buffer + offset)) = pixel_color;
}

void screen_draw_char (u32 x, u32 y, char c) {
    s8 idx = (s8)c;
    if (idx > 0 && idx < NUM_GLYPHS) {
        for (u32 i = 0; i < BYTES_PER_GLYPH; ++i) {
            s8 glyph = font[idx][i];
            for (u32 j = 0; j < 8; ++j) {
                if ((glyph >> j) & 1) {
                    screen_draw_pixel (x + j, y + i, WHITE);
                }
            }
        }
    }
}

void screen_draw_string (u32 x, u32 y, char* str) {
    while (*str) {
        if (*str == '\r') {
            x = 0;
        } else if (*str == '\n') {
            x = 0;
            y += GLYPH_HEIGHT;
        } else {
            screen_draw_char (x, y, *str);
            x += GLYPH_WIDTH;
        }
        ++str;
    }
}