#include "base.h"

/*
File: printf.c

Copyright (C) 2004  Kustaa Nyholm

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*/
typedef void (*putcf) (void*, char);
static putcf stdout_putf;
static void* stdout_putp;

#ifdef PRINTF_LONG_SUPPORT

static void uli2a (unsigned long int num, unsigned int base, int uc, char* bf) {
    int n          = 0;
    unsigned int d = 1;
    while (num / d >= base)
        d *= base;
    while (d != 0) {
        int dgt = num / d;
        num %= d;
        d /= base;
        if (n || dgt > 0 || d == 0) {
            *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
            ++n;
        }
    }
    *bf = 0;
}

static void li2a (long num, char* bf) {
    if (num < 0) {
        num   = -num;
        *bf++ = '-';
    }
    uli2a (num, 10, 0, bf);
}

#endif

static void ui2a (unsigned int num, unsigned int base, int uc, char* bf) {
    int n          = 0;
    unsigned int d = 1;
    while (num / d >= base)
        d *= base;
    while (d != 0) {
        int dgt = num / d;
        num %= d;
        d /= base;
        if (n || dgt > 0 || d == 0) {
            *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
            ++n;
        }
    }
    *bf = 0;
}

static void i2a (int num, char* bf) {
    if (num < 0) {
        num   = -num;
        *bf++ = '-';
    }
    ui2a (num, 10, 0, bf);
}

static int a2d (char ch) {
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    else
        return -1;
}

static char a2i (char ch, char** src, int base, int* nump) {
    char* p = *src;
    int num = 0;
    int digit;
    while ((digit = a2d (ch)) >= 0) {
        if (digit > base)
            break;
        num = num * base + digit;
        ch  = *p++;
    }
    *src  = p;
    *nump = num;
    return ch;
}

static void putchw (void* putp, putcf putf, int n, char z, char* bf) {
    char fc = z ? '0' : ' ';
    char ch;
    char* p = bf;
    while (*p++ && n > 0)
        n--;
    while (n-- > 0)
        putf (putp, fc);
    while ((ch = *bf++))
        putf (putp, ch);
}

void tfp_format (void* putp, putcf putf, char* fmt, va_list va) {
    char bf[12];

    char ch;


    while ((ch = *(fmt++))) {
        if (ch != '%')
            putf (putp, ch);
        else {
            char lz = 0;
#ifdef PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            ch    = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i (ch, &fmt, 10, &w);
            }
#ifdef PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch  = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
            case 0: goto abort;
            case 'u': {
#ifdef PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a (va_arg (va, unsigned long int), 10, 0, bf);
                else
#endif
                    ui2a (va_arg (va, unsigned int), 10, 0, bf);
                putchw (putp, putf, w, lz, bf);
                break;
            }
            case 'd': {
#ifdef PRINTF_LONG_SUPPORT
                if (lng)
                    li2a (va_arg (va, unsigned long int), bf);
                else
#endif
                    i2a (va_arg (va, int), bf);
                putchw (putp, putf, w, lz, bf);
                break;
            }
            case 'x':
            case 'X':
#ifdef PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a (va_arg (va, unsigned long int), 16, (ch == 'X'), bf);
                else
#endif
                    ui2a (va_arg (va, unsigned int), 16, (ch == 'X'), bf);
                putchw (putp, putf, w, lz, bf);
                break;
            case 'c': putf (putp, (char)(va_arg (va, int))); break;
            case 's': putchw (putp, putf, w, 0, va_arg (va, char*)); break;
            case '%': putf (putp, ch);
            default: break;
            }
        }
    }
abort:;
}


void init_printf (void* putp, void (*putf) (void*, char)) {
    stdout_putf = putf;
    stdout_putp = putp;
}

void tfp_printf (char* fmt, ...) {
    va_list va;
    va_start (va, fmt);
    tfp_format (stdout_putp, stdout_putf, fmt, va);
    va_end (va);
}

static void putcp (void* p, char c) {
    *(*((char**)p))++ = c;
}


void tfp_sprintf (char* s, char* fmt, ...) {
    va_list va;
    va_start (va, fmt);
    tfp_format (&s, putcp, fmt, va);
    putcp (&s, 0);
    va_end (va);
}

void write32 (uintptr_t addr, u32 val) {
    REG_PTR32 (addr) = val;
}

void write64 (uintptr_t addr, u64 val) {
    REG_PTR64 (addr) = val;
}

u32 read32 (uintptr_t addr) {
    return REG_PTR32 (addr);
}

u64 get_arm_core_id (void) {
    u64 core_id;
    asm volatile("mrs %0, mpidr_el1" : "=r"(core_id));
    return core_id & 0x3;
}

u64 get_arm_exception_lvl (void) {
    u64 lvl;
    asm volatile("mrs x0, CurrentEL\n"
                 "lsr x0, x0, #2\n"
                 "mov %0, x0\n"
                 : "=r"(lvl));
    return lvl;
}

void memset (void* src, u8 value, size_t nbytes) {
    u8* ptr = (u8*)src;
    while (nbytes-- > 0) {
        *ptr++ = value;
    }
}

void memcpy (void const* src, size_t nbytes, void* dest) {
    u8 const* src_ptr = (u8 const*)src;
    u8* dest_ptr      = (u8*)dest;
    while (nbytes-- > 0) {
        *dest_ptr++ = *src_ptr++;
    }
}

size_t strlen (char const* str) {
    size_t i = 0;
    while (*str++ != '\0') {
        ++i;
    }
    return i;
}

void strcpy (char const* src, char* dest) {
    while ((*dest++ = *src++) != '\0') {
    }
}

void strcat (char const* src, char* dest) {
    size_t dest_len = strlen (dest);
    size_t src_len  = strlen (src);
    memcpy ((void const*)src, src_len, (dest + dest_len));
    dest[dest_len + src_len] = '\0';
}

static char assert_buffer_[2048] = { '\0' };

void assertion_failed (char const* fname, unsigned int line_num, char* fmt, ...) {
    memset (assert_buffer_, '\0', sizeof (assert_buffer_));
    char str[] = "[ASSERT FAILED] -- FNAME ";
    strcat (str, assert_buffer_);
    strcat (fname, assert_buffer_);

    char line_buf[32] = { '\0' };
    memset (line_buf, '\0', sizeof (assert_buffer_));
    sprintf (line_buf, " LINE [%u]", line_num);
    strcat (line_buf, assert_buffer_);

    char msg_buf[1024] = { '\0' };
    memset (msg_buf, '\0', sizeof (assert_buffer_));
    va_list va;
    va_start (va, fmt);
    tfp_format (&msg_buf, putcp, fmt, va);
    putcp (&msg_buf, 0);
    va_end (va);
    strcat (msg_buf, assert_buffer_);

    LOG_ERROR (assert_buffer_);

    // Report the error and then hang here
    while (1) {
    };
}