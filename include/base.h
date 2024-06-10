#ifndef _BASE_H
#define _BASE_H

// Raspberry Pi 4b
#define PERIPH_BASE   0x0FC000000 // When in "Low Peripheral Mode"
#define PERIPH_END    0x100000000UL
#define ARM_NUM_CORES 4

#ifndef __ASSEMBLER__
/*
File: printf.h

Copyright (C) 2004  Kustaa Nyholm

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

This library is really just two files: 'printf.h' and 'printf.c'.

They provide a simple and small (+200 loc) printf functionality to
be used in embedded systems.

I've found them so usefull in debugging that I do not bother with a
debugger at all.

They are distributed in source form, so to use them, just compile them
into your project.

Two printf variants are provided: printf and sprintf.

The formats supported by this implementation are: 'd' 'u' 'c' 's' 'x' 'X'.

Zero padding and field width are also supported.

If the library is compiled with 'PRINTF_SUPPORT_LONG' defined then the
long specifier is also
supported. Note that this will pull in some long math routines (pun intended!)
and thus make your executable noticably longer.

The memory foot print of course depends on the target cpu, compiler and
compiler options, but a rough guestimate (based on a H8S target) is about
1.4 kB for code and some twenty 'int's and 'char's, say 60 bytes of stack space.
Not too bad. Your milage may vary. By hacking the source code you can
get rid of some hunred bytes, I'm sure, but personally I feel the balance of
functionality and flexibility versus  code size is close to optimal for
many embedded systems.

To use the printf you need to supply your own character output function,
something like :

    void putc ( void* p, char c)
        {
        while (!SERIAL_PORT_EMPTY) ;
        SERIAL_PORT_TX_REGISTER = c;
        }

Before you can call printf you need to initialize it to use your
character output function with something like:

    init_printf(NULL,putc);

Notice the 'NULL' in 'init_printf' and the parameter 'void* p' in 'putc',
the NULL (or any pointer) you pass into the 'init_printf' will eventually be
passed to your 'putc' routine. This allows you to pass some storage space (or
anything really) to the character output function, if necessary.
This is not often needed but it was implemented like that because it made
implementing the sprintf function so neat (look at the source code).

The code is re-entrant, except for the 'init_printf' function, so it
is safe to call it from interupts too, although this may result in mixed output.
If you rely on re-entrancy, take care that your 'putc' function is re-entrant!

The printf and sprintf functions are actually macros that translate to
'tfp_printf' and 'tfp_sprintf'. This makes it possible
to use them along with 'stdio.h' printf's in a single source file.
You just need to undef the names before you include the 'stdio.h'.
Note that these are not function like macros, so if you have variables
or struct members with these names, things will explode in your face.
Without variadic macros this is the best we can do to wrap these
fucnction. If it is a problem just give up the macros and use the
functions directly or rename them.

For further details see source code.

regs Kusti, 23.10.2004
*/

#include <stdarg.h>
#include <stdbool.h>

void init_printf (void* putp, void (*putf) (void*, char));
void tfp_printf (char* fmt, ...);
void tfp_sprintf (char* s, char* fmt, ...);
void tfp_format (void* putp, void (*putf) (void*, char), char* fmt, va_list va);
#define printf  tfp_printf
#define sprintf tfp_sprintf

#define LOG_INFO(...)          \
    do {                       \
        printf ("[INFO] -- "); \
        printf (__VA_ARGS__);  \
        printf ("\r\n");       \
    } while (0)
#define LOG_DEBUG(...)          \
    do {                        \
        printf ("[DEBUG] -- "); \
        printf (__VA_ARGS__);   \
        printf ("\r\n");        \
    } while (0)
#define LOG_WARN(...)          \
    do {                       \
        printf ("[WARN] -- "); \
        printf (__VA_ARGS__);  \
        printf ("\r\n");       \
    } while (0)
#define LOG_ERROR(...)          \
    do {                        \
        printf ("[ERROR] -- "); \
        printf (__VA_ARGS__);   \
        printf ("\r\n");        \
    } while (0)

void assertion_failed (char const* fname, unsigned int line_num, char* fmt, ...);

#define STATIC_ASSERT(expr) _Static_assert(expr)
#define ASSERT(expr, ...)                                       \
    do {                                                        \
        if (!(expr)) {                                          \
            assertion_failed (__FILE__, __LINE__, __VA_ARGS__); \
        }                                                       \
    } while (0)

typedef signed char s8;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef long unsigned int u64;
typedef u64 size_t;
typedef signed int s32;
typedef long signed int s64;
typedef u64 uintptr_t;
typedef void (*irq_handler_t) (u32 irq_id, void* context);

STATIC_ASSERT ((sizeof (u8) == 1));
STATIC_ASSERT ((sizeof (s8) == 1));
STATIC_ASSERT ((sizeof (u16) == 2));
STATIC_ASSERT ((sizeof (u32) == 4));
STATIC_ASSERT ((sizeof (u64) == 8));
STATIC_ASSERT ((sizeof (s64) == 8));
STATIC_ASSERT ((sizeof (uintptr_t) == 8));

/* https://sourceware.org/binutils/docs/ld/Source-Code-Reference.html */
extern u32 __kernel_start;
extern u32 __kernel_text_end;
extern u32 __kernel_end;

// #define KERNEL_LOAD_ADDR    0x80000

#define KERNEL_START_ADDR         ((uintptr_t)&__kernel_start)
#define KERNEL_TEXT_END_ADDR      ((uintptr_t)&__kernel_text_end)
#define KERNEL_END_ADDR           ((uintptr_t)&__kernel_end)
// #define KERNEL_MEM_SIZE             (KERNEL_END - KERNEL_START)

#define NULLPTR                   ((void*)0)
#define REG_PTR8(reg_addr)        *((u8 volatile*)reg_addr)
#define REG_PTR16(reg_addr)       *((u16 volatile*)reg_addr)
#define REG_PTR32(reg_addr)       *((u32 volatile*)reg_addr)
#define REG_PTR64(reg_addr)       *((u64 volatile*)reg_addr)

#define U32_MAX                   0xFFFFFFFF
#define FALSE                     0

#define CACHE_LINE_NBYTES         64

#define GCC_NODISCARD             __attribute__ ((__warn_unused_result__))
#define GCC_ALIGN_ADDR(balign)    __attribute__ ((aligned (balign)))
#define GCC_CACHE_ALIGNED         GCC_ALIGN_ADDR (CACHE_LINE_NBYTES)
#define GCC_PACKED                __attribute__ ((packed))


/*
 * A mask of 0x3FFF_FFFF (~0xC000_0000) is used to convert a VPU Bus address to an ARM physical address.
 * This works because the top 2 bits are used by the VPU for cache control
 * and the bottom 30 bits are the physical address in SRAM.
 * So a bus address of 0xC000_0020 gets mapped to the physical SRAM address of 0x20
 */
#define VPU_BUS_TO_ARM_ADDR(addr) ((uintptr_t)addr & (~0xC0000000))
/*
 * A 32 bit VPU Bus address uses the most significant 2 bits for cache control.
 * The DMA bus address space is 0xC000_0000 to 0xFFFF_FFFF and it is an uncached space.
 * It represents the 1st GB of SRAM. So a physical SRAM address of 0x20 gets mapped to the bus address of 0xC000_0020
 *
 * 0xC000_0000 is an alias for the 1st GB of SRAM. This alias tells the VPU to not cache the data at this address
 */
#define ARM_TO_VPU_BUS_ADDR(addr) (VPU_BUS_TO_ARM_ADDR (addr) | 0xC0000000)

void write8 (uintptr_t addr, u16 val);
void write16 (uintptr_t addr, u16 val);
void write32 (uintptr_t addr, u32 val);
void write64 (uintptr_t addr, u64 val);
u8 read8 (uintptr_t addr);
u16 read16 (uintptr_t addr);
u32 read32 (uintptr_t addr);
u64 get_arm_core_id (void);
u64 get_arm_exception_lvl (void);
void memset (void* src, u8 value, size_t nbytes);
void memcpy (void const* src, size_t nbytes, void* dest);

size_t strlen (char const* str);
void strcpy (char const* src, char* dest);
void strcat (char const* src, char* dest);

/*
********* Cache ***********************
https://github.com/raspberrypi/linux/blob/rpi-6.6.y/arch/arm/boot/dts/broadcom/bcm2711.dtsi

        cpu3: cpu@3 {
            device_type = "cpu";
            compatible = "arm,cortex-a72";
            reg = <3>;
            enable-method = "spin-table";
            cpu-release-addr = <0x0 0x000000f0>;
            d-cache-size = <0x8000>;
            d-cache-line-size = <64>;
            d-cache-sets = <256>; // 32KiB(size)/64(line-size)=512ways/2-way set
            i-cache-size = <0xc000>;
            i-cache-line-size = <64>;
            i-cache-sets = <256>; // 48KiB(size)/64(line-size)=768ways/3-way set
            next-level-cache = <&l2>;
        };

        //Source for d/i-cache-line-size and d/i-cache-sets
        // https://developer.arm.com/documentation/100095/0003
        // /Level-2-Memory-System/About-the-L2-memory-system?lang=en
        // Source for d/i-cache-size
        // https://www.raspberrypi.com/documentation/computers
        // /processors.html#bcm2711
        l2: l2-cache0 {
            compatible = "cache";
            cache-unified;
            cache-size = <0x100000>;
            cache-line-size = <64>;
            cache-sets = <1024>; // 1MiB(size)/64(line-size)=16384ways/16-way
set cache-level = <2>;
        };
*/

#endif // __ASSEMBLER__

#endif /*_ACTYPES_H */