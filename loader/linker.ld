
SECTIONS
{
    PROVIDE(__MAX_LOADER_SIZE = 2048); /* 2 KB*/

    . = 0x80000 - __MAX_LOADER_SIZE;     /* Kernel load address for AArch64 */
    PROVIDE(__loader_start = .);
    .text : { KEEP(*(.text.boot)) *(.text .text.* .gnu.linkonce.t*) }
    .rodata : { *(.rodata .rodata.* .gnu.linkonce.r*) }
    .data : { *(.data .data.* .gnu.linkonce.d*) }
    .bss (NOLOAD) : {
        . = ALIGN(16);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    }
    __loader_end = .;

   /DISCARD/ : { *(.comment) *(.gnu*) *(.note*) *(.eh_frame*) }

   __bss_size = (__bss_end - __bss_start); /* >> 3 */
   __loader_size = (__loader_end - __loader_start); /* >> 3 */
}
ASSERT(__loader_size < __MAX_LOADER_SIZE, "Loader is larger than max size of 2 KB");