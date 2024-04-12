/*
* The primary purpose of the linker script is to describe how the sections in 
* the input object files (_c.o and _s.o) should be mapped into the output file (.elf).
*
* https://sourceware.org/binutils/docs/ld/Scripts.html#Scripts
*
*
*
* After startup, the Raspberry Pi loads kernel8.img into memory and starts execution 
* from the beginning of the file. That's why the .text.boot section must be first;
*
*
*
* The .bss section contains data that should be initialized to 0. By putting 
* such data in a separate section, the compiler can save some space in the 
* ELF binary––only the section size is stored in the ELF header, but the 
* section itself is omitted. After loading the image into memory, we must 
* initialize the .bss section to 0; that's why we need to record the start 
* and end of the section (hence the bss_begin and bss_end symbols) and 
* align the section so that it starts at an address that is a multiple of 8. 
* If the section is not aligned, it would be more difficult to use the str 
* instruction to store 0 at the beginning of the bss section because the 
* str instruction can be used only with 8-byte-aligned addresses.
*/


SECTIONS
{
    .text.boot : { *(.text.boot) }
    .text :  { *(.text) }
    .rodata : { *(.rodata) }
    .data : { *(.data) }
    . = ALIGN(0x8);
    bss_begin = .;
    .bss : { *(.bss*) } 
    bss_end = .;
}