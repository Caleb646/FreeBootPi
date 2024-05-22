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
    /* 
    * Put the end of the kernel stacks here. Below 0x1000 is where the armstub sets cores 1-3 to spin
    */
    . = 0x1000;

    .stack_core3 :	{
        . = ALIGN(16);				/* Stack must always be aligned to 16 byte boundary AAPCS call standard */
        __stack_start_core3__ = .;
        . = . + 512;				/* EL0 stack size */
        __EL0_stack_core3 = .;
        . = . + 16384;				/* EL1 stack size */
        __EL1_stack_core3 = .;
        . = . + 512;				/* EL2 stack size (start-up) */
        __EL2_stack_core3 = .;
        __stack_end_core3__ = .;  
    }

    .stack_core2 :	{
        . = ALIGN(16);				 /* Stack must always be aligned to 16 byte boundary AAPCS call standard */
        __stack_start_core2__ = .;
        . = . + 512;				/* EL0 stack size */
        __EL0_stack_core2 = .;
        . = . + 16384;				/* EL1 stack size */
        __EL1_stack_core2 = .;
        . = . + 512;				/* EL2 stack size (start-up) */
        __EL2_stack_core2 = .;
        __stack_end_core2__ = .;  
    }

    .stack_core1 : {
        . = ALIGN(16);				/* Stack must always be aligned to 16 byte boundary AAPCS64 call standard */
        __stack_start_core1__ = .;
        . = . + 512;				/* EL0 stack size */
        __EL0_stack_core1 = .;
        . = . + 16384;				/* EL1 stack size */
        __EL1_stack_core1 = .;
        . = . + 512;				/* EL2 stack size (start-up) */
        __EL2_stack_core1 = .;
        __stack_end_core1__ = .;    
    }

    .stack_core0 : {
        . = ALIGN(16);				/* Stack must always be aligned to 16 byte boundary AAPCS64 call standard */
        __stack_start_core0__ = .;
        . = . + 512;				/* EL0 stack size */
        __EL0_stack_core0 = .;
        . = . + 16384;				/* EL1 stack size */
        __EL1_stack_core0 = .;
        . = . + 512;				/* EL2 stack size (start-up) */
        __EL2_stack_core0 = .;
        __stack_end_core0__ = .;    
    }
    __KERNEL_STACK_END = .;

    /* 
    * Kernel load address for AArch64 
    */
    . = 0x80000;     
    __kernel_start = .;
    .text : { KEEP(*(.text.boot)) *(.text .text.* .gnu.linkonce.t*) }
    __kernel_text_end = .;
    .rodata : { *(.rodata .rodata.* .gnu.linkonce.r*) }
    PROVIDE(_data = .);
    .data : { *(.data .data.* .gnu.linkonce.d*) }
    .bss (NOLOAD) : {
        . = ALIGN(16);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    }
    __kernel_end = .;

   /DISCARD/ : { *(.comment) *(.gnu*) *(.note*) *(.eh_frame*) }
    __bss_size = (__bss_end - __bss_start); 
}

ASSERT(__KERNEL_STACK_END < 0x80000, "Kernel stack space overwritting kernel");
ASSERT( (__kernel_end - __kernel_start) < 0x500000, "Kernel size minus stacks should be less than 5MB");