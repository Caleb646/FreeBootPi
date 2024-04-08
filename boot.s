#include "mm.h"

.section ".text.boot" // Specify that everything defined in boot.S should go in the .text.boot

.globl _start // When the kernel is started, execution begins at the start function:
_start:
    // The Raspberry Pi has four core processors, and after 
    // the device is powered on, each core begins to execute the same code. 
    // However, we don't want to work with four cores; we want to work only 
    // with the first one and put all of the other cores in an endless loop. 
    // This is exactly what the _start function is responsible for. It gets 
    // the processor ID from the mpidr_el1 system register. If 
    // the current process ID is 0, then execution is transferred to the master function:
    mrs    x0, mpidr_el1   // mrs = Load value from a system register to one of the general purpose registers (x0–x30)     
    and    x0, x0,#0xFF    // Check processor id
    cbz    x0, master        
    b      proc_hang       // Hang for all non-primary CPU

proc_hang: 
    b proc_hang

master:
    // Init bss section to zero
    adr    x0, bss_begin
    adr    x1, bss_end
    sub    x1, x1, x0
    // the first seven arguments are passed to the called function 
    // via registers x0–x6. The memzero function accepts only 
    // two arguments: the start address (bss_begin) and the size of 
    // the section needed to be cleaned (bss_end - bss_begin).
    bl     memzero
    
    // LOW_MEMORY is defined in mm.h and is equal to 4MB. 
    // Our kernel's stack won't grow (grows from higher to lower addresses) very large and the image 
    // itself is tiny, so 4MB is more than enough for us.
    mov    sp, #LOW_MEMORY // initialize the stack pointer
    bl    kernel_main // pass execution to the kernel_main function