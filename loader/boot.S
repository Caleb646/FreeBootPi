.section ".text.boot"  // Make sure the linker puts this at the start of the kernel image

// https://github.com/raspberrypi/tools/blob/master/armstubs/armstub8.S
// The Raspberry Pi armsub8.S will run before the _start function.
// This stub has cores 1 to 3 suspended waiting on a 64 bit value to have
// an address written to it.
// The stub also enables the GIC and puts us in Exception Level 2
.global _start  
_start: //
    // Check processor ID is zero, else hang
    mrs     x1, mpidr_el1
    and     x1, x1, #3

    // TODO: Temporary
    // cbz     x1, primary_cpu_start
    // b       secondary_cpu_start

    // Core 0 copies and relocates the loader
    cbz     x1, relocate_loader
    b       secondary_cpu_start

    relocate_loader: //
        ldr     x1, =0x80000
        ldr     x2, =_start
        ldr     x3, =__loader_size
    relocate_loader_inner:  //
        ldr     x4, [x1], #8 // ldrb = Load unsigned Byte
        str     x4, [x2], #8 // strb = Store unsigned Byte
        subs    x3, x3, #8
        b.gt    relocate_loader_inner
    b primary_cpu_start

primary_cpu_start: //
    // Set stack to start below the relocated loader
    ldr     x1, =_start
    mov     sp, x1
    b bss_clear

    bss_clear: //
         ldr     x1, =__bss_start
         ldr     x2, =__bss_size
    bss_clear_inner:  //
         str     xzr, [x1], #8 // 
         subs    x2, x2, #8
         b.gt    bss_clear_inner

    // https://github.com/raspberrypi/tools/blob/master/armstubs/armstub8.S
    // The Raspberry Pi armsub8.S will run before the _start function.
    // This stub has cores 1 to 3 suspended waiting on a 64 bit value to have
    // an address written to it.
    // The stub also enables the GIC and puts us in Exception Level 2

    // Can't leave the secondary cpus spinning in the current
    // address space because it will be overwritten. So they need
    // to be moved to the relocated secondary_cpu_start
    // sev wakes up all cores which were suspended with a wfe
    // instruction.
    // sev 

    // Load absolute address of loader_main (C entry function)
    // Absolute address is (0x8000 - MAX_LOADER_SIZE)
    ldr     x1, =loader_main
    blr     x1
    // In case it does return, halt the master core too
    ldr     x1, =hang
    br      x1

secondary_cpu_start: //
    wfe
    ldr     x1, =secondary_cpu_start
    blr     x1
    ldr     x1, =hang
    br      x1

hang: //
    b hang


// #include "mm.h"
// #include "arm/sysregs.h"
//
// .section ".text.boot" // Specify that everything defined in boot.S should go in the .text.boot
//
// .globl _start // When the kernel is started, execution begins at the start function:
// _start:
//     // The Raspberry Pi has four core processors, and after 
//     // the device is powered on, each core begins to execute the same code. 
//     // However, we don't want to work with four cores; we want to work only 
//     // with the first one and put all of the other cores in an endless loop. 
//     // This is exactly what the _start function is responsible for. It gets 
//     // the processor ID from the mpidr_el1 system register. If 
//     // the current process ID is 0, then execution is transferred to the master function:
//     mrs    x0, mpidr_el1   // mrs = Load value from a system register to one of the general purpose registers (x0–x30)     
//     and    x0, x0, #0xFF    // Check processor id
//     cbz    x0, master        
//     b      proc_hang       // Hang for all non-primary CPU
//
// proc_hang: 
//     b proc_hang
//
// master:
//     // sctlr_el1 is responsible for configuring different 
//     // parameters of the processor, when it operates at EL1. 
//     // For example, it controls whether the cache is enabled 
//     // and whether the MMU (Memory Management Unit) is turned on. 
//     // sctlr_el1 is accessible from all exception levels higher or equal than EL1
//     ldr	x0, =SCTLR_VALUE_MMU_DISABLED
// 	msr	sctlr_el1, x0		
//
//     // Hypervisor Configuration Register (EL2)
// 	ldr	x0, =HCR_VALUE
// 	msr	hcr_el2, x0
//
//     // Secure Configuration Register (EL3)
// 	ldr	x0, =SCR_VALUE
// 	msr	scr_el3, x0
//
//     // SPSR_EL3, Saved Program Status Register (EL3),
// 	ldr	x0, =SPSR_VALUE
// 	msr	spsr_el3, x0
//
//     // ELR_EL3, Exception Link Register (EL3)
//     // elr_el3 holds the address, to which we are going 
//     // to return after eret instruction will be executed. 
//     // Here we set this address to the location of el1_entry label.
// 	adr	x0, el1_entry		
// 	msr	elr_el3, x0
//
// 	eret
//
// // Exception Level 1 entrypoint
// el1_entry:
//     // Init bss section to zero
//     adr    x0, __bss_start
//     adr    x1, __bss_size
//     // the first seven arguments are passed to the called function 
//     // via registers x0–x6. The memzero function accepts only 
//     // two arguments: the start address (bss_begin) and the size of 
//     // the section needed to be cleaned (bss_end - bss_begin).
//     bl     memzero
//   
//     // LOW_MEMORY is defined in mm.h and is equal to 4MB. 
//     // Our kernel's stack won't grow (grows from higher to lower addresses) very large and the image 
//     // itself is tiny, so 4MB is more than enough for us.
//     mov    sp, LOW_MEMORY // initialize the stack pointer
//     bl    kernel_main // pass execution to the kernel_main function