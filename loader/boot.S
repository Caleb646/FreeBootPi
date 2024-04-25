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
    // Core 0 copies and relocates the loader
    cbz     x1, relocate_loader
    b       secondary_cpu_start

    relocate_loader: //
        ldr     x1, =0x80000
        ldr     x2, =_start
        ldr     x3, =__loader_size
    relocate_loader_inner:  //
        // Every section is 8 byte aligned. Determined in the linker
        // script
        ldr     x4, [x1], #8 
        str     x4, [x2], #8
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