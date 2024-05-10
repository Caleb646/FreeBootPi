#ifndef	_BASE_H
#define	_BASE_H

// Raspberry Pi 4b
#define PBASE 0x0FC000000 // When in "Low Peripheral Mode"
#define ARM_NUM_CORES 4

#ifndef __ASSEMBLER__

typedef signed char s8;
typedef unsigned char u8;
typedef unsigned int u32;
typedef long unsigned int u64;
typedef u64 size_t;

typedef signed int s32;
typedef long signed int s64;

typedef unsigned long uintptr_t;

typedef void (*irq_handler_t)(u32 irq_id);

/* https://sourceware.org/binutils/docs/ld/Source-Code-Reference.html */
extern u32 __kernel_plus_stacks_start;
extern u32 __kernel_plus_stacks_end;

#define KERNEL_START &__kernel_plus_stacks_start
#define KERNEL_END &__kernel_plus_stacks_end
#define KERNEL_SIZE (KERNEL_END - KERNEL_START)

#define NULLPTR ((void*)0)
#define REG_PTR32(reg_addr) *((u32 volatile *)reg_addr)
#define REG_PTR64(reg_addr) *((u64 volatile *)reg_addr)

#define FALSE 		0

#define GCC_NODISCARD 							__attribute__ ((__warn_unused_result__))
#define GCC_ALIGN_ADDR(byte_alignment) 			__attribute__((aligned(byte_alignment)))

void put32(u64 addr, u32 val);
u32 get32(u64 addr);
u64 get_arm_core_id(void);
u64 get_arm_exception_lvl(void);
void memset(void* src, u8 value, size_t nbytes);
void memcopy(void* src, size_t nbytes, void* dest);

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
			cache-sets = <1024>; // 1MiB(size)/64(line-size)=16384ways/16-way set
			cache-level = <2>;
		};
*/

#endif // __ASSEMBLER__

#endif  /*_ACTYPES_H */