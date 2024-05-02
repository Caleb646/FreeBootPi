#ifndef	_BASE_H
#define	_BASE_H

// Raspberry Pi 4b
#define PBASE 0x0FC000000 // When in "Low Peripheral Mode"
#define ARM_NUM_CORES 4

#ifndef __ASSEMBLER__

typedef unsigned char s8;
typedef unsigned char u8;
typedef unsigned int u32;
typedef long unsigned int u64;

typedef signed int s32;
typedef long signed int s64;

typedef void (*irq_handler_t)(u32 irq_id);

#define REG_PTR32(reg_addr) *((u32 volatile *)reg_addr)
#define REG_PTR64(reg_addr) *((u64 volatile *)reg_addr)

void put32(u64 addr, u32 val);
u32 get32(u64 addr);
u64 get_arm_core_id(void);
u64 get_arm_exception_lvl(void);

#endif // __ASSEMBLER__

#endif  /*_ACTYPES_H */