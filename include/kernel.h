#ifndef _KERNEL_H
#define _KERNEL_H

#define S_FRAME_SIZE			256 		// size of all saved registers 

#define SYNC_INVALID_EL1t		0 
#define IRQ_INVALID_EL1t		1 
#define FIQ_INVALID_EL1t		2 
#define ERROR_INVALID_EL1t		3 

#define SYNC_INVALID_EL1h		4 
#define IRQ_INVALID_EL1h		5 
#define FIQ_INVALID_EL1h		6 
#define ERROR_INVALID_EL1h		7 

#define SYNC_INVALID_EL0_64	    8 
#define IRQ_INVALID_EL0_64	    9 
#define FIQ_INVALID_EL0_64		10 
#define ERROR_INVALID_EL0_64	11 

#define SYNC_INVALID_EL0_32		12 
#define IRQ_INVALID_EL0_32		13 
#define FIQ_INVALID_EL0_32		14 
#define ERROR_INVALID_EL0_32	15 

#define PAGE_SHIFT 12
#define TABLE_SHIFT 9
#define SECTION_SHIFT (PAGE_SHIFT + TABLE_SHIFT)
#define PAGE_SIZE (1 << PAGE_SHIFT)	
#define SECTION_SIZE (1 << SECTION_SHIFT)	


#ifndef __ASSEMBLER__
#include "base.h"

/********************** GIC Interrupts ***************************/
u32 gic_get_cpu_id(void);
void gic_enable_interrupt(u32 irq_id);
void gic_assign_target(u32 irq_id, u32 gic_cpu_id);
void gic_enable(void);

void delay(u64);
void put32(u64, u32);
u32 get32(u64);
u32 get_exception_lvl(void);

/********************** ARM Interrupts ***************************/
#define IRQ_BIT (1 << 7)
#define FIQ_BIT (1 << 6)
#define ENABLE_IRQ          asm volatile ("msr daifset, #2")
#define DISABLE_IRQ         asm volatile ("msr daifclr, #2")
#define ENABLE_FIQ          asm volatile ("msr daifset, #1")
#define DISABLE_FIQ         asm volatile ("msr daifclr, #1")
#define ENABLE_IRQ_FIQ      asm volatile ("msr daifset, #3")
#define DISABLE_IRQ_FIQ     asm volatile ("msr daifclr, #3")
#define MAX_NESTED_INTERRUPTS 24
u64 get_daif_flags(void);
void set_daif_flags(u64);
s32 enter_critical(u32 target_lvl);
s32 leave_critical(void);

/********************** Synchronization ***************************/


#endif // __ASSEMBLER__

#endif // _KERNEL_H