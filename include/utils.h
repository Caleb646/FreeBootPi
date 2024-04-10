#ifndef	_BOOT_UTILS_H
#define	_BOOT_UTILS_H

extern void delay(unsigned long);
extern void put32(unsigned long, unsigned int);
extern unsigned int get32(unsigned long);
extern int get_exception_lvl(void);

#endif  /*_BOOT_UTILS_H */