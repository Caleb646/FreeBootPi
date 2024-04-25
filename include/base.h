#ifndef	_BASE_H
#define	_BASE_H

typedef char u8;
typedef unsigned int u32;
typedef long unsigned int u64;

typedef signed int s32;
typedef long signed int s64;

#define LOG_DEBUG(...) do { printf(__VA_ARGS__); printf("\r\n"); } while(0)
#define LOG_ERROR(...) do { printf(__VA_ARGS__); printf("\r\n"); } while(0)

// Raspberry Pi 4b
#define PBASE 0x0FC000000 // When in "Low Peripheral Mode"
// #define PBASE 0x47C000000

#endif  /*_ACTYPES_H */