#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>

#define UART_FMT_BUF_SZ (160)

unsigned long millis(void);

void wait(unsigned int ms);

void print(const char* str);

void println(const char* str);

void printfmt(const char* format, ...);

uint16_t reverse16(uint16_t value);

uint32_t reverse32(uint32_t value);

#endif