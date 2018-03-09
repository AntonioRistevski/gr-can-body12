#ifndef __UTIL_H__
#define __UTIL_H__

#define UART_FMT_BUF_SZ (160)

unsigned long millis(void);

void wait(unsigned int ms);

void print(const char* str);

void printfmt(const char* format, ...);

#endif