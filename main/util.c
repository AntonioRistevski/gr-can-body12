#include "util.h"

#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "rom/ets_sys.h"

unsigned long millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void wait(unsigned int ms) {
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

void waitus(unsigned int us) {
	ets_delay_us(us);
}

void print(const char* str) {
	uart_write_bytes(UART_NUM_1, str, strlen(str));
}

void println(const char* str) {
	print(str);
	print("\n");
}

static char fmtbuf[UART_FMT_BUF_SZ];
void printfmt(const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsprintf(fmtbuf, format, args);
	va_end(args);
	print(fmtbuf);
}

inline uint16_t reverse16(uint16_t value) {
    return (((value & 0x00FF) << 8) |
            ((value & 0xFF00) >> 8));
}

inline uint32_t reverse32(uint32_t value) {
    return (((value & 0x000000FF) << 24) |
            ((value & 0x0000FF00) <<  8) |
            ((value & 0x00FF0000) >>  8) |
            ((value & 0xFF000000) >> 24));
}