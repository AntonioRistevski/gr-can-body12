#include "util.h"

#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

unsigned long millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void wait(unsigned int ms) {
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

void print(const char* str) {
	uart_write_bytes(UART_NUM_1, str, strlen(str));
}

static char fmtbuf[UART_FMT_BUF_SZ];
void printfmt(const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsprintf(fmtbuf, format, args);
	va_end(args);
	print(fmtbuf);
}