#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "CAN.h"

#define ECHO_TEST_TXD  (GPIO_NUM_1)
#define ECHO_TEST_RXD  (GPIO_NUM_3)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

uint8_t data[BUF_SIZE];
unsigned int dataAvailable = 0;
char str[BUF_SIZE];
unsigned int slen = 0;
CAN_device_t CAN_cfg;

SemaphoreHandle_t mutex;

void print(const char* str) {
	uart_write_bytes(UART_NUM_1, str, strlen(str));
}

void wait(unsigned int ms) {
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void read_task() {
	while (1) {
		if(xSemaphoreTake(mutex, 10) == pdTRUE) {
			int len = uart_read_bytes(UART_NUM_1, data + dataAvailable, BUF_SIZE - dataAvailable, 20 / portTICK_RATE_MS);
			dataAvailable += len;
			xSemaphoreGive(mutex);
			wait(10);
		}
	}
}

static void app_task() {
	while(1) {
		if(dataAvailable && xSemaphoreTake(mutex, 0) == pdTRUE) {
			memcpy(str + slen, data, dataAvailable);
			slen += dataAvailable;
			dataAvailable = 0;
			xSemaphoreGive(mutex);

			bool hasEndl = false;
			for(int i = 0; i < slen; i++) {
				if(str[i] == '\r')
					hasEndl = true;
			}

			if(hasEndl) {
				str[slen] = '\n';
				str[slen+1] = '\0';
				print(str);
				slen = 0;
			}
		}

		CAN_frame_t frame;
		if(xQueueReceive(CAN_cfg.rx_queue, &frame, 0) == pdTRUE) {
			char buf[256];
			sprintf(buf, "F: %03x DLC %d %02x\n", frame.MsgID, frame.FIR.B.DLC, frame.data.u8[0]);
			print(buf);
			
			CAN_frame_t ret;
			ret.MsgID = 0x1a4;
			ret.FIR.B.DLC = 3;
			ret.data.u8[0] = 0xc0;
			ret.data.u8[1] = 0xff;
			ret.data.u8[2] = 0xee;
			CAN_write_frame(&ret);
		}

		wait(10);
	}
}

void init_uart() {
	wait(250);
	uart_config_t uart_config = {
		.baud_rate = 921600,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
	uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void init_can() {
	CAN_cfg.speed = CAN_SPEED_83K3BPS;
	CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
	CAN_cfg.rx_pin_id = GPIO_NUM_26;
	CAN_cfg.tx_pin_id = GPIO_NUM_27;
	CAN_init();
}

void app_main() {
	init_uart();
	init_can();

	mutex = xSemaphoreCreateMutex();
	if(mutex == NULL)
		print("MUTEX COULD NOT BE CREATED");
	
	xTaskCreate(read_task, "uart_read_task", 1024*2, NULL, 10, NULL);
	xTaskCreate(app_task, "uart_app_task", 1024*2, NULL, 9, NULL);
}
