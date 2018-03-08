#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "CAN.h"

#define UART_USB_TXD    	(GPIO_NUM_1)
#define UART_USB_RXD    	(GPIO_NUM_3)
#define UART_SPEED      	(921600)
#define UART_TX_BUF_SZ  	(0)
#define UART_RX_BUF_SZ  	(256)
#define UART_EVT_BUF_SZ		(128)
#define UART_FMT_BUF_SZ		(160)
#define UART_LINE_BUF_SZ	(512)

#define CAN_RXD				(GPIO_NUM_26)
#define CAN_TXD				(GPIO_NUM_27)
#define CAN_QUEUE_SZ 		(10)
#define CAN_SPEED			(CAN_SPEED_500KBPS)

typedef struct {
	int16_t data;
	unsigned long lastUpdate;
	unsigned long invalidAfter;
} CAN_int16_data_t;

QueueHandle_t uartEventQueue, uartLineQueue, canUpdateQueue;
CAN_device_t CAN_cfg;

CAN_int16_data_t rpm;
bool transmit = true;

static void uart_read_task();
static void uart_action_task();
static void can_rx_task();
static void ctrl_task();

unsigned long millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void print(const char* str) {
	uart_write_bytes(UART_NUM_1, str, strlen(str));
}

char __fmtbuf[UART_FMT_BUF_SZ];
void printfmt(const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsprintf(__fmtbuf, format, args);
	va_end(args);
	print(__fmtbuf);
}

void printerr(const char* id, const char* msg, int code) {
	printfmt("{\"type\": \"error\", \"message\": \"%s\", \"id\": \"%s\", \"code\": %d, \"timestamp\": %lu}\n", msg, id, code, millis());
}

void printmsg(const char* id, const char* msg) {
	printfmt("{\"type\": \"info\", \"message\": \"%s\", \"id\": \"%s\", \"timestamp\": %lu}\n", msg, id, millis());
}

void wait(unsigned int ms) {
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

void initVariables() {
	rpm.lastUpdate = 0;
	rpm.invalidAfter = 250; // After 250ms, the RPM data will be considered invalid
}

void initUART() {
	uart_config_t uart_config = {
		.baud_rate = UART_SPEED,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_ODD,
		.stop_bits = UART_STOP_BITS_2,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, UART_USB_TXD, UART_USB_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(UART_NUM_1, UART_RX_BUF_SZ, UART_TX_BUF_SZ, UART_EVT_BUF_SZ, &uartEventQueue, 0);
	uartLineQueue = xQueueCreate(64, sizeof(char*));
	
	xTaskCreate(uart_read_task, "uart_read_task", 1024*2, NULL, 10, NULL);
	xTaskCreate(uart_action_task, "uart_action_task", 1024*6, NULL, 8, NULL);
}

void initCAN() {
	CAN_cfg.speed = CAN_SPEED;
	CAN_cfg.rx_queue = xQueueCreate(CAN_QUEUE_SZ, sizeof(CAN_frame_t));
	CAN_cfg.rx_pin_id = CAN_RXD;
	CAN_cfg.tx_pin_id = CAN_TXD;
	CAN_init();
	xTaskCreate(can_rx_task, "can_rx_task", 1024*2, NULL, 9, NULL);
}

bool dataIsValid(CAN_int16_data_t* data) {
	if(data->lastUpdate == 0)
		return false; // Data has never been updated
	if(data->lastUpdate + data->invalidAfter < millis())
		return false; // Data is no longer valid
	return true;
}

void canSend(int id, int dlc, unsigned char data[]) {
	CAN_frame_t send;
	send.FIR.B.FF = CAN_frame_std;
	send.FIR.B.RTR = 0;
	send.id = id;
	send.dlc = dlc;
	for(int i = 0; i < dlc; i++)
		send.data.bytes[i] = data[i];
	CAN_write_frame(&send);
}

static void uart_read_task() {
	uart_event_t event;
	uint8_t data[UART_LINE_BUF_SZ];
	uint16_t dataAvailable = 0;
	while(true) {
		if(xQueueReceive(uartEventQueue, &event, portMAX_DELAY) == pdTRUE) {
			switch(event.type) {
				case UART_DATA: {
					// This is happening any time a byte is received over UART
					bool hasEndl = false;
					int space = UART_LINE_BUF_SZ - dataAvailable;
					// First we read into the data buffer
					int len = uart_read_bytes(UART_NUM_1, data + dataAvailable, event.size > space ? space : event.size, portMAX_DELAY);
					dataAvailable += len;
					// Then we look for the endline
					for(uint8_t* d = data + dataAvailable - len; d < data + dataAvailable; d++) {
						if(*d == '\r')
							*d = '\n';
						if(*d == '\n')
							hasEndl = true;
					}
					// If we got the endline, copy the string into the queue, and it will be handled by uart_action_task
					if(hasEndl) {
						*(data + dataAvailable) = 0;
						int slen = strlen((const char*) data);
						char* line = malloc(slen + 1);
						strcpy(line, (const char*) data);
						xQueueSend(uartLineQueue, &line, portMAX_DELAY);
						dataAvailable = 0;
					}
					break;
				}
				case UART_BREAK:
					printmsg("uartbreak", "UART Break");
					break;
				case UART_BUFFER_FULL:
					printmsg("uartbufmax", "UART RX Buffer Full");
					break;
				case UART_FIFO_OVF:
					printmsg("uartovf", "UART FIFO Overflow");
					break;
				case UART_FRAME_ERR:
					printmsg("uartframe", "UART RX Frame Error");
					break;
				case UART_PARITY_ERR:
					// Don't really care about parity errors?
					break;
				case UART_DATA_BREAK:
					printmsg("uartdbreak", "UART Data Break");
					break;
				case UART_PATTERN_DET:
					printmsg("uartpattern", "UART Pattern Detected");
					break;
				case UART_EVENT_MAX:
					printmsg("uartevtmax", "UART Event Buffer Full");
					break;
				default:
					printerr("uartevt", "UART Event", event.type);
			}
		}
	}
}

static void uart_action_task() {
	char* inputBuffer;
	while(true) {
		if(xQueueReceive(uartLineQueue, &inputBuffer, portMAX_DELAY) == pdTRUE) {
			if(strcmp(inputBuffer, "stop\n") == 0) {
				transmit = false;
				print("OK, stopped\n");
			}
			else if(strcmp(inputBuffer, "go\n") == 0) {
				transmit = true;
				print("OK, transmitting\n");
			}
			else {
				printfmt("? %s", inputBuffer);
			}

			free(inputBuffer); // uart_read_task mallocs, we have to free the memory
		}
	}
}

static void can_rx_task() {
	CAN_frame_t frame;
	while(true) {
		if(xQueueReceive(CAN_cfg.rx_queue, &frame, portMAX_DELAY) == pdTRUE) {
			//xQueueSend(canUpdateQueue, &frame, portMAX_DELAY);
			switch(frame.id) {
				case 0x5F0:
					if(frame.dlc == 8) { // Probably check the DLC to make sure you're not getting garbage
						rpm.data = ((frame.data.bytes[6]) << 8) | frame.data.bytes[7]; // This is where you want to grab any data from the frame
						rpm.lastUpdate = millis();
					}
			}
		}
	}
}

static void ctrl_task() {
	gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1ULL<<0));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
	unsigned char toSend[1];
	while(true) {
		// if(dataIsValid(&rpm))
		// 	toSend[0] = (rpm.data > 4000 ? 0xff : 0x0f);
		// else
		// 	toSend[0] = 0x00;

		// This control loop is sending 0x00, 0x0f, and 0xff to ID 0x130
		// 0x00 is sent if no valid data exists
		// 0x0f is sent if RPM < 4000
		// 0xff is sent if RPM >= 4000

		toSend[0] = (!gpio_get_level(GPIO_NUM_0)) & 0x01;
		if(transmit)
			canSend(0x130, 1, toSend);

		wait(10); // This control loop will be run at 100Hz
	}
}

static void ctrl_task2() {
	unsigned char toSend[8];
	for(int i = 0; i < 8; i++)
		toSend[i] = 0xaa;
	while(true) {
		canSend(0x1, 8, toSend);
		vTaskDelay(5);
		//wait(1); // This control loop will be run at 100Hz
	}
}

void app_main() {
	initVariables();
	initUART();
	initCAN();

	//printfmt("Git Version %s\n", GRVERSION);

	xTaskCreate(ctrl_task, "ctrl_task", 1024, NULL, 7, NULL);
	xTaskCreate(ctrl_task2, "ctrl_task2", 1024, NULL, 6, NULL);
}
