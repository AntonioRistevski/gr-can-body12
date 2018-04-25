#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "CAN.h"
#include "CAN_UDS.h"
#include "util.h"

#define UART_USB_TXD    	(GPIO_NUM_1)
#define UART_USB_RXD    	(GPIO_NUM_3)
#define UART_SPEED      	(921600)
#define UART_TX_BUF_SZ  	(0)
#define UART_RX_BUF_SZ  	(8128)
#define UART_EVT_BUF_SZ		(128)
#define UART_LINE_BUF_SZ	(512)

#define CAN_RXD				(GPIO_NUM_26)
#define CAN_TXD				(GPIO_NUM_27)
#define CAN_QUEUE_SZ 		(35)
#define CAN_SPEED			(CAN_SPEED_500KBPS)

typedef struct {
	int16_t data;
	unsigned long lastUpdate;
	unsigned long invalidAfter;
} CAN_int16_data_t;

QueueHandle_t uartEventQueue, uartLineQueue;
CAN_device_t CAN_cfg; // Name cannot change, extern reference
CAN_UDS_cfg_t CAN_UDS_cfg; // Name cannot change, extern reference

CAN_int16_data_t rpm;
bool transmit = true;

static void uart_read_task();
static void uart_action_task();
static void can_rx_task();
static void ctrl_task();

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
}

void initCAN() {
	CAN_cfg.speed = CAN_SPEED;
	CAN_cfg.rx_queue = xQueueCreate(CAN_QUEUE_SZ, sizeof(CAN_frame_t));
	CAN_cfg.rx_pin_id = CAN_RXD;
	CAN_cfg.tx_pin_id = CAN_TXD;
	CAN_init();
}

void initCAN_UDS() {
	CAN_UDS_cfg.outId = 0x711;
	CAN_UDS_cfg.inId = 0x712;
	CAN_UDS_cfg.queue = xQueueCreate(CAN_QUEUE_SZ, sizeof(CAN_frame_t));
	strcpy(CAN_UDS_cfg.name, "Dashboard Module");
	strcpy(CAN_UDS_cfg.version, GRVERSION);
	CAN_UDS_init();
}

bool dataIsValid(CAN_int16_data_t* data) {
	if(data->lastUpdate == 0)
		return false; // Data has never been updated
	if(data->lastUpdate + data->invalidAfter < millis())
		return false; // Data is no longer valid
	return true;
}

static void uart_read_task() {
	uart_event_t event;
	uint8_t data[UART_RX_BUF_SZ];
	uint16_t dataAvailable = 0;
	while(true) {
		if(xQueueReceive(uartEventQueue, &event, portMAX_DELAY) == pdTRUE) {
			switch(event.type) {
				case UART_DATA:
				{
					int endlCount = 0;
					int space = UART_RX_BUF_SZ - dataAvailable;
					int len = uart_read_bytes(UART_NUM_1, data + dataAvailable, event.size > space ? space : event.size, portMAX_DELAY);
					dataAvailable += len;
					for(uint8_t* d = data + dataAvailable - len; d < data + dataAvailable; d++) {
						if(*d == '\n')
							endlCount++;
					}
					if(endlCount != 0) {
						char* part = strtok((const char*) data, "\n");
						while(endlCount > 0) {
							int slen = strlen(part);
							char* line = pvPortMalloc(slen+1);
							strcpy(line, part);
							xQueueSend(uartLineQueue, &line, portMAX_DELAY);
							endlCount--;
							part = strtok(NULL, "\n");
						}

						data[0] = 0;
						if(part != NULL)
							strcpy((const char*) data, part);

						dataAvailable = 0;
					}
					break;
				}
				case UART_BREAK:
					print("UART Break\n");
					break;
				case UART_BUFFER_FULL:
					print("UART RX Buffer Full\n");
					break;
				case UART_FIFO_OVF:
					print("UART FIFO Overflow\n");
					break;
				case UART_FRAME_ERR:
					print("UART RX Frame Error\n");
					break;
				case UART_DATA_BREAK:
					print("UART Data Break\n");
					break;
				case UART_PATTERN_DET:
					print("UART Pattern Detected\n");
					break;
				case UART_EVENT_MAX:
					print("UART Event Buffer Full\n");
					break;
				case UART_PARITY_ERR: // Don't care
					break;
				default:
					printfmt("UART Event %d\n", event.type);
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
				printfmt("? %s\n", inputBuffer);
			}

			vPortFree(inputBuffer); // uart_read_task mallocs, we have to free the memory
		}
	}
}

static void can_rx_task() {
	CAN_frame_t frame;
	while(true) {
		if(xQueueReceive(CAN_cfg.rx_queue, &frame, portMAX_DELAY) == pdTRUE) {
			xQueueSend(CAN_UDS_cfg.queue, &frame, portMAX_DELAY); // Pass messages through to the updater
			switch(frame.id) {
				// when in neutral
				// starter status
				// engine status
				// engine temperature
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

	// GPIO 0 is starter button in (active low)
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_0;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 0");

	// GPIO 2 is starter button LED (active high)
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_SEL_2;
	io_conf.pull_down_en = 1;
	io_conf.pull_up_en = 0;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 2");

	// GPIO 32 is fuel switch in (active low)
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_32;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 32");

	// GPIO 34 is fan override switch in (active low)
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_34;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 34");

	bool starter = false;
	bool fuel = false;
	bool fan = false;
	bool fanOverride = false;

	bool starterLight = false;

	unsigned char toSend[1];
	while(true) {
		starter = (!gpio_get_level(GPIO_NUM_0));
		fuel = (!gpio_get_level(GPIO_NUM_32));
		fanOverride = (!gpio_get_level(GPIO_NUM_34));

		if(fanOverride)
			fan = true;
		else {
			// TODO auto control
			fan = false;
		}

		// if(dataIsValid(&rpm))
		// 	toSend[0] = (rpm.data > 4000 ? 0xff : 0x0f);
		// else
		// 	toSend[0] = 0x00;

		// This control loop is sending 0x00, 0x0f, and 0xff to ID 0x130
		// 0x00 is sent if no valid data exists
		// 0x0f is sent if RPM < 4000
		// 0xff is sent if RPM >= 4000

		toSend[0] = starter;
		toSend[0] |= fuel << 1;
		toSend[0] |= fan << 2;
		if(transmit)
			CAN_send(0x130, 1, toSend);

		char buf[10];
		buf[0] = starter ? 'S' : 's';
		buf[1] = fuel ? 'F' : 'f';
		buf[2] = fan ? 'C' : 'c';
		buf[3] = '\n';
		buf[4] = 0;
		print(buf);

		gpio_set_level(GPIO_NUM_2, starterLight);
		starterLight = !starterLight;

		wait(10); // This control loop will be run at 100Hz
	}
}

void app_main() {
	initVariables();
	initUART();
	initCAN();
	initCAN_UDS();

	xTaskCreate(uart_read_task, "uart_read_task", 1024 + UART_RX_BUF_SZ, NULL, 10, NULL);
	xTaskCreate(uart_action_task, "uart_action_task", 1024*6, NULL, 8, NULL);
	xTaskCreate(can_rx_task, "can_rx_task", 1024*2, NULL, 9, NULL);
	xTaskCreate(ctrl_task, "ctrl_task", 1024, NULL, 7, NULL);
}
