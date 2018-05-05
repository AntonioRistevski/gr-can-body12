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
	bool data;
	unsigned long lastUpdate;
	unsigned long invalidAfter;
} CAN_bool_data_t;

QueueHandle_t uartEventQueue, uartLineQueue;
CAN_device_t CAN_cfg; // Name cannot change, extern reference
CAN_UDS_cfg_t CAN_UDS_cfg; // Name cannot change, extern reference

bool transmit = true;

CAN_bool_data_t starter;
CAN_bool_data_t fuel;
CAN_bool_data_t fan;
CAN_bool_data_t brakeLight;
bool neutral = false;

static void uart_read_task();
static void uart_action_task();
static void can_rx_task();
static void ctrl_task();

void initVariables() {
	starter.lastUpdate = false;
	starter.invalidAfter = 250;
	fuel.lastUpdate = false;
	fuel.invalidAfter = 250;
	fan.lastUpdate = false;
	fan.invalidAfter = 250;
	brakeLight.lastUpdate = false;
	brakeLight.invalidAfter = 250;
}

void initGPIO() {
	gpio_config_t io_conf;

	// GPIO 32 is starter relay
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_SEL_32;
	io_conf.pull_down_en = true;
	io_conf.pull_up_en = false;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 32");

	// GPIO 33 is fuel pump relay
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_SEL_33;
	io_conf.pull_down_en = true;
	io_conf.pull_up_en = false;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 33");

	// GPIO 25 is fan relay
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_SEL_25;
	io_conf.pull_down_en = true;
	io_conf.pull_up_en = false;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 25");

	// GPIO 14 is brake light
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_SEL_14;
	io_conf.pull_down_en = true;
	io_conf.pull_up_en = false;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 14");
	
	// GPIO 12 is neutral sense
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_12;
	io_conf.pull_down_en = false;
	io_conf.pull_up_en = true;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 12");
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
	CAN_UDS_cfg.outId = 0x721;
	CAN_UDS_cfg.inId = 0x722;
	CAN_UDS_cfg.queue = xQueueCreate(CAN_QUEUE_SZ, sizeof(CAN_frame_t));
	strcpy(CAN_UDS_cfg.name, "Body Control Module");
	strcpy(CAN_UDS_cfg.version, GRVERSION);
	CAN_UDS_init();
}

bool dataIsValid(CAN_bool_data_t* data) {
	if(data->lastUpdate == 0)
		return false; // Data has never been updated
	if(data->lastUpdate + data->invalidAfter < millis())
		return false; // Data is no longer valid
	return true;
}

bool valueOr(CAN_bool_data_t* data, bool ifInvalid) {
	if(dataIsValid(data))
		return data->data;
	return ifInvalid;
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
						char* part = strtok((char*) data, "\n");
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
							strcpy((char*) data, part);

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
			unsigned long curTime = millis();
			switch(frame.id) {
				case 0x130:
					if(frame.dlc == 1) {
						starter.data = (frame.data.bytes[0] & 1);
						starter.lastUpdate = curTime;
						fuel.data = (frame.data.bytes[0] >> 1) & 1;
						fuel.lastUpdate = curTime;
						fan.data = (frame.data.bytes[0] >> 2) & 1;
						fan.lastUpdate = curTime;
					}
					break;
				case 0x120:
					brakeLight.data = (frame.data.bytes[0] & 1);
					brakeLight.lastUpdate = curTime;
					break;
				case 0x5F0:
					if(frame.dlc == 8) { // Probably check the DLC to make sure you're not getting garbage
						// rpm.data = ((frame.data.bytes[6]) << 8) | frame.data.bytes[7]; // This is where you want to grab any data from the frame
						// rpm.lastUpdate = millis();
					}
			}
		}
	}
}

static void can_tx_task() {
	unsigned char toSend[1];
	while(true) {
		toSend[0] = neutral;
		if(transmit)
			CAN_send(0x10, 1, toSend);

		wait(10); // This control loop will be run at 100Hz
	}
}

unsigned long fanOnAt = 0;
static void ctrl_task() {
	while(true) {
		neutral = !gpio_get_level(GPIO_NUM_12);
		
		gpio_set_level(GPIO_NUM_32, valueOr(&starter, false));
		gpio_set_level(GPIO_NUM_33, valueOr(&fuel, true));
		gpio_set_level(GPIO_NUM_25, valueOr(&fan, false));
		gpio_set_level(GPIO_NUM_14, valueOr(&brakeLight, false));

		wait(5); // This control loop will be run at 200Hz
	}
}

void app_main() {
	initVariables();
	initUART();
	initGPIO();
	initCAN();
	initCAN_UDS();

	xTaskCreate(uart_read_task, "uart_read_task", 1024 + UART_RX_BUF_SZ, NULL, 10, NULL);
	xTaskCreate(uart_action_task, "uart_action_task", 1024*6, NULL, 8, NULL);
	xTaskCreate(can_rx_task, "can_rx_task", 1024*2, NULL, 9, NULL);
	xTaskCreate(can_tx_task, "can_tx_task", 1024, NULL, 9, NULL);
	xTaskCreate(ctrl_task, "ctrl_task", 1024, NULL, 10, NULL);
}
