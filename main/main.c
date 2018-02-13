#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "CAN.h"

#define UART_USB_TXD    (GPIO_NUM_1)
#define UART_USB_RXD    (GPIO_NUM_3)
#define UART_SPEED      (921600)
#define UART_TX_BUF_SZ  (0)
#define UART_RX_BUF_SZ  (256)
#define UART_EVT_BUF_SZ (128)
QueueHandle_t uartEventQueue, uartLineQueue;
int uartParityEvents = 0;

#define NVS_STOCK_NAME "CANOpener"
#define NVS_NAMESAPCE "canopener"
#define NVS_KEY_BOARDNAME "boardName"
#define clearBoardName() nvs_erase_key(storage, NVS_KEY_BOARDNAME)

#define FILTER_SIZE (32)

char boardName[64];
CAN_device_t CAN_cfg;
nvs_handle storage;

bool setupComplete = false;
bool quickSuppress = false;
bool quickSuppressToggle();
bool quickSuppressCheck(long unsigned int rxId);
char setSpeed = 'x'; // Used for information reporting only
bool filteringEnabled = false;
uint16_t filter[FILTER_SIZE];
int currentFilterSize = 0;
bool filterSuppressing = false;

static void uart_read_task();
static void can_tx_task();
static void can_rx_task();

unsigned long millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void print(const char* str) {
	uart_write_bytes(UART_NUM_1, str, strlen(str));
}

char __fmtbuf[160];
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

void readBoardName() {
	size_t sz = sizeof(boardName);
	esp_err_t e = nvs_get_str(storage, NVS_KEY_BOARDNAME, boardName, &sz);
	if(e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND)
		printerr("nvserror2", "Error reading from NVS", e);
	if(e != ESP_OK)
		strcpy(boardName, NVS_STOCK_NAME);
}

esp_err_t setBoardName(const char* name) {
	size_t sz = strlen(name);
	if(sz == 0) {
		esp_err_t e2 = clearBoardName();
		readBoardName();
		return e2;
	}
	if(sz > 63)
		return ESP_ERR_NVS_INVALID_LENGTH;
	esp_err_t e = nvs_set_str(storage, NVS_KEY_BOARDNAME, name);
	if(e != ESP_OK) return e;
	e = nvs_commit(storage);
	readBoardName();
	return e;
}

void boardInformation() {
	printfmt("{\"type\": \"boardInformation\", \"name\": \"%s\", \"version\": \"V2.0.0\", ", boardName);
	printfmt("\"author\": \"Paul Hollinsky\", \"speed\": \"%c\", \"filterSuppressing\": %s, \"maxFilters\": %d, ", setSpeed, (filterSuppressing ? "true" : "false"), FILTER_SIZE);
	printfmt("\"quickSuppress\": %s, \"parityErrors\": %d, \"freeMemory\": %d, \"filter\": [", (quickSuppress ? "true" : "false"), uartParityEvents, xPortGetFreeHeapSize());
	for(int i = 0; i < currentFilterSize; i++) {
		printfmt("\"0x%.2X\"", filter[i]);
		if(i != currentFilterSize-1)
			print(", ");
	}
	printfmt("], \"hardware\": \"esp32\", \"timestamp\": %lu}\n", millis());
}

void help() {
	print("{\"type\": \"info\", \"message\": \"Commands:\\n\\t/i [speed] - Initialize at the specified speed, which can be (1=83.3k, 2=125k, 3=500k)\\n\\t/? - Show Help\\n\\t/. - Board Information\\n\\t/n [name] - Set Board Name\\n\\t/o [filters...] - Only show the specified IDs, hex represented and split by spaces\\n\\t/s [filters...] - Suppress the following IDs, hex represented and split by spaces\\n\\t/a - Disable filtering\\n\\t/q - Quick suppress toggle\\n\\t/r - Reset ");
	printfmt("%s\", \"id\": \"help\", \"timestamp\": %lu}\n", boardName, millis());
}

void init(char speed) {
	char buf[84];

	if(setupComplete) {
		sprintf(buf, "%s Already Initialized", boardName);
		printerr("init2", buf, 2);
		return;
	}
	
	switch(speed) {
		case '1':
			CAN_cfg.speed = CAN_SPEED_83K3BPS;
			break;
		case '2':
			CAN_cfg.speed = CAN_SPEED_125KBPS;
			break;
		case '3':
			CAN_cfg.speed = CAN_SPEED_500KBPS;
			break;
		default:
			printerr("badspeed", "Invalid speed value", speed);
			return;
	}

	CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
	CAN_cfg.rx_pin_id = GPIO_NUM_26;
	CAN_cfg.tx_pin_id = GPIO_NUM_27;
	CAN_init();

	setupComplete = true;
	setSpeed = speed;

	sprintf(buf, "%s Initialized", boardName);
	printmsg("init", buf);

	xTaskCreate(can_rx_task, "can_rx_task", 1024*2, NULL, 9, NULL);
}

static void uart_read_task() {
	uart_event_t event;
	uint8_t data[512];
	uint16_t dataAvailable = 0;
	while(true) {
		if(xQueueReceive(uartEventQueue, &event, portMAX_DELAY) == pdTRUE) {
			switch(event.type) {
				case UART_DATA:
				{
					bool hasEndl = false;
					int space = 512 - dataAvailable;
					int len = uart_read_bytes(UART_NUM_1, data + dataAvailable, event.size > space ? space : event.size, portMAX_DELAY);
					dataAvailable += len;
					for(uint8_t* d = data + dataAvailable - len; d < data + dataAvailable; d++) {
						if(*d == '\r')
							*d = '\n';
						if(*d == '\n')
							hasEndl = true;
					}
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
					uartParityEvents++;
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

bool quickSuppressCheck(long unsigned int rxId) {
	if(!quickSuppress)
		return false;
	
	switch(rxId) {
		case 0:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 9:
		case 10:
		case 12:
		case 18:
		case 20:
		case 22:
		case 23:
		case 24:
		case 25:
		case 30:
		case 31:
		case 44:
		case 48:
		case 52:
		case 56:
		case 88:
		case 112:
		case 140:
		case 144:
		case 154:
		case 156:
		case 158:
		case 178:
		case 241:
		case 266:
		case 384:
		case 404:
		case 408:
		case 416:
		case 434:
		case 444:
		case 459:
		case 513:
		case 528:
		case 546:
		case 624:
		case 688:
		case 724:
		case 728:
		case 800:
		case 802:
		case 818:
		case 992:
		case 1024:
		case 1025:
		case 1026:
		case 1027:
		case 1028:
		case 1029:
		case 1031:
		case 1032:
		case 1033:
		case 1034:
		case 1035:
		case 1041:
		case 1042:
		case 1043:
		case 1044:
		case 1045:
		case 1046:
		case 1048:
		case 1051:
		case 1052:
		case 0x1a:
		case 0x420:
		case 0x11:
		case 0xfc:
		case 0x1b6:
		case 0x41d:
		case 0x288:
		case 0x1ac:
		case 0xee:
		case 0x40f:
		case 0x40d:
		case 0x40c:
		case 0x18c:
			return true;
		default:
			return false;
	}
}

bool quickSuppressToggle() {
	quickSuppress = !quickSuppress;
	char buf[64];
	sprintf(buf, "Quick suppress is now %s", (quickSuppress) ? "on" : "off");
	printmsg("qsset", buf);
	return quickSuppress;
}

static void can_tx_task() {
	esp_err_t e;
	char* inputBuffer;
	char* part;
	CAN_frame_t send;
	send.FIR.B.FF = CAN_frame_std;
	send.FIR.B.RTR = 0;
	while(true) {
		if(xQueueReceive(uartLineQueue, &inputBuffer, portMAX_DELAY) == pdTRUE) {
			part = strtok(inputBuffer, " ");
			if(part[0] == '/') { // control sequence
				switch(part[1]) {
					case 'i':
						init(part[3]);
						break;
					case '?':
						help();
						break;
					case '.':
						boardInformation();
						break;
					case 'n':
						e = setBoardName(inputBuffer+3);
						if(e != ESP_OK)
							printerr("badnamenvs", "The board name could not be set", e);
						else
							printmsg("namechange", "The name has been changed");
						break;
					case 'o':
					case 's':
						filterSuppressing = (part[1] == 's');
						filteringEnabled = true;
						currentFilterSize = 0;
						part = strtok(NULL, " ");
						while(part != NULL) {
							if(currentFilterSize >= FILTER_SIZE) {
								printmsg("filterwarn", "Too many filters!");
								break;
							}
							filter[currentFilterSize++] = (uint16_t) strtol(part, NULL, 16);
							part = strtok(NULL, " ");
						}
						printmsg("filteron", "Filtering enabled");
						break;
					case 'a':
						filteringEnabled = false;
						printmsg("filteroff", "Filtering disabled");
						if(quickSuppress)
							printmsg("qsnote", "Note: Quick suppress is still on");
						break;
					case 'q':
						quickSuppressToggle();
						break;
					case '~':
					{
						char buf[256];
						vTaskGetRunTimeStats(buf);
						print(buf);
					}
					break;
					case 'r':
						printmsg("reset", "Resetting...");
						uart_wait_tx_done(UART_NUM_1, 5000 / portTICK_PERIOD_MS);
						esp_restart();
				}
			} else if(!setupComplete) {
				printerr("notstarted", "Not yet initialized, message not sent", -1);
			} else {
				send.MsgID = (int) strtol(part, NULL, 16);
				send.FIR.B.DLC = 0;
				part = strtok(NULL, " ");
				while(part != NULL) {
					send.data.u8[send.FIR.B.DLC++] = (unsigned char) strtol(part, NULL, 16);
					part = strtok(NULL, " ");
				}
				CAN_write_frame(&send);
			}
			free(inputBuffer);
		}
	}
}

static void can_rx_task() {
	CAN_frame_t frame;
	char jsonbuf[256];
	char minijsonbuf[16];
	while(true) {
		if(xQueueReceive(CAN_cfg.rx_queue, &frame, portMAX_DELAY) == pdTRUE) {
			if(quickSuppressCheck(frame.MsgID))
				goto rx_task_end;

			if(filteringEnabled) {
				bool found = false;
				for(int i = 0; i < currentFilterSize; i++) {
					if(frame.MsgID == filter[i])
						found = true; 
				}
				if(found == filterSuppressing)
					goto rx_task_end;
			}

			sprintf(jsonbuf, "{\"id\": \"0x%.3lX\", \"dlc\": %1d, \"timestamp\": %lu, \"type\": \"standard\", \"data\": [", (unsigned long) frame.MsgID, frame.FIR.B.DLC, millis());
			for(int i = 0; i < frame.FIR.B.DLC; i++) {
				sprintf(minijsonbuf, "\"0x%.2X\"%s", frame.data.u8[i], ((i != frame.FIR.B.DLC-1) ? ", " : "]}\n"));
				strcat(jsonbuf, minijsonbuf);
			}
			print(jsonbuf);
		}

		rx_task_end:
		taskYIELD();
	}
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
	print("\n");
}

void initNVS() {
	esp_err_t e = nvs_flash_init();
	if (e == ESP_ERR_NVS_NO_FREE_PAGES) {
		printerr("nvsferr", "NVS is being initialized", e);
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
		if(e != ESP_OK) {
			printerr("nvsierr2", "NVS was not initialized", e);
		}
    } else if(e != ESP_OK) {
		printerr("nvsierr", "NVS was not initialized", e);
	}

	e = nvs_open(NVS_NAMESAPCE, NVS_READWRITE, &storage);
	if(e != ESP_OK)
		printerr("nvsoerr", "NVS was not opened", e);
}

void app_main() {
	initUART();
	initNVS();
	readBoardName();

	{
		char buf[75];
		sprintf(buf, "%s Connected", boardName);
		printmsg("poweron", buf);
	}

	xTaskCreate(uart_read_task, "uart_read_task", 1024*2, NULL, 10, NULL);
	xTaskCreate(can_tx_task, "can_tx_task", 1024*6, NULL, 8, NULL);
}
