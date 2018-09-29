#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

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
#define CAN_QUEUE_SZ 		(100)
#define CAN_SPEED			(CAN_SPEED_500KBPS)

#define MULTISAMPLING		(100)

#define CAL_USE_PRESSURE_SENSOR_1	(0)
#define CAL_USE_PRESSURE_SENSOR_2	(1)
#define CAL_USE_FAN_CONTROL			(2)
#define CAL_FAN_ON_TEMP				(3)
#define CAL_FAN_OFF_TEMP			(4)

typedef struct {
	int16_t data;
	unsigned long lastUpdate;
	unsigned long invalidAfter;
} CAN_int16_data_t;

typedef struct {
	bool data;
	unsigned long lastUpdate;
	unsigned long invalidAfter;
} CAN_bool_data_t;

QueueHandle_t uartEventQueue, uartLineQueue;
CAN_device_t CAN_cfg; // Name cannot change, extern reference
CAN_UDS_cfg_t CAN_UDS_cfg; // Name cannot change, extern reference
esp_adc_cal_characteristics_t characteristics;

bool transmit = true;

CAN_bool_data_t starter;
CAN_bool_data_t fuel;
CAN_bool_data_t fan;
CAN_bool_data_t brakeLight;
CAN_int16_data_t rpm;
CAN_int16_data_t fuelPressure;
CAN_int16_data_t coolant;
bool neutral = false;

int16_t voltageX = 0;
int16_t voltageY = 0;
int16_t voltageZ = 0;

int16_t VoltageXInt = 0;
int16_t VoltageYInt = 0;
int16_t VoltageZInt = 0;

bool fanRunningByAlgorithm = false;

static void uart_read_task();
static void uart_action_task();
static void can_rx_task();
static void ctrl_task();

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initVariables() {
	starter.data = false;
	starter.lastUpdate = 0;
	starter.invalidAfter = 250;
	fuel.data = false;
	fuel.lastUpdate = 0;
	fuel.invalidAfter = 250;
	fan.data = false;
	fan.lastUpdate = 0;
	fan.invalidAfter = 250;
	brakeLight.data = false;
	brakeLight.lastUpdate = 0;
	brakeLight.invalidAfter = 250;
	rpm.data = 0;
	rpm.lastUpdate = 0;
	rpm.invalidAfter = 100;
	fuelPressure.data = 0;
	fuelPressure.lastUpdate = 0;
	fuelPressure.invalidAfter = 100;
	coolant.data = 0;
	coolant.lastUpdate = 0;
	coolant.invalidAfter = 5000;
}

void initGPIO() {
	gpio_config_t io_conf;

	// GPIO 35 is Z - Axis | Accelerometer
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_35;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 35");

	// GPIO 34 is Y - Axis | Accelerometer
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_35;
	io_conf.pull_down_en = true;
	io_conf.pull_up_en = false;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 35");

	// GPIO 39 is X - Axis | Accelerometer
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_35;
	io_conf.pull_down_en = true;
	io_conf.pull_up_en = false;
	if(gpio_config(&io_conf) != ESP_OK)
		println("Error setting up GPIO 35");

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

	// Enable ADC
	adc_power_on();
	adc1_config_width(ADC_WIDTH_BIT_12);						//configure precision
	adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_11);	//and attetntuation | Channel 3 = gpio39 | 11 db = 0 to 3.9v attentuation
	adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_11);	//and attetntuation | Channel 6 = gpio34 | 11 db = 0 to 3.9v attentuation
	adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);	//and attetntuation | Channel 7 = gpio35 | 11 db = 0 to 3.9v attentuation
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1101, &characteristics);
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

bool i16DataIsValid(CAN_int16_data_t* data) {
	if(data->lastUpdate == 0)
		return false; // Data has never been updated
	if(data->lastUpdate + data->invalidAfter < millis())
		return false; // Data is no longer valid
	return true;
}

int16_t i16ValueOr(CAN_int16_data_t* data, int16_t ifInvalid) {
	if(i16DataIsValid(data))
		return data->data;
	return ifInvalid;
}

bool boolDataIsValid(CAN_bool_data_t* data) {
	if(data->lastUpdate == 0)
		return false; // Data has never been updated
	if(data->lastUpdate + data->invalidAfter < millis())
		return false; // Data is no longer valid
	return true;
}

bool boolValueOr(CAN_bool_data_t* data, bool ifInvalid) {
	if(boolDataIsValid(data))
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
				// && frame.data.bytes[0] == ~frame.data.bytes[1]
					if(frame.dlc == 2 || frame.dlc == 1) {
						starter.data = (frame.data.bytes[0] & 1);
						starter.lastUpdate = curTime;
						fuel.data = (frame.data.bytes[0] >> 1) & 1;
						fuel.lastUpdate = curTime;
						fan.data = (frame.data.bytes[0] >> 2) & 1;
						fan.lastUpdate = curTime;
					}
					break;
				case 0x120:
					brakeLight.data = 0;
					if(getCalibrationOr(CAL_USE_PRESSURE_SENSOR_1, true)) {
						brakeLight.data = ((frame.data.bytes[0] >> 1) & 1);
					}
					if(getCalibrationOr(CAL_USE_PRESSURE_SENSOR_2, true)) {
						brakeLight.data |= ((frame.data.bytes[0] >> 2) & 1);
					}
					brakeLight.lastUpdate = curTime;
					break;
				case 0x5F0:
					if(frame.dlc == 8) { // Probably check the DLC to make sure you're not getting garbage
						rpm.data = ((frame.data.bytes[6]) << 8) | frame.data.bytes[7]; // This is where you want to grab any data from the frame
						rpm.lastUpdate = millis();
					}
					break;
				case 0x5F2:
					if(frame.dlc == 8) {
						coolant.data = ((frame.data.bytes[6]) << 8) | frame.data.bytes[7];
						coolant.lastUpdate = millis();
					}
					break;
				case 0x5FD:
					if(frame.dlc == 8) {
						fuelPressure.data = (((frame.data.bytes[0]) << 8) | frame.data.bytes[1]);
						fuelPressure.lastUpdate = millis();
					}
			}
		}
	}
}

//bool lastFuel = false;
//uint8_t fuelFaults = 0;
static void can_tx_task() {
	unsigned char toSend[1];
	unsigned char accel[6];
	unsigned char accelinit[6];

	while(true) {		
		toSend[0] = neutral;
		toSend[0] |= (fanRunningByAlgorithm & 0x1) << 1;
		//toSend[1] = fuelFaults;
		
		if(transmit)
			CAN_send(0x10, 1, toSend);
		wait(5);

		accel[0] = voltageX >> 8;
		accel[1] = voltageX;
		accel[2] = voltageY >> 8;
		accel[3] = voltageY;
		accel[4] = voltageZ >> 8;
		accel[5] = voltageZ;

		if(transmit)
			CAN_send(0x380, 6, accel);
		wait(5);

		accelinit[0] = VoltageXInt >> 8;
		accelinit[1] = VoltageXInt;
		accelinit[2] = VoltageYInt >> 8;
		accelinit[3] = VoltageYInt;
		accelinit[4] = VoltageZInt >> 8;
		accelinit[5] = VoltageZInt;

		if(transmit)
			CAN_send(0x381, 6, accelinit);
		wait(5);
	}
}

bool fanShouldRun() {
	if(!getCalibrationOr(CAL_USE_FAN_CONTROL, true))
		return false; // Global disable

	uint16_t calculatedRPM = i16ValueOr(&rpm, 0);
	uint16_t calculatedClt = i16ValueOr(&coolant, 100);

	if(calculatedRPM < 1000)
		return false; // Do not run the fan if we're not running the car

	if(calculatedClt > getCalibrationOr(CAL_FAN_ON_TEMP, 180))
		fanRunningByAlgorithm = true;

	if(calculatedClt < getCalibrationOr(CAL_FAN_OFF_TEMP, 160))
		fanRunningByAlgorithm = false;

	return fanRunningByAlgorithm;
}

static void ctrl_task() {
	uint32_t adc1_gpio39 = 0;
	uint32_t adc1_gpio34 = 0;
	uint32_t adc1_gpio35 = 0;
	for(int i = 0; i < MULTISAMPLING; i++) {
		adc1_gpio39 += adc1_get_raw(ADC1_CHANNEL_3);	//read
		adc1_gpio34 += adc1_get_raw(ADC1_CHANNEL_7);
		adc1_gpio35 += adc1_get_raw(ADC1_CHANNEL_6);
	}
	adc1_gpio39 /= MULTISAMPLING;
	adc1_gpio34 /= MULTISAMPLING;
	adc1_gpio35 /= MULTISAMPLING;

	uint32_t pinVoltageXInt = esp_adc_cal_raw_to_voltage(adc1_gpio39, &characteristics); // this value returns to milivolts
	uint32_t pinVoltageYInt = esp_adc_cal_raw_to_voltage(adc1_gpio34, &characteristics);
	uint32_t pinVoltageZInt = esp_adc_cal_raw_to_voltage(adc1_gpio35, &characteristics);

	float sensorVoltageXInt = map(pinVoltageXInt, 0, 3000, -3, 3);
	float sensorVoltageYInt = map(pinVoltageYInt, 0, 3000, -3, 3);
	float sensorVoltageZInt = map(pinVoltageZInt, 0, 3000, -3, 3);

	VoltageXInt = sensorVoltageXInt * 1000;
	VoltageYInt = sensorVoltageYInt * 1000;
	VoltageZInt = sensorVoltageZInt * 1000;
	
	wait(5);

	while(true) {
		neutral = !gpio_get_level(GPIO_NUM_12);
		
		gpio_set_level(GPIO_NUM_32, boolValueOr(&starter, fanShouldRun()));
		// This is a fuel pressure regulator in software if you need it
		/*float fPressure = ((float) i16ValueOr(&fuelPressure, 0)) / 10.0;
		if(valueOr(&fuel, (i16ValueOr(&rpm, 105) > 100))) {
			if(fPressure > 55 && lastFuel) {
				lastFuel = false;
				fuelFaults++;
			}
			if(fPressure < 46) {
				lastFuel = true;
			}
			gpio_set_level(GPIO_NUM_33, lastFuel);
		} else {
			gpio_set_level(GPIO_NUM_33, false);
		}	*/
		gpio_set_level(GPIO_NUM_33, boolValueOr(&fuel, (i16ValueOr(&rpm, 105) > 100)));
		gpio_set_level(GPIO_NUM_25, boolValueOr(&fan, fanShouldRun()));
		gpio_set_level(GPIO_NUM_14, boolValueOr(&brakeLight, false));	//valueOr(&brakeLight, false)

		uint32_t adc1_gpio39 = adc1_get_raw(ADC1_CHANNEL_3);	//read
		uint32_t adc1_gpio34 = adc1_get_raw(ADC1_CHANNEL_7);
		uint32_t adc1_gpio35 = adc1_get_raw(ADC1_CHANNEL_6);
		uint32_t pinVoltageX = esp_adc_cal_raw_to_voltage(adc1_gpio39, &characteristics); // this value returns to milivolts
		uint32_t pinVoltageY = esp_adc_cal_raw_to_voltage(adc1_gpio34, &characteristics);
		uint32_t pinVoltageZ = esp_adc_cal_raw_to_voltage(adc1_gpio35, &characteristics);

		float sensorVoltageX = map(pinVoltageX, 0, 3000, -3, 3) - sensorVoltageXInt;
		float sensorVoltageY = map(pinVoltageY, 0, 3000, -3, 3) - sensorVoltageYInt;
		float sensorVoltageZ = map(pinVoltageZ, 0, 3000, -3, 3) - sensorVoltageZInt;

		voltageX = sensorVoltageX * 1000;
		voltageY = sensorVoltageY * 1000;
		voltageZ = sensorVoltageZ * 1000;

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
