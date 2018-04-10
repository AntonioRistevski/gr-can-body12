/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 * Copyright (c) 2018, Paul Hollinsky, hollinsky.com
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "CAN.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esp_intr.h"
#include "soc/dport_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "CAN_regdef.h"
#include "CAN_config.h"
#include "util.h"

static void CAN_read_frame();
static SemaphoreHandle_t sendMutex = NULL;
static SemaphoreHandle_t asyncMutex = NULL;

bool txComplete = false;
static void CAN_isr(void *arg_p){
	// Read interrupt status and clear flags
	__CAN_IRQ_t interrupt = MODULE_CAN->IR.U;

	// Handle TX complete interrupt
	if ((interrupt & __CAN_IRQ_TX) != 0)
		txComplete = true;

	// Handle RX frame available interrupt
	if ((interrupt & __CAN_IRQ_RX) != 0)
		CAN_read_frame();

	// Handle error interrupts.
	if ((interrupt & (__CAN_IRQ_ERR						//0x4
					  | __CAN_IRQ_DATA_OVERRUN			//0x8
					  | __CAN_IRQ_WAKEUP				//0x10
					  | __CAN_IRQ_ERR_PASSIVE			//0x20
					  | __CAN_IRQ_ARB_LOST				//0x40
					  | __CAN_IRQ_BUS_ERR				//0x80
	)) != 0) {
		/*handler*/
	}
}

static void CAN_read_frame(){
	//check if we have a queue. If not, operation is aborted.
	if (CAN_cfg.rx_queue == NULL) {
		// Let the hardware know the frame has been read.
		MODULE_CAN->CMR.B.RRB=1;
		return;
	}

	//read frame from hardware
	CAN_frame_t frame;
	frame.FIR.U = MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;
	frame.dlc = frame.FIR.B.DLC;

	//check if this is a standard or extended CAN frame
	if(frame.FIR.B.FF == CAN_frame_std) { //standard frame
		//get arbitration id
		frame.id = _CAN_GET_STD_ID;

		//deep copy data bytes
		for(int i = 0; i < frame.dlc; i++)
			frame.data.bytes[i] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[i];
	}
	else { //extended frame
		//get arbitration id
		frame.id = _CAN_GET_EXT_ID;

		//deep copy data bytes
		for(int i = 0; i < frame.dlc; i++)
			frame.data.bytes[i] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[i];
	}

	//send frame to input queue
	xQueueSendFromISR(CAN_cfg.rx_queue, &frame, 0);

	//Let the hardware know the frame has been read.
	MODULE_CAN->CMR.B.RRB = 1;
}

int CAN_write_frame(const CAN_frame_t* p_frame){
	if(xSemaphoreTake(asyncMutex, portMAX_DELAY) == pdFALSE)
		return -1; // Timed out

	portMUX_TYPE mux;
	vPortCPUInitializeMutex(&mux);
	if(!vPortCPUAcquireMutexTimeout(&mux, portMUX_NO_TIMEOUT))
		return -2; // Could not aquire mux
	taskENTER_CRITICAL(&mux);

	//write frame information record
	MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = p_frame->FIR.U;
	MODULE_CAN->MBX_CTRL.FCTRL.FIR.B.DLC = p_frame->dlc;

	//check if this is a standard or extended CAN frame
	if(p_frame->FIR.B.FF==CAN_frame_std) { //standard frame
		//write message ID
		_CAN_SET_STD_ID(p_frame->id);

		//write the frame data to the hardware
		for(int i = 0; i < p_frame->dlc; i++)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[i] = p_frame->data.bytes[i];
	}
	else { //extended frame
		//write message ID
		_CAN_SET_EXT_ID(p_frame->id);

		//write the frame data to the hardware
		for(int i = 0; i < p_frame->dlc; i++)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[i] = p_frame->data.bytes[i];
	}

	// Transmit frame
	MODULE_CAN->CMR.B.TR=1;
	taskEXIT_CRITICAL(&mux);
	vPortCPUReleaseMutex(&mux);
	xSemaphoreGive(asyncMutex);
	return 0;
}

int CAN_write_frame_sync(const CAN_frame_t* p_frame) {
	if(xSemaphoreTake(sendMutex, portMAX_DELAY) == pdFALSE)
		return -1; // Timed out
	txComplete = false;
	int res = CAN_write_frame(p_frame);
	if(res)
		return res; // Do not block, because we got an error
    unsigned long endTime = millis() + 500;
	while(!txComplete && millis() < endTime) // Blocking action
        taskYIELD();
    if(!txComplete)
        res = -1; // Mark that we timed out
	txComplete = false;
	xSemaphoreGive(sendMutex);
	return res;
}

int CAN_send(int id, int dlc, unsigned char data[]) {
	CAN_frame_t send;
	send.FIR.B.FF = CAN_frame_std;
	send.FIR.B.RTR = 0;
	send.FIR.B.reserved_24 = 0;
	send.FIR.B.unknown_2 = 0;
	send.id = id;
	send.dlc = dlc;
	for(int i = 0; i < dlc; i++)
		send.data.bytes[i] = data[i];
	return CAN_write_frame_sync(&send);
}

int CAN_ISO_send(const uint16_t id, const uint16_t dlc, const unsigned char data[]) {
	if(dlc > 4095)
		return -1; // Frame too big!
	int i = 0;
	CAN_frame_t send;
	send.FIR.B.FF = CAN_frame_std;
	send.FIR.B.RTR = 0;
	send.FIR.B.reserved_24 = 0;
	send.FIR.B.unknown_2 = 0;
	send.id = id;
	send.dlc = 8;
	for(i = 0; i < 8; i++)
		send.data.bytes[i] = CAN_PADDING_BYTE;

	if(dlc < 8) {
		send.data.bytes[0] = dlc;
		for(i = 1; i <= dlc; i++)
			send.data.bytes[i] = data[i-1];
		return CAN_write_frame_sync(&send);
	} else {
		int sent = 0;
		send.data.bytes[0] = 0x10; // Start frame
		send.data.bytes[0] |= 0x0f & (dlc << 16);
        send.data.bytes[1] = dlc;
		for(i = 2; i < 8; i++)
			send.data.bytes[i] = data[i-2];
		sent += 6;
		int res = CAN_write_frame_sync(&send);
		if(res)
			return res; // Failure
		
		uint8_t index = 1;
		while(sent < dlc) {
			wait(CAN_ISO_WAIT_TIME);

			index &= 0xf; // Throw away upper 4 bits
			send.data.bytes[0] = 0x20 | index++; // Continue frame
			for(i = 1; i < 8; i++) {
				if(sent + (i-1) >= dlc)
					send.data.bytes[i] = CAN_PADDING_BYTE;
				else
					send.data.bytes[i] = data[sent + i - 1];
			}
			sent += 7;
			int res = CAN_write_frame_sync(&send);
			if(res)
				return res; // Failure
		}
		return 0;
	}
}

int CAN_ISO_str_send(const uint16_t id, const char* str) {
	return CAN_ISO_send(id, strlen(str), (const unsigned char*) str);
}

void CAN_ISO_frame_minify(const CAN_ISO_static_frame_t* src, CAN_ISO_frame_t* dest) {
	dest->id = src->id;
	dest->dlc = src->dlc;
	dest->data = malloc(dest->dlc);
	memcpy(dest->data, src->data, dest->dlc);
}

void CAN_ISO_static_frame_invalidate(CAN_ISO_static_frame_t* frame) {
	frame->dlc = 0;
	frame->lastFrame.id = 0x800; // Not a valid Arb ID, so used as a flag
	frame->lastIndex = 0;
	frame->dataReceived = 0;
}

int CAN_init() {
	MODULE_CAN->MOD.B.RM = 1;

	if(sendMutex == NULL)
		sendMutex = xSemaphoreCreateMutex();
	if(sendMutex == NULL)
		return -1; // The mutex could not be created successfully

    if(asyncMutex == NULL)
		asyncMutex = xSemaphoreCreateMutex();
	if(asyncMutex == NULL)
		return -2; // The mutex could not be created successfully

	//enable module
	DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
	DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);

	//configure TX pin
	gpio_set_level(CAN_cfg.tx_pin_id, 1);
	gpio_set_direction(CAN_cfg.tx_pin_id, GPIO_MODE_OUTPUT);
	gpio_matrix_out(CAN_cfg.tx_pin_id, CAN_TX_IDX, 0, 0);
	gpio_pad_select_gpio(CAN_cfg.tx_pin_id);

	//configure RX pin
	gpio_set_direction(CAN_cfg.rx_pin_id, GPIO_MODE_INPUT);
	gpio_matrix_in(CAN_cfg.rx_pin_id,CAN_RX_IDX, 0);
	gpio_pad_select_gpio(CAN_cfg.rx_pin_id);

	//set to PELICAN mode
	MODULE_CAN->CDR.B.CAN_M = 0x1;

	//synchronization jump width is the same for all baud rates
	MODULE_CAN->BTR0.B.SJW = 0x1;

	//TSEG2 is the same for all baud rates
	MODULE_CAN->BTR1.B.TSEG2 = 0x1;

	//time quantum
	double __tq;

	//select time quantum and set TSEG1
	switch(CAN_cfg.speed){
		case CAN_SPEED_1000KBPS:
			MODULE_CAN->BTR1.B.TSEG1	=0x4;
			__tq = 0.125;
			break;
		case CAN_SPEED_800KBPS:
			MODULE_CAN->BTR1.B.TSEG1	=0x6;
			__tq = 0.125;
			break;
		case CAN_SPEED_83K3BPS:
			MODULE_CAN->BTR1.B.TSEG1	=0xc;
			__tq = ((float)1000/83.3) / 16;
		default:
			MODULE_CAN->BTR1.B.TSEG1	=0xc;
			__tq = ((float)1000/CAN_cfg.speed) / 16;
	}

	//set baud rate prescaler
	MODULE_CAN->BTR0.B.BRP = (uint8_t) round((((APB_CLK_FREQ * __tq) / 2) - 1) / 1000000) - 1;

	/* Set sampling
	 * 1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     (class A and B) where filtering spikes on the bus line is beneficial
	 * 0 -> single; the bus is sampled once; recommended for high speed buses (SAE class C)*/
	MODULE_CAN->BTR1.B.SAM = (CAN_cfg.speed < 125);

	//enable all interrupts
	MODULE_CAN->IER.U = 0xff;

	//no acceptance filtering, as we want to fetch all messages
	MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0;
	MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0;
	MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0;
	MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0;
	MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;

	//set to normal mode
	MODULE_CAN->OCR.B.OCMODE = __CAN_OC_NOM;

	//clear error counters
	MODULE_CAN->TXERR.U = 0;
	MODULE_CAN->RXERR.U = 0;
	(void)MODULE_CAN->ECC;

	//clear interrupt flags
	(void)MODULE_CAN->IR.U;

	//install CAN ISR
	esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, CAN_isr, NULL, NULL);

	//Showtime. Release Reset Mode.
	MODULE_CAN->MOD.B.RM = 0;
	return 0;
}

int CAN_stop(){
	//enter reset mode
	MODULE_CAN->MOD.B.RM = 1;
	return 0;
}
