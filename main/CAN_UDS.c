#include "CAN_UDS.h"

#include "freertos/task.h"
#include "esp_ota_ops.h"

#include "CAN.h"
#include "util.h"
#include <string.h>

static bool stop = false;
static bool uploading = false;
static bool updated = false;
static QueueHandle_t isoQueue;

static void CAN_UDS_rx_task() {
	int i;
	CAN_frame_t frame;
	CAN_ISO_frame_t sendFrame;
	CAN_ISO_static_frame_t isoFrame;
	isoFrame.id = CAN_UDS_cfg.inId;
	isoFrame.lastFrame.id = 0x800; // Not a valid Arb ID, so used as a flag
	while(!stop) {
		if(xQueueReceive(CAN_UDS_cfg.queue, &frame, portMAX_DELAY) == pdTRUE) {
			if(frame.id == CAN_UDS_cfg.inId || (frame.id == 0x700 && isoFrame.lastFrame.id == 0x800)) {
				uint8_t ctrl = frame.data.bytes[0] >> 4;

				if(ctrl == 1 && isoFrame.lastFrame.id != 0x800)
					CAN_ISO_static_frame_invalidate(&isoFrame);

				if(isoFrame.lastFrame.id == 0x800) { // New ISO-TP packet
					isoFrame.lastFrame = frame;
					switch(ctrl) {
						case 0x1:
							isoFrame.dlc = ((frame.data.bytes[0] & 0x0f) << 8) | frame.data.bytes[1];
							for(i = 2; i < 8; i++)
								isoFrame.data[i-2] = frame.data.bytes[i];
							isoFrame.dataReceived += 6;

							unsigned char buf[3];
							buf[0] = 0x30; // Flow control
							buf[1] = 0x00; // Unlimited blocks
							buf[2] = 0x00; // With no spacing needed
							CAN_send(CAN_UDS_cfg.outId, 3, buf);
							break;
						case 0x0:
							isoFrame.dlc = frame.data.bytes[0];
							for(i = 1; i <= isoFrame.dlc; i++)
								isoFrame.data[i-1] = frame.data.bytes[i];
							isoFrame.dataReceived += 6;
							CAN_ISO_frame_minify(&isoFrame, &sendFrame);
							xQueueSend(isoQueue, &sendFrame, portMAX_DELAY);
							// intentionally not breaking here, iso frame is over
						default:
							CAN_ISO_static_frame_invalidate(&isoFrame);
					}
				} else {
					uint8_t index = frame.data.bytes[0] & 0xf;
					uint8_t indexShouldBe = (isoFrame.lastIndex+1) & 0xf;
					
					if(index != indexShouldBe) {
						unsigned char buf[4];
						buf[0] = index; // Reject
						buf[1] = ctrl;
						buf[2] = indexShouldBe; // Conditions not correct
						CAN_ISO_send(2, 3, buf);
						CAN_ISO_static_frame_invalidate(&isoFrame);
					}

					if(ctrl != 2 || isoFrame.lastFrame.id == 0x800) // Second condition is checking if frame was invalidated
						continue; // Makes sure flow control (ctrl=3) does not get parsed

					for(i = 1; i < 8; i++)
						isoFrame.data[i + isoFrame.dataReceived - 1] = frame.data.bytes[i];

					isoFrame.dataReceived += 7;
					isoFrame.lastFrame = frame;
					isoFrame.lastIndex = index;

					if(isoFrame.dataReceived >= isoFrame.dlc) {
						CAN_ISO_frame_minify(&isoFrame, &sendFrame);
						xQueueSend(isoQueue, &sendFrame, portMAX_DELAY);
						CAN_ISO_static_frame_invalidate(&isoFrame);
					}
				}
			}
		}
	}
}

static void CAN_UDS_ISO_task() {
	CAN_ISO_frame_t frame;
	esp_ota_handle_t otaHandle;
	const esp_partition_t* nextPartition = NULL;
	size_t updateSize = NULL;
	size_t bytesWritten = 0;
	esp_err_t err;
	uint8_t buf[1024];
	int lastUpdateIndex = -1;
	int lastBytes = -1;
	while(!stop) {
		if(xQueueReceive(isoQueue, &frame, portMAX_DELAY) == pdTRUE) {
			uint8_t sid = frame.data[0];
			buf[0] = sid + 0x40; // Standard response sid
			//uint8_t sf = frame.data[1]; subfunction, not used yet
			switch(sid) {
				case 0x11: // ECU Reset
					if(uploading) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x22; // Conditions not correct
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					buf[1] = 0; // OK
					CAN_ISO_send(CAN_UDS_cfg.outId, 2, buf);
					wait(100);
					CAN_stop();
					wait(100);
					esp_restart();
					break;
				case 0x22: // Read Data By Identifier (DID)
					if(uploading) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x22; // Conditions not correct
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					switch(frame.data[1] << 8 | frame.data[2]) {
						case 0x0000: { // Module Information
							char version[148];
							sprintf(version + 3, "{\"n\":\"%s\",\"v\":\"%s\"}", CAN_UDS_cfg.name, CAN_UDS_cfg.version);
							version[0] = sid + 0x40; // Response ID
							version[1] = frame.data[1];
							version[2] = frame.data[2];
							int msglen = strlen((const char*) (version+3)) + 3;
							CAN_ISO_send(CAN_UDS_cfg.outId, msglen, (const unsigned char*) version);
							break;
						}
						default:
							buf[0] = 0x7f; // Reject
							buf[1] = sid;
							buf[2] = 0x31; // Request out of range
							CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
							break;
					}
					break;
				case 0x23: // Read Memory By Address
					if(frame.dlc != 6) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x13; // Incorrect message length or invalid format
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}
					
					buf[0] = 0x7f; // Reject
					buf[1] = sid;
					buf[2] = 0x31; // Request out of range (always for now)
					CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
					// uint8_t* memory = (uint8_t*) (frame.data[1] << 24 | frame.data[2] << 16 | frame.data[3] << 8 | frame.data[4]);
					// uint8_t amt = frame.data[5];
					// for(i = 0; i < amt; i++)
					// 	buf[i+1] = *(memory + i);
					// CAN_ISO_send(CAN_UDS_cfg.outId, amt+1, buf);
					break;
				case 0x34: // Request Download
					if(uploading) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x22; // Conditions not correct
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					lastBytes = -1;

					bytesWritten = 0;
					updateSize = (frame.data[1] << 24 | frame.data[2] << 16 | frame.data[3] << 8 | frame.data[4]);
					nextPartition = esp_ota_get_next_update_partition(NULL);
					err = esp_ota_begin(nextPartition, updateSize, &otaHandle);
					if(err != ESP_OK) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x10; // General reject
						buf[3] = err >> 8;
						buf[4] = err;
						CAN_ISO_send(CAN_UDS_cfg.outId, 5, buf);
						break;
					}

					uploading = true;
					buf[1] = 0x00; // 16-bit block size
					buf[2] = 0xff; // Currently 255 bytes per block
					CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);

					vTaskPrioritySet(NULL, 8);
					break;
				case 0x36: // Transfer Data
					if(!uploading) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x22; // Conditions not correct
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					int indexShouldBe = (lastUpdateIndex + 1) & 0xff;
					if(frame.data[1] != indexShouldBe) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x10; // General reject
						buf[3] = frame.data[1];
						buf[4] = indexShouldBe;
						buf[5] = 0xff; // Index out of order
						CAN_ISO_send(CAN_UDS_cfg.outId, 6, buf);
						break;
					}
					lastUpdateIndex = frame.data[1];

					uint16_t bytes = frame.dlc - 2;
					if(lastBytes == -1)
						lastBytes = bytes;
					if(bytes != lastBytes && bytesWritten + bytes < updateSize) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x10; // General reject
						buf[3] = bytes;
						buf[4] = lastBytes;
						buf[5] = 0xfe; // Wrong size
						CAN_ISO_send(CAN_UDS_cfg.outId, 6, buf);
						break;
					}
					err = esp_ota_write(otaHandle, frame.data+2, bytes);
					if(err != ESP_OK) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x10; // General reject
						buf[3] = err >> 16;
						buf[4] = err >> 8;
						buf[5] = err;
						buf[6] = 0x00; // esp_ota_write
						CAN_ISO_send(CAN_UDS_cfg.outId, 7, buf);
						break;
					}

					bytesWritten += bytes;
					if(bytesWritten >= updateSize) {
						err = esp_ota_end(otaHandle);
						if(err != ESP_OK) {
							buf[0] = 0x7f; // Reject
							buf[1] = sid;
							buf[2] = 0x10; // General reject
							buf[3] = err >> 16;
							buf[4] = err >> 8;
							buf[5] = err;
							buf[6] = 0x01; // esp_ota_end
							CAN_ISO_send(CAN_UDS_cfg.outId, 7, buf);
							break;
						}

						err = esp_ota_set_boot_partition(nextPartition);
						if(err != ESP_OK) {
							buf[0] = 0x7f; // Reject
							buf[1] = sid;
							buf[2] = 0x10; // General reject
							buf[3] = err >> 16;
							buf[4] = err >> 8;
							buf[5] = err;
							buf[6] = 0x02; // esp_ota_set_boot_partition
							CAN_ISO_send(CAN_UDS_cfg.outId, 7, buf);
							break;
						}

						buf[1] = 0x00; // Okay, ready to restart
						CAN_ISO_send(CAN_UDS_cfg.outId, 2, buf);
						uploading = false;
						updated = true;
					} else {
						buf[1] = 0x01; // Okay, ready for more
						buf[2] = bytesWritten >> 24;
						buf[3] = bytesWritten >> 16;
						buf[4] = bytesWritten >> 8;
						buf[5] = bytesWritten;
						CAN_ISO_send(CAN_UDS_cfg.outId, 6, buf);
					}
					break;
				case 0x37: // Request Transfer Exit
					if(!uploading) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x22; // Conditions not correct
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					uploading = false;
					buf[1] = esp_ota_end(otaHandle);
					CAN_ISO_send(CAN_UDS_cfg.outId, 2, buf);
					break;
				default:
					buf[0] = 0x7f; // Reject
					buf[1] = sid;
					buf[2] = 0x11; // Service not supported
					CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
			}

			vPortFree(frame.data); // Always free the data malloc'd by CAN_ISO_frame_minify
		}
	}
}

int CAN_UDS_init() {
	stop = false;

	isoQueue = xQueueCreate(15, sizeof(CAN_ISO_frame_t));
	if(isoQueue == pdFALSE)
		return -1; // Queue could not be created

	xTaskCreate(CAN_UDS_rx_task, "CAN_UDS_rx_task", 1024 * 6, NULL, 7, NULL);
	xTaskCreate(CAN_UDS_ISO_task, "CAN_UDS_ISO_task", 1024 * 3, NULL, 8, NULL);
	return 0;
}

void CAN_UDS_stop() {
	stop = true;
	if(isoQueue != NULL)
		vQueueDelete(isoQueue);
}