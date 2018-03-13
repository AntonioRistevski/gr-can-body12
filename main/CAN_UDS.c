#include "CAN_UDS.h"

#include "freertos/task.h"
#include "esp_ota_ops.h"

#include "CAN.h"
#include "util.h"

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
			if(frame.id == CAN_UDS_cfg.inId) {
				uint8_t ctrl = frame.data.bytes[0] >> 4;

				if(ctrl == 1 && isoFrame.lastFrame.id != 0x800)
					CAN_ISO_static_frame_invalidate(&isoFrame);

				if(isoFrame.lastFrame.id == 0x800) { // New ISO-TP packet
					isoFrame.lastFrame = frame;
					switch(ctrl) {
						case 0x1:
							isoFrame.dlc = frame.data.words[0] & 0x0fff;
							for(i = 2; i < 8; i++)
								isoFrame.data[i-2] = frame.data.bytes[i];
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
					
					if(index != indexShouldBe)
						CAN_ISO_static_frame_invalidate(&isoFrame);

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
	int i;
	CAN_ISO_frame_t frame;
	esp_ota_handle_t otaHandle;
	const esp_partition_t* nextPartition = NULL;
	size_t updateSize = NULL;
	size_t bytesWritten = 0;
	esp_err_t err;
	uint8_t buf[260];
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
				case 0x23: // Read Memory By Address
					if(frame.dlc != 6) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x13; // Incorrect message length or invalid format
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					uint8_t* memory = (uint8_t*) (frame.data[1] << 24 | frame.data[2] << 16 | frame.data[3] << 8 | frame.data[4]);
					uint8_t amt = frame.data[5];
					for(i = 0; i < amt; i++)
						buf[i+1] = *(memory + i);
					CAN_ISO_send(CAN_UDS_cfg.outId, amt+1, buf);
					break;
				case 0x34: // Request Download
					if(uploading) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x22; // Conditions not correct
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					bytesWritten = 0;
					updateSize = (frame.data[1] << 24 | frame.data[2] << 16 | frame.data[3] << 8 | frame.data[4]);
					nextPartition = esp_ota_get_next_update_partition(NULL);
					err = esp_ota_begin(nextPartition, updateSize, &otaHandle);
					if(err != ESP_OK) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x10; // General reject
						buf[3] = err;
						CAN_ISO_send(CAN_UDS_cfg.outId, 4, buf);
						break;
					}

					uploading = true;
					buf[1] = 0x00; // 16-bit block size
					buf[2] = 0xff; // Currently 255 bytes per block
					CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
					break;
				case 0x36: // Transfer Data
					if(!uploading) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x22; // Conditions not correct
						CAN_ISO_send(CAN_UDS_cfg.outId, 3, buf);
						break;
					}

					uint16_t bytes = frame.dlc - 1;
					err = esp_ota_write(otaHandle, frame.data+1, bytes);
					if(err != ESP_OK) {
						buf[0] = 0x7f; // Reject
						buf[1] = sid;
						buf[2] = 0x10; // General reject
						buf[3] = err;
						buf[4] = 0x00; // esp_ota_write
						CAN_ISO_send(CAN_UDS_cfg.outId, 5, buf);
						break;
					}

					bytesWritten += bytes;
					if(bytesWritten >= updateSize) {
						err = esp_ota_end(otaHandle);
						if(err != ESP_OK) {
							buf[0] = 0x7f; // Reject
							buf[1] = sid;
							buf[2] = 0x10; // General reject
							buf[3] = err;
							buf[4] = 0x01; // esp_ota_end
							CAN_ISO_send(CAN_UDS_cfg.outId, 5, buf);
							break;
						}

						err = esp_ota_set_boot_partition(nextPartition);
						if(err != ESP_OK) {
							buf[0] = 0x7f; // Reject
							buf[1] = sid;
							buf[2] = 0x10; // General reject
							buf[3] = err;
							buf[4] = 0x02; // esp_ota_set_boot_partition
							CAN_ISO_send(CAN_UDS_cfg.outId, 5, buf);
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

			free(frame.data); // Always free the data malloc'd by CAN_ISO_frame_minify
		}
	}
}

static void CAN_UDS_broadcast_task() {
	char buff[33];
	unsigned char toSend[5];
	char version[145];
	sprintf(version, "{\"n\":\"%s\",\"v\":\"%s\"}", CAN_UDS_cfg.name, CAN_UDS_cfg.version);
	while(!stop) {
		wait(CAN_UDS_BROADCAST_RATE);
		print(version);
		print("\n");

		toSend[0] = CAN_UDS_cfg.outId >> 8;
		toSend[1] = CAN_UDS_cfg.outId;
		toSend[2] = CAN_UDS_cfg.inId >> 8;
		toSend[3] = CAN_UDS_cfg.inId;
		toSend[4] = CAN_UDS_cfg.broadcastId >> 8;
		toSend[5] = CAN_UDS_cfg.broadcastId;
		toSend[6] = uploading | (updated << 1);
		print("send1\n");
		int r = CAN_send(CAN_UDS_BROADCAST_ARB, 7, toSend);
		itoa(r, buff, 10);
		println(buff);

		if(uploading)
			continue; // Don't transmit version while uploading

		r = CAN_ISO_str_send(CAN_UDS_cfg.broadcastId, version);
		itoa(r, buff, 10);
		println(buff);
	}
}

int CAN_UDS_init() {
	stop = false;

	isoQueue = xQueueCreate(10, sizeof(CAN_ISO_frame_t));
	if(isoQueue == pdFALSE)
		return -1; // Queue could not be created

	xTaskCreate(CAN_UDS_broadcast_task, "CAN_UDS_broadcast_task", 1024 * 4, NULL, 7, NULL);
	xTaskCreate(CAN_UDS_rx_task, "CAN_UDS_rx_task", 1024 * 6, NULL, 7, NULL);
	xTaskCreate(CAN_UDS_ISO_task, "CAN_UDS_ISO_task", 1024 * 3, NULL, 6, NULL);
	return 0;
}

void CAN_UDS_stop() {
	stop = true;
	if(isoQueue != NULL)
		vQueueDelete(isoQueue);
}