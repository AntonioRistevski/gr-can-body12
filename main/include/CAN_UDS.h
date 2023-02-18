#ifndef __DRIVERS_CAN_UDS__
#define __DRIVERS_CAN_UDS__

#define CAN_UDS_BROADCAST_ARB 0x700

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/** \brief CAN Update Configuration */
typedef struct {
    uint16_t 		inId;     				/**< \brief Unique Arbitration ID for Module (Module Listens) */
	uint16_t 		outId;     				/**< \brief Unique Arbitration ID for Module (Module Speaks) */
	QueueHandle_t	queue;					/**< \brief CAN RX Queue */
	char			name[64];				/**< \brief Module Name */
	char			version[64];			/**< \brief Version Number */
} CAN_UDS_cfg_t;

typedef struct {
	bool enabled;
	union {
		uint32_t as_uint;
		int32_t as_int;
		uint8_t as_bytes[4];
	} access;
} CAN_UDS_cal_t;
#define UDS_CAL_VALUE_COUNT (20)
CAN_UDS_cal_t calibrations[UDS_CAL_VALUE_COUNT];

uint32_t getCalibrationOr(int index, uint32_t or);

int32_t getSignedCalibrationOr(int index, int32_t or);

/** \brief CAN Update Configuration Reference */
extern CAN_UDS_cfg_t CAN_UDS_cfg;

/**
 * \brief Initialize the CAN Update module. CAN must be initialized first.
 *
 * \return 0 CAN Module had been initialized
 */
int CAN_UDS_init(void);

#endif