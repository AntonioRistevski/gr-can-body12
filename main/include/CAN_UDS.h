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

/** \brief CAN Update Configuration Reference */
extern CAN_UDS_cfg_t CAN_UDS_cfg;

/**
 * \brief Initialize the CAN Update module. CAN must be initialized first.
 *
 * \return 0 CAN Module had been initialized
 */
int CAN_UDS_init(void);

#endif