#ifndef __DRIVERS_CAN_UDS__
#define __DRIVERS_CAN_UDS__

#define CAN_UDS_BROADCAST_ARB 0x700
#define CAN_UDS_BROADCAST_RATE (2500)

/** \brief CAN Update Configuration */
typedef struct {
    uint16_t 		inId;     				/**< \brief Unique Arbitration ID for Module (Module Listens) */
	uint16_t 		outId;     				/**< \brief Unique Arbitration ID for Module (Module Speaks) */
	uint16_t 		broadcastId;	  		/**< \brief Unique Arbitration ID for Module (Module Broadcasts) */
	QueueHandle_t	queue;					/**< \brief CAN RX Queue */
	char[64]		name;					/**< \brief Module Name */
	char[64]		version;				/**< \brief Version Number */
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