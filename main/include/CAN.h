/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
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
 */

#ifndef __DRIVERS_CAN_H__
#define __DRIVERS_CAN_H__

#include "CAN_config.h"

#define CAN_PADDING_BYTE (0xAA)
#define CAN_ISO_WAIT_TIME (0)

#define CAN_STD_ID_MAX (0x7FF)
#define CAN_EXT_ID_MAX (0x3FFFFFFF)

/**
 * \brief CAN frame type (standard/extended)
 */
typedef enum {
	CAN_frame_std = 0, 						/**< Standard frame, using 11 bit identifer. */
	CAN_frame_ext = 1 						/**< Extended frame, using 29 bit identifer. */
} CAN_frame_format_t;

/**
 * \brief CAN RTR
 */
typedef enum {
	CAN_no_RTR = 0, 						/**< No RTR frame. */
	CAN_RTR = 1 							/**< RTR frame. */
}CAN_RTR_t;

/** \brief Frame information record type */
typedef union{uint32_t U;					/**< \brief Unsigned access */
	 struct {
		uint8_t 			DLC:4;        	/**< \brief [3:0] DLC, Data length container */
		unsigned int 		unknown_2:2;    /**< \brief \internal unknown */
		CAN_RTR_t 			RTR:1;          /**< \brief [6:6] RTR, Remote Transmission Request */
		CAN_frame_format_t 	FF:1;           /**< \brief [7:7] Frame Format, see# CAN_frame_format_t*/
		unsigned int 		reserved_24:24;	/**< \brief \internal Reserved */
	} B;
} CAN_FIR_t;


/** \brief CAN Frame structure */
typedef struct {
	CAN_FIR_t	FIR;						/**< \brief Frame information record */
    uint32_t 	id;     					/**< \brief Message ID */
	uint8_t		dlc;						/**< \brief Message Data Length */
    union {
        uint8_t u8[8];						/**< \brief Payload byte access */
		uint8_t bytes[8];					/**< \brief Payload byte access (nicer name) */
		uint16_t u16[4];					/**< \brief Payload word access */
		uint16_t words[4];					/**< \brief Payload word access (nicer name) */
        uint32_t u32[2];					/**< \brief Payload u32 access */
    } data;
} CAN_frame_t;

/** \brief CAN ISO-TP Frame structure */
typedef struct {
    uint16_t 	id;     					/**< \brief Message ID */
	uint16_t	dlc;						/**< \brief Message Data Length */
    uint8_t*	data;						/**< \brief Message Data */
} CAN_ISO_frame_t;

/** \brief CAN ISO-TP Frame static structure */
typedef struct {
    uint16_t 		id;     				/**< \brief Message ID */
	uint16_t		dlc;					/**< \brief Message Data Length */
	CAN_frame_t		lastFrame;				/**< \brief Last frame received */
	uint8_t			lastCtrl;				/**< \brief Control bits from last frame */
	uint8_t			lastIndex;				/**< \brief Index from last frame */
	uint16_t		dataReceived;			/**< \brief Length of data buffer*/
    uint8_t			data[4095];				/**< \brief Message Data */
} CAN_ISO_static_frame_t;

/**
 * \brief Initialize the CAN Module
 *
 * \return 0 CAN Module had been initialized
 */
int CAN_init(void);

/**
 * \brief Send a can frame
 *
 * \param	p_frame	Pointer to the frame to be send, see #CAN_frame_t
 * \return  0 Frame has been written to the module
 */
int CAN_write_frame(const CAN_frame_t* p_frame);

/**
 * \brief Send a can frame, synchronously
 *
 * \param	p_frame	Pointer to the frame to be send, see #CAN_frame_t
 * \return  0 Frame has been written to the bus
 */
int CAN_write_frame_sync(const CAN_frame_t* p_frame);

/**
 * \brief Send a standard can frame, synchronously
 *
 * \param	id The arbitration ID to use
 * \param	dlc The length of the data
 * \param	data The data to send
 * \return  0 Frame has been written to the bus
 */
int CAN_send(int id, int dlc, unsigned char data[]);

/**
 * \brief Send ISO-TP frames, synchronously
 *
 * \param	id The arbitration ID to use
 * \param	dlc The length of the data
 * \param	data The data to send
 * \return  0 Frames have been written to the bus
 */
int CAN_ISO_send(const uint16_t id, const uint16_t dlc, const unsigned char data[]);

/**
 * \brief Send a (null terminated) string as ISO-TP frames, synchronously
 *
 * \param	id The arbitration ID to use
 * \param	str The data to send
 * \return  0 Frames have been written to the bus
 */
int CAN_ISO_str_send(const uint16_t id, const char* str);

/**
 * \brief Minify an ISO-TP frame (uses malloc!)
 *
 * \param	src Static Frame
 * \param	dest Dynamic Frame
 */
void CAN_ISO_frame_minify(const CAN_ISO_static_frame_t* src, CAN_ISO_frame_t* dest);

/**
 * \brief Invalidate an ISO-TP frame
 *
 * \param	frame Static Frame
 */
void CAN_ISO_static_frame_invalidate(CAN_ISO_static_frame_t* frame);

/**
 * \brief Stops the CAN Module
 *
 * \return 0 CAN Module was stopped
 */
int CAN_stop(void);

#endif
