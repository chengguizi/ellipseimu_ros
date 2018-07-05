/*!
 *	\file		binaryLogDvl.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		05 June 2013
 *
 *	\brief		This file is used to parse received DVL binary logs.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2013, SBG Systems SAS. All rights reserved.
 *	
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */
#ifndef __BINARY_LOG_DVL_H__
#define __BINARY_LOG_DVL_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log DVL status definitions                                         -//
//----------------------------------------------------------------------//

/*!
 * DVL status mask definitions
 */
#define	SBG_ECOM_DVL_VELOCITY_VALID		(0x0001u << 0)			/*!< Set to 1 if the DVL equipment was able to measure a valid velocity. */
#define SBG_ECOM_DVL_TIME_SYNC			(0x0001u << 1)			/*!< Set to 1 if the DVL data is correctly synchronized. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for DVL data.
 */
typedef struct _SbgLogDvlData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	uint16	status;					/*!< DVL status bitmask. */
	float	velocity[3];			/*!< X, Y, Z velocities in m.s^-1 expressed in the DVL instrument frame. */
	float	velocityStdDev[3];		/*!< X, Y, Z velocities standard deviation in m.s^-1. */
} SbgLogDvlData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK / SBG_ECOM_LOG_DVL_WATER_TRACK message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDvlData(const void *pPayload, uint32 payloadSize, SbgLogDvlData *pOutputData);

#endif
