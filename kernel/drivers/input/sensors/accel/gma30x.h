/* 
 * Copyright (c) 2014 Globalmems, Inc.  All rights reserved.
 *
 * This source is subject to the Globalmems Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of Globalmems Inc.
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#ifndef GMA30X_H
#define GMA30X_H

#include <linux/ioctl.h>
//#define EWMA_FILTER				/* Default:Enable EWMA ,Do not disable */
//#define SMA_FILTER				/* Enable or Disable SMA. Default:Disable */
/* Exponentially weighted moving average (EWMA) */
#ifdef EWMA_FILTER
#define EWMA_POSITIVE	10000	/* The values ​​are all positive */
#define EWMA_FACTOR		1024	/* Magnification 2^10=1024，All values ​​<< 10 */
#define EWMA_WEIGHT_X	2		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
#define EWMA_WEIGHT_Y	2		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
#define EWMA_WEIGHT_Z	4		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
#endif
/* for Simple Moving Average (SMA) */
#ifdef SMA_FILTER	/* Simple Moving Average */
#define SMA_AVG	4	/* AVG sensor data */
#endif

#define AutoZeroZ	1	/* Default Disable.	Z asix AutoZero (GRAVITY_ON_Z AUTO) */
#define AutoZeroY	0	/* Default Disable.	Y asix AutoZero (GRAVITY_ON_Y AUTO) */
#define AutoZeroX	0	/* Default Disable.	X asix AutoZero (GRAVITY_ON_X AUTO) */
#define LevelValueRange_0_0078125	(GRAVITY_EARTH_1000/128)/* Level conditions about 0.0078125g,ps: gma303 1g=256(LSB) |X| < 2(LSB) && |Y| < 2(LSB)*/
#define LevelValueRange_0_015625	(GRAVITY_EARTH_1000/64)	/* Level conditions about 0.015625g	,ps: |X| < 4(LSB) && |Y| < 4(LSB)*/
#define LevelValueRange_0_03125		(GRAVITY_EARTH_1000/32)	/* Level conditions about 0.03125g	,ps: |X| < 8(LSB) && |Y| < 8(LSB)*/
#define LevelValueRange_0_0625		(GRAVITY_EARTH_1000/16)	/* Level conditions about 0.0625g	,ps: |X| < 16(LSB) && |Y| < 16(LSB)*/
#define LevelValueRange_0_1			(GRAVITY_EARTH_1000/10)	/* Level conditions about 0.1000g	,ps: |X| < 25(LSB) && |Y| < 25(LSB)*/
#define LevelValueRange_0_125		(GRAVITY_EARTH_1000/8)	/* Level conditions about 0.1250g	,ps: |X| < 32(LSB) && |Y| < 32(LSB)*/
#define LevelValueRange_0_25		(GRAVITY_EARTH_1000/4)	/* Level conditions about 0.2500g	,ps: |X| < 64(LSB) && |Y| < 64(LSB)*/
#define LevelValueRange_0_5			(GRAVITY_EARTH_1000/2)	/* Level conditions about 0.5000g	,ps: |X| < 128(LSB) && |Y| < 128(LSB)*/
#define LevelValueRange_2_0			(GRAVITY_EARTH_1000*2)	/* Level conditions about 2.0000g	*/
#define	GRAVITY_EARTH_1000	1024*981		/*	Magnification 1G = 1024*981*/
#define ABSMAX	(GRAVITY_EARTH_1000 * 16)

#define AVG_NUM 				8		/* for calibration */
#define SENSOR_DATA_SIZE 		3 
#define DEFAULT_SENSITIVITY 	1024	/* raw data sensitivity */

#define RBUFF_SIZE			12				/* Rx buffer size */
#define INPUT_NAME_ACC		"accelerometer"	/* Input Device Name  */

#if(defined(CONFIG_GS_GMA302) || defined(CONFIG_GS_GMA302_MODULE))
#define GSENSOR_ID			"gma302"    /* Device name for GMA302 misc. device */
#else
#define GSENSOR_ID			"gma303"    /* Device name for GMA303 misc. device */
#endif

#define SENSOR_I2C_ADDR		0x18
/* Registers */
#define GMA1302_REG_PID 	0x00
#define GMA1302_REG_PD 		0x01
#define GMA1302_REG_ACTR 	0x02
#define GMA1302_REG_MTHR 	0x03
#define GMA1302_REG_STADR 	0x04
#define GMA1302_REG_STATUS 	0x05
#define GMA1302_REG_DX	 	0x06
#define GMA1302_REG_INTCR 	0x15
#define GMA1302_REG_CONTR1 	0x16
#define GMA1302_REG_CONTR2 	0x17
#define GMA1302_REG_CONTR3 	0x18
#define GMA1302_REG_OSM	 	0x38

#define GMA1302_MODE_RESET				0x02
#define GMA1302_MODE_POWERDOWN			0x05

#define GMA302_VAL_WMI					0x02
#define GMA303_VAL_WMI					0x03
#define GMA303_VAL_WMI_RD				0x33
#define GMA30x_VAL_WMI					0x55
#define GMA1302_VAL_OFFSET_TC_ON		0x40
#define GMA1302_VAL_DATA_READY_ON		0x2a
#define GMA1302_VAL_OFF					0x00
#define GMA1302_VAL_LPF_ON				0x09 /* low-pass filter on*/
#define GMA1302_VAL_HPF_ON				0x1b /* high-pass filter on*/
#define GMA1302_VAL_TRESHOLD_MAX		0x1F /* treshold set to max*/
#define GMA1302_VAL_LOW_NOISE			0x5F /* Oversampling low noise*/
#define GMA1302_VAL_ACTR_RESET			0x00 /* Reset DSP and AFE */
#define GMA1302_VAL_ACTR_STOP			0x01 /* Stop DSP*/
#define GMA1302_VAL_ACTR_CONTINUOUS		0x02 /* Enter continuous mode */
#define GMA1302_VAL_ACTR_NON_CONTINUOUS	0x04 /* Enter non-continuous mode */

#define MMAIO	0xA1
/* IOCTLs for gsensor library */
#define ECS_IOCTL_INIT		_IO(MMAIO, 0x01)
#define ECS_IOCTL_RESET		_IO(MMAIO, 0x04)
#define ECS_IOCTL_CLOSE		_IO(MMAIO, 0x02)
#define ECS_IOCTL_START		_IO(MMAIO, 0x03)
#define ECS_IOCTL_GETDATA	_IOR(MMAIO, 0x08, char[RBUFF_SIZE+1])
#define SENSOR_CALIBRATION	_IOWR(MMAIO, 0x05 , int[SENSOR_DATA_SIZE])

/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_RATE	_IOW(MMAIO, 0x10, char)

/*status*/
#define GMA301_OPEN				1
#define GMA301_CLOSE			0
#define GMA301_NORMAL			2
#define GMA301_LOWPOWER			3

#define SENSOR_DATA_SIZE		3
#define GMA301_SENSOR_RATE_1	0
#define GMA301_SENSOR_RATE_2	1
#define GMA301_SENSOR_RATE_3	2
#define GMA301_SENSOR_RATE_4	3

#define POWER_OR_RATE			1
#define SW_RESET				1
#define GMA301_INTERRUPUT		1
#define GMA301_POWERDOWN		0 
#define GMA301_POWERON			1 

#define GRAVITY_ON_Z_NEGATIVE	1
#define GRAVITY_ON_Z_POSITIVE	2
#define GRAVITY_ON_Y_NEGATIVE	3
#define GRAVITY_ON_Y_POSITIVE	4
#define GRAVITY_ON_X_NEGATIVE	5
#define GRAVITY_ON_X_POSITIVE	6
#define GRAVITY_ON_X_AUTO		7
#define GRAVITY_ON_Y_AUTO		8
#define GRAVITY_ON_Z_AUTO		9

#define ABS(a) ((a) < 0 ? -(a) : (a))
#endif

