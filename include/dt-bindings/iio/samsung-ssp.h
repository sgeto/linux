// SPDX-License-Identifier: GPL-2.0

#ifndef _DT_BINDINGS_SAMSUNG_SSP_H
#define _DT_BINDINGS_SAMSUNG_SSP_H

/* 
 * Sensor position based on alterations to x/y/z coordinate data
 * that needs to be done. Physical position on the board
 * will depend on the sensor in question.
 */
#define SENSOR_POS_PXPYPZ	0
#define SENSOR_POS_NXPYPZ	1
#define SENSOR_POS_NXNYPZ	2
#define SENSOR_POS_PXNYPZ	3
#define SENSOR_POS_NXPYNZ	4
#define SENSOR_POS_PXPYNZ	5
#define SENSOR_POS_PXNYNZ	6
#define SENSOR_POS_NXNYNZ	7

#endif
