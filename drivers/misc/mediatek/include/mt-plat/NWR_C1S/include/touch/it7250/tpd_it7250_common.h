/*
 * Copyright (C) 2010 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_WAKEUP_TRIAL	60
#define TPD_WAKEUP_DELAY	100

#define TPD_DELAY		(2*HZ/100)

#define TPD_RES_X		400
#define TPD_RES_Y		400

#define TPD_UP_DOWN_SWAP 		1
#define TPD_LEFT_RIGHT_SWAP 	1
//#define TPD_X_Y_SWAP			1

#define IT7252_NO_POWER_OFF_IN_SLEEP	1
typedef void (*GES_CBFUNC)(u8);
/*****************************************************************************
 * ENUM
 ****************************************************************************/


/*****************************************************************************
 * STRUCTURE
 ****************************************************************************/


extern struct tpd_device *tpd;
extern unsigned int tpd_rst_gpio_number;
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);

#endif /* TOUCHPANEL_H__ */
