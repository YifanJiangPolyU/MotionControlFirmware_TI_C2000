/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define power stage control functions
*/

#ifndef _PWR_STAGE_CTRL_H
#define _PWR_STAGE_CTRL_H

#include "F28x_Project.h"

void SensorEnable(void);
void PwmTimerEnable(void);
void PwrEnable(void);
void PwrDisable(void);
void PwrReset(void);

#endif
