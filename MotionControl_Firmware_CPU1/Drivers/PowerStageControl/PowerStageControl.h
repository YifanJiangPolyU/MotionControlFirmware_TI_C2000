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
#include "ControlTypeDef.h"

#ifdef __cplusplus
  extern "C" {
#endif

#define PWM_COUNTER_TOP      3125
#define PWM_MAX_DUTY         3000
#define PWM_MIN_DUTY         150

void SensorEnable(void);
void PwmTimerEnable(void);
void PwrEnable(void);
void PwrDisable(void);
void PwrReset(void);

void PwrSetPwmDuty(PwmDutyVec * duty);

#ifdef __cplusplus
  }
#endif

#endif