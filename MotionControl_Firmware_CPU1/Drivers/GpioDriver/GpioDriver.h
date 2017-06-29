/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define GPIO drivers
*/

#ifndef _GPIO_DRIVRE_H
#define _GPIO_DRIVRE_H

#include "F28x_Project.h"

#ifdef __cplusplus
  extern "C" {
#endif

void SetErrorLed(void);
void ClearErrorLed(void);
void SetStatusLed(void);
void ClearStatusLed(void);
void SetDrv8301GateEnable(void);
void ClearDrv8301GateEnable(void);

#ifdef __cplusplus
  }
#endif

#endif
