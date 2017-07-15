/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement encoder driver functions
* in the case of TI C28x, reading EQEP modules
*/

#ifndef _ENCODER_DRIVER_H
#define _ENCODER_DRIVER_H

#include "F28x_Project.h"

#ifdef __cplusplus
  extern "C" {
#endif

int32_t GetEncoder1Position(void);
int32_t GetEncoder2Position(void);

#ifdef __cplusplus
  }
#endif

#endif
