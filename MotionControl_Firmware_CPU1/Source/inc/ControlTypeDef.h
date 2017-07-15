/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define some convenient types
*/

#ifndef _CONTROL_TYPEDEF_H
#define _CONTROL_TYPEDEF_H

#include "F28x_Project.h"
#include "stdint.h"

#ifdef __cplusplus
  extern "C" {
#endif

typedef struct DQCurrentTypedef{
  float32_t D;
  float32_t Q;
} DQCurrentVec;

typedef struct AlBeCurrentTypedef{
  float32_t Alpha;
  float32_t Beta;
} AlBeCurrentVec;

typedef struct PhaseCurrentTypedef{
  float32_t A;
  float32_t B;
} PhaseCurrentVec;

typedef struct PhaseCVoltageTypedef{
  float32_t A;
  float32_t B;
  float32_t C;
} PhaseVoltageVec;

typedef struct PwmDutyTypedef{
  uint16_t A;
  uint16_t B;
  uint16_t C;
} PwmDutyVec;


#ifdef __cplusplus
  }
#endif

#endif
