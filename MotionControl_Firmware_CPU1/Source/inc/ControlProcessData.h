/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* Control process data class
*
* holds some important values to be shared among processes
*/

#ifndef CONTROL_PROCESS_DATA_H
#define CONTROL_PROCESS_DATA_H

#include "stdint.h"
#include "F28x_Project.h"

class ControlProcessData{

public:

  ControlProcessData():
    _CommAngleCosine(1),
    _CommAngleSine(0),
    _TorqueSetpoint(0),
    _CurrentSetpointA(0),
    _CurrentSetpointB(0),
    _PositionSetpoint(0),
    _VelocitySetpoint(0),
    _AccelSetpoint(0)
  {

  };

  ~ControlProcessData(){};

  // process data
  uint16_t * _CurrentValueBufferPhaseA;
  uint16_t * _CurrentValueBufferPhaseB;
  uint16_t _CurrentValuePhaseA[4];
  uint16_t _CurrentValuePhaseB[4];

  float32_t _CommAngleCosine;
  float32_t _CommAngleSine;

  float32_t _TorqueSetpoint;
  float32_t _CurrentSetpointA;
  float32_t _CurrentSetpointB;

  int32_t _PositionSetpoint;
  float32_t _VelocitySetpoint;
  float32_t _AccelSetpoint;

};

#endif
