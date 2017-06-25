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
    _CurrentSenseGain(0),
    _VoltageSenseGain(0),
    _CommAngleCosine(1),
    _CommAngleSine(0),
    _ForceSetpoint(0),
    _CurrentSetpointA(0),
    _CurrentSetpointB(0),
    _PositionSetpoint(0),
    _VelocitySetpoint(0),
    _AccelSetpoint(0)
  {

  };

  ~ControlProcessData(){};

  // current actual values
  uint16_t * _CurrentValueBufferPhaseA;
  uint16_t * _CurrentValueBufferPhaseB;
  uint16_t _CurrentValuePhaseA[4];
  uint16_t _CurrentValuePhaseB[4];
  uint16_t _CurrentValueDcLine;

  // current value in mA
  float32_t _CurrentValueA;
  float32_t _CurrentValueB;

  // voltage actual value
  float32_t _VoltageDcLine;

  // position
  int32_t _Position;            // unit: encoder count
  float32_t _Velocity;          // unit: count/sp
  float32_t _Acceleration;      // unit: count/sp^2

  // gains
  float32_t _CurrentSenseGain;         // unit: mA/LSB
  float32_t _VoltageSenseGain;         // unit: V/LSB
  float32_t _TemperatureSenseGain1;    // unit: K/LSB
  float32_t _TemperatureSenseGain2;    // unit: K/LSB
  float32_t _ForceGain;                // unit: (count/sp^2)/mA

  // offsets
  uint16_t _CurrentSenseOffsetA;        // unit: LSB
  uint16_t _CurrentSenseOffsetB;
  uint16_t _TemperatureSenseOffset1;
  uint16_t _TemperatureSenseOffset2;

  // unit conversion constants
  float32_t _Factor_PositionToSI;       // encoder cnt to SI unit (rad or mm)
  float32_t _Factor_VelocityToSI;       // cnt/sp to SI unit (rad/s or mm/s)
  float32_t _Factor_AccelerationToSI;   // cnt/sp to SI unit (rad/s^2 or mm/s^2)

  // commutation angles
  float32_t _CommAngleCosine;
  float32_t _CommAngleSine;

  // demand values
  int32_t _PositionSetpoint;
  float32_t _VelocitySetpoint;
  float32_t _AccelSetpoint;
  float32_t _ForceSetpoint;
  float32_t _CurrentSetpointA;      // unit: mA
  float32_t _CurrentSetpointB;



  // position control limit values
  float32_t _VelocityLimit;           // unit: cnt/sp
  float32_t _AccelerationLimit;       // unit: cnt/sp^2


};

#endif
