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

#include "SysDef.h"
#include "PdoDataTypeDef.h"
#include "ControlTypeDef.h"
#include "ObjectDictionaryEntryBase.h"
#include "Drivers/PowerStageControl/PowerStageControl.h"

// ADC phase current sense gain (mA/LSB)
#define CURREN_SENSE_GAIN_PHASE     7.32421875f

// ADC DC line current sense gain (mA/LSB)
#define CURREN_SENSE_GAIN_DCLINE    7.32421875f

// ADC DC line voltage sense gain (V/LSB)
#define VOLTAGE_SENSE_GAIN_DCLINE    0.0058520361035f

class ControlProcessData: public ObjectDictionaryEntryBase{

public:

  ControlProcessData():
    _SyncFlag(0),
    _OperationEnabled(false),
    _ControlProcess(PROCESS_NONE),
    _MotorType(MTR_TYPE_NONE),
    _DcLineVoltageUpperLimit(PWR_MAX_DCLINE_VOLTAGE),
    _DcLineVoltageLowerLimit(PWR_MIN_DCLINE_VOLTAGE),
    _DcLineCurrentLimit(PWR_MAX_DCLINE_CURRENT),
    _MotorCurrentLimitRMS(400),
    _MotorCurrentLimitPEAK(400),
    _CommAngleCosine(1),
    _CommAngleSine(0),
    _ForceSetpoint(0),
    _PositionSetpoint(0),
    _VelocitySetpoint(0),
    _AccelSetpoint(0),
    _PdoID(PDO_ID_DEBUG)
  {
    _DQCurrentSetpoint.D = 0;
    _DQCurrentSetpoint.Q = 0;
    _PhaseCurrentSetpoint.A = 0;
    _PhaseCurrentSetpoint.B = 0;

    _MotorCurrentLimitPEAKSquared = (float32_t)_MotorCurrentLimitPEAK;
    _MotorCurrentLimitPEAKSquared /= 10.f;
    _MotorCurrentLimitPEAKSquared *= _MotorCurrentLimitPEAKSquared;

    _MotorCurrentLimitRMSSquared = (float32_t)_MotorCurrentLimitRMS;
    _MotorCurrentLimitRMSSquared /= 10.0f;
    _MotorCurrentLimitRMSSquared *= _MotorCurrentLimitRMSSquared;

    InitCLAGains();
  };

  ~ControlProcessData(){};

  /* Access functions for internal use */
  void UpdateMeasurements(void);
  void SetCurrentSweepSineBuffer(int16_t data);
  void ClearCurrentSweepSineBuffer(void);

  /* object dictionary access functions */
  void AccessParameter(ObdAccessHandle * handle);
  void AccessMotorType(ObdAccessHandle * handle);
  void AccessControlProcess(ObdAccessHandle * handle);
  void AccessDcLineVoltage(ObdAccessHandle * handle);
  void AccessDcLineCurrent(ObdAccessHandle * handle);
  void AccessCpuTemperature(ObdAccessHandle * handle);
  void AccessPowerStageTemperature(ObdAccessHandle * handle);

  void AccessDcLineVoltageUpperLimit(ObdAccessHandle * handle);
  void AccessDcLineVoltageLowerLimit(ObdAccessHandle * handle);
  void AccessDcLineCurrentLimit(ObdAccessHandle * handle);
  void AccessMotorCurrentLimitRMS(ObdAccessHandle * handle);
  void AccessMotorCurrentLimitPEAK(ObdAccessHandle * handle);

  void AccessCommutationAngle(ObdAccessHandle * handle);
  void AccessCommutationAngle_Cos(ObdAccessHandle * handle);
  void AccessCommutationAngle_Sin(ObdAccessHandle * handle);

  void AccessDQCurrentSetpoint_D(ObdAccessHandle * handle);
  void AccessDQCurrentSetpoint_Q(ObdAccessHandle * handle);


  uint16_t _SyncFlag;
  bool     _OperationEnabled;

  uint16_t _ControlProcess;       // control process used for operation
  uint16_t _MotorType;

  // current actual values
  float32_t * _CurrentValueBufferPhaseA;
  float32_t * _CurrentValueBufferPhaseB;

  // current actual value in mA
  PhaseCurrentVec _CurrentActualValue;
  AlBeVec _StatorCurrent;

  // power supply status actual value
  uint32_t _VoltageValueDcLine;
  uint16_t _CurrentValueDcLine;

  // current and voltage limits
  uint16_t _DcLineVoltageUpperLimit;
  uint16_t _DcLineVoltageLowerLimit;
  uint16_t _DcLineCurrentLimit;
  uint16_t _MotorCurrentLimitRMS;
  uint16_t _MotorCurrentLimitPEAK;

  float32_t _MotorCurrentLimitPEAKSquared;
  float32_t _MotorCurrentLimitRMSSquared;

  // position
  int32_t _Position;            // unit: encoder count
  float32_t _Velocity;          // unit: count/sp
  float32_t _Acceleration;      // unit: count/sp^2

  // gains
  float32_t _CurrentSenseGain_Phase;   // unit: mA/LSB
  float32_t _CurrentSenseGain_DcLine;   // unit: mA/LSB
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
  float32_t _CommAngle;
  float32_t _CommAngleCosine;
  float32_t _CommAngleSine;

  // demand values
  int32_t _PositionSetpoint;
  float32_t _VelocitySetpoint;
  float32_t _AccelSetpoint;
  float32_t _ForceSetpoint;
  DQVec _DQCurrentSetpoint;                   // unit: mA
  PhaseCurrentVec _PhaseCurrentSetpoint;      // unit: mA

  // position control limit values
  float32_t _VelocityLimit;           // unit: cnt/sp
  float32_t _AccelerationLimit;       // unit: cnt/sp^2

  // buffer to store current loop sweepsine data
  int16_t _CurrentSweepSineBuffer[4];

  // current ID of PDO
  char _PdoID;

private:
  void InitCLAGains(void);


};

#endif
