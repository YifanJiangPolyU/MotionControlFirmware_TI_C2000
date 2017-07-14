/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* current loop sweep sine class
*
* Control the execution of current loop sweep sine, to obtain the frequency
* domain response of the current loop. This is the sweep sine process
* controller and setpoint generator only, current loop is executed by CLA.
*/

#include "CurrentLoopSweepSine.h"
#include "math.h"

const uint16_t _CurrentControlFrequency = 32000;
const float32_t _TimeBase = 1.f/_CurrentControlFrequency;

/**
 *  Execute the current loop sweepsine process
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CurrentLoopSweepSine::Execute(void){

  float32_t Amplitude = 0;

  if(_TimeStamp < _TimeMax){
    Amplitude = GenerateSweepSine();
    _TimeStamp += 1;
  } else {

  }
}

/**
 *  Generate the sweep sine Signal
 *  @param   None
 *  @retval  Next point in the sweepsine waveform
 */
#pragma CODE_SECTION(".TI.ramfunc");
float32_t CurrentLoopSweepSine::GenerateSweepSine(void){

  float32_t Time = _TimeStamp * _TimeBase;
  float32_t Angle = _StartFreq*Time + _HalfRampRate*Time*Time;
  float32_t Amplitude = _ExcitationAmplitude * sin(Angle);

  return Amplitude;
}

/**
 *  Reset the current loop sweepsine process
 */
void CurrentLoopSweepSine::Reset(void){

  _CurrentLoopController->Reset();
  _TimeStamp = 0;
  _HalfRampRate = _RampRate * 0.5;

  if(_StartFreq<_EndFreq){
    _TimeMax = (uint16_t)((_EndFreq - _StartFreq)/_RampRate/_TimeBase);
  } else {
    _TimeMax = 0;
  }

}

void CurrentLoopSweepSine::AccessExcitationAmplitude(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _ExcitationAmplitude = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _ExcitationAmplitude;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopSweepSine::AccessExcitationLength(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _TimeMax = handle->Data.DataInt16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt16[0] = _TimeMax;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopSweepSine::AccessStartFrequency(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _StartFreq = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _StartFreq;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopSweepSine::AccessEndFrequency(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _EndFreq = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _EndFreq;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopSweepSine::AccessRampRate(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _RampRate = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _RampRate;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
