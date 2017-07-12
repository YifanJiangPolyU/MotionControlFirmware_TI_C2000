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

/**
 *  Execute the current loop sweepsine process
 */
void CurrentLoopSweepSine::Execute(void){

}

/**
 *  Generate the sweep sine Signal
 *  @param   None
 *  @retval  Next point in the sweepsine waveform
 */
float32_t CurrentLoopSweepSine::SignalGeneration(void){
  return 0.0f;
}

/**
 *  Reset the current loop sweepsine process
 */
void CurrentLoopSweepSine::Reset(void){
  _CurrentLoopController->Reset();
  _TimeCounter = 0;
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
      _ExcitationLength = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _ExcitationLength;
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

void CurrentLoopSweepSine::AccessSweepRate(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _SweepRate = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _SweepRate;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
