/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement the calibration process class
*/

#include "CalibrationProcess.h"
#include "CPU1_CLA1_common.h"
#include "Drivers/GpioDriver/GpioDriver.h"

float32_t calibration_debug[40];
CalibrationProcess::CalibrationProcess(ControlProcessData * ControlProcessDataPtr):
  _ControlProcessData(ControlProcessDataPtr),
  _State(STATE_WAIT),
  _CurrentOffsetSumA(0),
  _CurrentOffsetSumB(0),
  _CurrentOffsetSampleCnt(0),
  _CurrentOffsetCycleCnt(0)
{
  // initialize offset to middel point
  _CurrentOffset_PhaseA = -2048.0f;
  _CurrentOffset_PhaseB = -2048.0f;

  CLA_CurrentSenseOffset_PhaseA = _CurrentOffset_PhaseB;
  CLA_CurrentSenseOffset_PhaseB = _CurrentOffset_PhaseB;
}


void CalibrationProcess::Execute(void){

  PwmDutyVec Pwm;
  PhaseCurrentVec CurrenActual;

  CurrenActual = _ControlProcessData->_CurrentActualValue;
  Pwm.A = 0;
  Pwm.B = 0;
  Pwm.C = 0;

  switch (_State) {
    case STATE_WAIT:
      _WaitCounter += 1;
      if(_WaitCounter==32000){
        _State = STATE_CAL_CURRENT_OFFSET;
      }
    case STATE_CAL_CURRENT_OFFSET:
      _CurrentOffsetSumA += CurrenActual.A;
      _CurrentOffsetSumB += CurrenActual.B;
      calibration_debug[_CurrentOffsetCycleCnt] = CurrenActual.A;

      _CurrentOffsetCycleCnt += 1;
      if(_CurrentOffsetCycleCnt>=40){
        _CurrentOffset_PhaseA = _CurrentOffsetSumA / 40.f;
        _CurrentOffset_PhaseB = _CurrentOffsetSumB / 40.f;
        CLA_CurrentSenseOffset_PhaseA = _CurrentOffset_PhaseA;
        CLA_CurrentSenseOffset_PhaseB = _CurrentOffset_PhaseB;
        _State = STATE_COMPLETE;
      }
      break;
    case STATE_COMPLETE:
      ClearErrorLed();
      _ProcessShouldQuit = true;
      break;
    default:
      break;
  }

  PwrSetPwmDuty(&Pwm);
}


void CalibrationProcess::Reset(void){

  PwmDutyVec Pwm;
  Pwm.A = 0;
  Pwm.B = 0;
  Pwm.C = 0;

  _CurrentOffsetSumA = 0;
  _CurrentOffsetSumB = 0;
  _CurrentOffsetSampleCnt = 0;
  _CurrentOffsetCycleCnt = 0;

  _WaitCounter = 0;

  _State = STATE_WAIT;

  SetErrorLed();

  PwrSetPwmDuty(&Pwm);
}


void CalibrationProcess::AccessCurrentOffsetPhaseA(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CurrentOffset_PhaseA;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CalibrationProcess::AccessCurrentOffsetPhaseB(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CurrentOffset_PhaseB;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
