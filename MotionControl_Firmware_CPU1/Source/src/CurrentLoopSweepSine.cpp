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
#include "PdoDataTypeDef.h"
#include "DataTypeHelper.h"
#include "math.h"

const uint16_t _CurrentControlFrequency = 32000;
const float32_t _TimeBase = 1.f/_CurrentControlFrequency;

/**
 *  Execute the current loop sweepsine process
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CurrentLoopSweepSine::Execute(void){

  PhaseCurrentVec CurrenDemand;
  PhaseCurrentVec CurrenActual = _ControlProcessData->_CurrentActualValue;

  PwmDutyVec Pwm;
  Pwm.A = 0;
  Pwm.B = 0;
  Pwm.C = 0;

  CurrenDemand.A = 0;
  CurrenDemand.B = 0;

  switch (_State) {
    case STATE_WAIT_SYNC:
      // synchronize sweepsine generation to ControlProcessMaster
      if(_ControlProcessData->_SyncFlag==0){
        _OldPdoID = _ControlProcessData->_PdoID;
        _ControlProcessData->_PdoID = PDO_ID_CLSW;
        _State = STATE_RUNNING;
      }
      break;
    case STATE_RUNNING:
      if(_TimeStamp < _TimeMax){
        if(_ActivePhase=='A'){
          CurrenDemand.A = GenerateSweepSine();
          _ControlProcessData->SetCurrentSweepSineBuffer((int16_t)CurrenDemand.A);
        } else {
          CurrenDemand.B = GenerateSweepSine();
          _ControlProcessData->SetCurrentSweepSineBuffer((int16_t)CurrenDemand.B);
        }

        Pwm = _CurrentLoopController->Execute(&CurrenDemand, &CurrenActual);
        PwrSetPwmDuty(&Pwm);
        _TimeStamp += 1;


      } else {
        if(_ControlProcessData->_SyncFlag==0){
          _State = STATE_END;
        }
      }
      break;
    case STATE_END:
      _ControlProcessData->_PdoID = _OldPdoID;
      _ProcessShouldQuit = true;
      break;
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
 *  calculate sweep sine data length based on start freq, end freq, and
 *  ramp rate
 */
void CurrentLoopSweepSine::CalculateSweepSineParameters(void){
  _HalfRampRate = _RampRate * 0.5;

  if(_StartFreq<_EndFreq){
    _TimeMax = (uint16_t)((_EndFreq - _StartFreq)/_RampRate/_TimeBase);

    // calculate number of PDO messages required to transmit the sweep sine data
    // should be 1 plus the smallest integer larger than _TimeMax/4
    // the additional 1 msg account for the empty period required to
    // sync sweep sine to ControlProcessMaster
    _NumberOfPkg = _TimeMax/4;
    if(_NumberOfPkg*4 < _TimeMax){
      _NumberOfPkg += 2;
    } else {
      _NumberOfPkg += 1;
    }

  } else {
    _NumberOfPkg = 0xFFFF;
    _TimeMax = 0;
  }

}

/**
 *  Reset the current loop sweepsine process
 */
void CurrentLoopSweepSine::Reset(void){

  _State = STATE_WAIT_SYNC;
  _CurrentLoopController->Reset();
  _TimeStamp = 0;

  CalculateSweepSineParameters();

  _ControlProcessData->ClearCurrentSweepSineBuffer();
  _ProcessShouldQuit = false;
}

void CurrentLoopSweepSine::AccessExcitationAmplitude(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      if(handle->Data.DataFloat32>=0){
        _ExcitationAmplitude = handle->Data.DataFloat32;
        handle->AccessResult = OBD_ACCESS_SUCCESS;
      } else {
        handle->AccessResult = OBD_ACCESS_ERR_DATA_RANGE;
      }
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _ExcitationAmplitude;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopSweepSine::AccessDataLength(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt16[0] = _NumberOfPkg;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopSweepSine::AccessStartFrequency(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      if(handle->Data.DataFloat32>=0){
        _StartFreq = handle->Data.DataFloat32;
        handle->AccessResult = OBD_ACCESS_SUCCESS;
        CalculateSweepSineParameters();
      } else {
        handle->AccessResult = OBD_ACCESS_ERR_DATA_RANGE;
      }
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
      if(handle->Data.DataFloat32>=0){
        _EndFreq = handle->Data.DataFloat32;
        handle->AccessResult = OBD_ACCESS_SUCCESS;
        CalculateSweepSineParameters();
      } else {
        handle->AccessResult = OBD_ACCESS_ERR_DATA_RANGE;
      }
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
      if(handle->Data.DataFloat32>=0){
        _RampRate = handle->Data.DataFloat32;
        handle->AccessResult = OBD_ACCESS_SUCCESS;
        CalculateSweepSineParameters();
      } else {
        handle->AccessResult = OBD_ACCESS_ERR_DATA_RANGE;
      }
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _RampRate;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopSweepSine::AccessActivePhase(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      if(__byte_uint16_t(handle->Data.DataUint16[0], 0)=='A'){
        _ActivePhase = 'A';
        handle->AccessResult = OBD_ACCESS_SUCCESS;
      } else if(__byte_uint16_t(handle->Data.DataUint16[0], 0)=='B') {
        _ActivePhase = 'B';
        handle->AccessResult = OBD_ACCESS_SUCCESS;
      } else {
        handle->AccessResult = OBD_ACCESS_ERR_DATA_RANGE;
      }
      break;
    case SDO_CSS_READ:
      __byte_uint16_t(handle->Data.DataUint16[0], 0) = _ActivePhase;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
