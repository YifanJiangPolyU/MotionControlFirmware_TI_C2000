/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement the current controller class
*/

#include "CurrentLoopController.h"

CurrentLoopController::CurrentLoopController(ControlProcessData * ControlProcessDataPtr):
  _Kp(10.32),
  _Ki(0),
  _Setpoint_Ia(0),
  _Setpoint_Ib(0),
  _Error_Ia(0),
  _Error_Ib(0),
  _Integral_Ia(0),
  _Integral_Ib(0),
  _Output_Ua(0),
  _Output_Ub(0),
  _Output_Uc(0),
  _OutputOffset(0)
{
  _ControlProcessData = ControlProcessDataPtr;
}

/**
 *  execute the current controller
 */
#pragma CODE_SECTION(".TI.ramfunc");
PwmDutyVec CurrentLoopController::Execute(void){

  PwmDutyVec Pwm;

  float32_t VoltageDcLine = _ControlProcessData->_VoltageValueDcLine;
  float32_t VoltToPwmScaleFactor = PWM_MAX_DUTY / VoltageDcLine;
  float32_t OutputVoltageLimit = VoltageDcLine * 0.85;
  float32_t OutputVoltageMinimum = VoltageDcLine * 0.05;

  // execute PI controller
  _Error_Ia = _ControlProcessData->_CurrentSetpointA;
  _Error_Ib = _ControlProcessData->_CurrentSetpointB;
  _Error_Ia -= _ControlProcessData->_CurrentSenseGain*(_ControlProcessData->_CurrentValueA);
  _Error_Ib -= _ControlProcessData->_CurrentSenseGain*(_ControlProcessData->_CurrentValueB);
  _Integral_Ia += _Ki * _Error_Ia;
  _Integral_Ib += _Ki * _Error_Ib;
  _Output_Ua = _Kp * _Error_Ia + _Integral_Ia;
  _Output_Ub = _Kp * _Error_Ib + _Integral_Ib;

  // anti-windup and output voltage limiting, phase A
  if(_Output_Ua > OutputVoltageLimit){
    _Integral_Ia -= _Output_Ua - OutputVoltageLimit;
    _Output_Ua = OutputVoltageLimit;
  } else if(_Output_Ua < -OutputVoltageLimit) {
    _Integral_Ia -= _Output_Ua + OutputVoltageLimit;
    _Output_Ua = -OutputVoltageLimit;
  }

  // anti-windup and output voltage limiting, phase B
  if(_Output_Ub > OutputVoltageLimit){
    _Integral_Ib -= _Output_Ub - OutputVoltageLimit;
    _Output_Ub = OutputVoltageLimit;
  } else if(_Output_Ub < -OutputVoltageLimit) {
    _Integral_Ib -= _Output_Ub + OutputVoltageLimit;
    _Output_Ub = -OutputVoltageLimit;
  }

  _Output_Uc = -_Output_Ua -_Output_Ub;

  // calculate neutral point voltage
  if(_Output_Ua<_Output_Ub){
    if(_Output_Ua<_Output_Uc){
      _OutputOffset = -_Output_Ua;
    }
  }else if(_Output_Ub<_Output_Uc){
    _OutputOffset = -_Output_Ub;
  }else{
    _OutputOffset = -_Output_Uc;
  }

  // offset voltage demand by neutral point voltage
  // ensure positive PWM duty
  _OutputOffset += OutputVoltageMinimum;
  _Output_Ua += _OutputOffset;
  _Output_Ub += _OutputOffset;
  _Output_Uc += _OutputOffset;

  // calculate PWM duty
  Pwm.A = (uint16_t)(_Output_Ua * VoltToPwmScaleFactor);
  Pwm.B = (uint16_t)(_Output_Ub * VoltToPwmScaleFactor);
  Pwm.C = (uint16_t)(_Output_Uc * VoltToPwmScaleFactor);

  return Pwm;

}

void CurrentLoopController::Reset(void){
  _Integral_Ia = 0;
  _Integral_Ib = 0;
}

void CurrentLoopController::AccessCurrentLoopGains_Kp(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _Kp = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _Kp;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopController::AccessCurrentLoopGains_Ki(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _Ki = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _Ki;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopController::AccessCurrentLimits_Peak(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _CurrentLimitPeakValue = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CurrentLimitPeakValue;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CurrentLoopController::AccessCurrentLimits_RMS(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _CurrentLimitRmsValue = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CurrentLimitRmsValue;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
