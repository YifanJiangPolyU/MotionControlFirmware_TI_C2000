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
  _Kp(0),
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
void CurrentLoopController::Execute(void){

  // execute PI controller
  _Error_Ia = _Setpoint_Ia;
  _Error_Ib = _Setpoint_Ib;
  _Integral_Ia += _Ki * _Error_Ia;
  _Integral_Ib += _Ki * _Error_Ib;
  _Output_Ua = _Kp * _Error_Ia + _Integral_Ia;
  _Output_Ub = _Kp * _Error_Ib + _Integral_Ib;

  // output voltage limiting
  if(_Output_Ua > _OutputLimit){
    _Integral_Ia -= _Output_Ua - _OutputLimit;
    _Output_Ua = _OutputLimit;
  } else if(_Output_Ua < -_OutputLimit) {
    _Integral_Ia -= _Output_Ua + _OutputLimit;
    _Output_Ua = -_OutputLimit;
  }

  // anti-windup
  if(_Output_Ub > _OutputLimit){
    _Integral_Ib -= _Output_Ub - _OutputLimit;
    _Output_Ub = _OutputLimit;
  } else if(_Output_Ub < -_OutputLimit) {
    _Integral_Ib -= _Output_Ub + _OutputLimit;
    _Output_Ub = -_OutputLimit;
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
  _Output_Ua += _OutputOffset;
  _Output_Ub += _OutputOffset;
  _Output_Uc += _OutputOffset;

  // calculate PWM duty
  /*
  _OutputPWM_Ua = (uint16_t)(3125 * _Output_Ua / (VoltageSenseGain*dcVoltageSense));
  _OutputPWM_Ub = (uint16_t)(3125 * _Output_Ub / (VoltageSenseGain*dcVoltageSense));
  _OutputPWM_Uc = (uint16_t)(3125 * _Output_Uc / (VoltageSenseGain*dcVoltageSense));
  */

}

void CurrentLoopController::Reset(void){
  _Integral_Ia = 0;
  _Integral_Ib = 0;
}
