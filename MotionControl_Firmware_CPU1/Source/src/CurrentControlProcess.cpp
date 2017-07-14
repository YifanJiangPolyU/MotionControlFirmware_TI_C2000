/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement the current control process class
*/

#include "CurrentControlProcess.h"
#include "Drivers/PowerStageControl/PowerStageControl.h"

void CurrentControlProcess::Execute(void){
  PwmDutyVec PwmDutyCycle;

  // execute current controller
  PwmDutyCycle = _CurrentLoopController->Execute(&(_ControlProcessData->_CurrentSetpoint),
                                                 &(_ControlProcessData->_CurrentActualValue));

  // apply duty cycle
  PwrSetPwmDuty(&PwmDutyCycle);
}
