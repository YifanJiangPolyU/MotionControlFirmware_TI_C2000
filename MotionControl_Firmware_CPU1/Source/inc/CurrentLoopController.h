/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define the current controller class
*/

#ifndef _CURRENT_LOOP_CONTROLLER_H
#define _CURRENT_LOOP_CONTROLLER_H

#include "stdint.h"
#include "CiATypeDef.h"
#include "ControlProcessData.h"
#include "ControlProcessBase.h"
#include "ControlTypeDef.h"
#include "Drivers/PowerStageControl/PowerStageControl.h"
#include "Drivers/GpioDriver/GpioDriver.h"

class CurrentLoopController: public ObjectDictionaryEntryBase{

  public:
    CurrentLoopController(ControlProcessData * ControlProcessDataPtr);
    ~CurrentLoopController(){}

    PwmDutyVec Execute(PhaseCurrentVec * Demand, PhaseCurrentVec * Actual);
    void Reset(void);

    void AccessCurrentControlFrequency(ObdAccessHandle * handle);
    void AccessCurrentLoopGains_Kp(ObdAccessHandle * handle);
    void AccessCurrentLoopGains_Ki(ObdAccessHandle * handle);

    void AccessCurrentLimits_Peak(ObdAccessHandle * handle);
    void AccessCurrentLimits_RMS(ObdAccessHandle * handle);


  private:

    ControlProcessData * _ControlProcessData;

    float32_t _Kp;                     // P gain
    float32_t _Ki;                     // I gain
    float32_t _Setpoint_Ia;            // current requirement (ADC raw), phase A
    float32_t _Setpoint_Ib;            // current requirement (ADC raw), phase B

    float32_t _Error_Ia;               // current error, phase A
    float32_t _Error_Ib;               // current error, phase B
    float32_t _Integral_Ia;            // integral term, phase A
    float32_t _Integral_Ib;            // integral term, phase B
    float32_t _OutputOffset;            // offset required to give positive PWM duty
    ABCVec _OutputVoltage;             // output phase voltage 


    // current control limit values
    float32_t _CurrentLimitPeakValue;   // unit: mA
    float32_t _CurrentLimitRmsValue;

};

#endif
