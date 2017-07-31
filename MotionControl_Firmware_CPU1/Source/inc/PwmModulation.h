/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement pwm modulation functions
*/

#ifndef PWM_MODULATION_H
#define PWM_MODULATION_H

#include "ControlTypeDef.h"
#include "Drivers/PowerStageControl/PowerStageControl.h"

#ifdef __cplusplus
  extern "C" {
#endif

/**
 *  calculate PWM timer counter value based on requried voltage and scaling factor
 *  @param voltage                required voltage
 *  @param Pwm                    stores the computed PWM timer counter value
 *  @param VoltToPwmScaleFactor   PWM resolution: cnt / volt
 */
inline void PwmModulation(ABCVec* voltage, PwmDutyVec* Pwm, float32_t VoltToPwmScaleFactor){
  float32_t OutputOffset = 0;

  // calculate neutral point voltage offset
  if(voltage->A<voltage->B){
    if(voltage->A<voltage->C){
      OutputOffset = -voltage->A;
    }
  }else if(voltage->B<voltage->C){
    OutputOffset = -voltage->B;
  }else{
    OutputOffset = -voltage->C;
  }

  // offset voltage demand by neutral point voltage
  // ensure positive PWM duty
  voltage->A += OutputOffset;
  voltage->B += OutputOffset;
  voltage->C += OutputOffset;

  // calculate PWM duty
  Pwm->A = (uint16_t)(voltage->A * VoltToPwmScaleFactor) + PWM_MIN_DUTY;
  Pwm->B = (uint16_t)(voltage->B * VoltToPwmScaleFactor) + PWM_MIN_DUTY;
  Pwm->C = (uint16_t)(voltage->C * VoltToPwmScaleFactor) + PWM_MIN_DUTY;

  // limiting max duty cycle
  if(Pwm->A>PWM_MAX_DUTY){
    Pwm->A = PWM_MAX_DUTY;
  }

  if(Pwm->B>PWM_MAX_DUTY){
    Pwm->B = PWM_MAX_DUTY;
  }

  if(Pwm->C>PWM_MAX_DUTY){
    Pwm->C = PWM_MAX_DUTY;
  }

}

#ifdef __cplusplus
  }
#endif

#endif
