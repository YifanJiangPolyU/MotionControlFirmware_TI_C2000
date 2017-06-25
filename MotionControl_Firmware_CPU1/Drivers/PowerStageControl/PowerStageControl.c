/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement power stage control functions
*/


#include "PowerStageControl.h"
#include "Drivers/GpioDriver/GpioDriver.h"

/**
 *  enable current, voltage, and temperature sampling
 */
void SensorEnable(void){
  // enable EPWM1 triggering ADC
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;
}

/**
  * enable PWM timers
  */
void PwmTimerEnable(void){

  EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
  EPwm4Regs.TBCTL.bit.CTRMODE = 2;  //unfreeze, and enter up-down mode
  EPwm5Regs.TBCTL.bit.CTRMODE = 2;
  EPwm6Regs.TBCTL.bit.CTRMODE = 2;

  // synchronize PWM timer counters
  EPwm1Regs.TBCTL.bit.SWFSYNC = 1;
}

/**
  * enable power stage output
  */
void PwrEnable(void){
  // enable PWM output

  // set DRV gate enable
  SetDrv8301GateEnable();
}

/**
  * disable power stage output
  */
void PwrDisable(void){
  // Clear DRV gate enable
  ClearDrv8301GateEnable();
}
