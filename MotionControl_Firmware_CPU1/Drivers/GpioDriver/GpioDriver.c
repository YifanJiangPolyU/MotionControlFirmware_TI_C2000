/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement GPIO driver
*/

#include "GpioDriver.h"


/**
 *  toggle error LED (red, @ GPIO34)
 */
void SetErrorLed(void){
  GpioDataRegs.GPBDAT.bit.GPIO34 = 0;
}

/**
 *  clear error LED (red, @ GPIO34)
 */
void ClearErrorLed(void){
  GpioDataRegs.GPBDAT.bit.GPIO34 = 1;
}

/**
 *  toggle status LED (blue, @ GPIO31)
 */
void SetStatusLed(void){
  GpioDataRegs.GPADAT.bit.GPIO31 = 0;
}

/**
 *  clear status LED (blue, @ GPIO31)
 */
void ClearStatusLed(void){
  GpioDataRegs.GPADAT.bit.GPIO31 = 1;
}

/**
 *  Set Gate Enable pin for DRV8310 (@ GPIO26)
 */
void SetDrv8301GateEnable(void){
  GpioDataRegs.GPADAT.bit.GPIO26 = 0;
}

/**
 *  clear Gate Enable pin for DRV8310 (@ GPIO26)
 */
void ClearDrv8301GateEnable(void){
  GpioDataRegs.GPADAT.bit.GPIO26 = 1;
}
