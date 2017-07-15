/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement encoder driver functions
* in the case of TI C28x, reading EQEP modules
*/

#include "EncoderDriver.h"

/**
 *   read first encoder
 *   @retval     encoder count
 */
int32_t GetEncoder1Position(void){
  return EQep1Regs.QPOSCNT;
}

/**
 *   read second encoder
 *   @retval     encoder count
 */
int32_t GetEncoder2Position(void){
  return EQep2Regs.QPOSCNT;
}
