/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* commutation master class
*
* Contains functionalities related to commutation, including: a) initial
* commutation angle detection, b) initial polarity detection, and c) real-time
* commutation with encoder and/or hall sensors
*/

#include "CommutationMaster.h"
#include "math.h"


/**
 *  compute commutation angle per count
 */
void CommutationMaster::UpdateCommResolution(void){
  CommAnglePerCount = 6.28318530717958623200 / CountPerRev / NPolePair;
}


//__attribute__((ramfunc))
void CommutationMaster::Update(int32_t PosCounter){

  PosCounter_new = PosCounter;

  CommAngle += (PosCounter_new-PosCounter_old) * CommAnglePerCount;

  // handling commutation angle warp arround
  // range limited to 0 ~ 2pi
  if(CommAngle < 0){
    CommAngle += 6.28318530717958623200f;
  }else if(CommAngle > 6.28318530717958623200f){
    CommAngle -= 6.28318530717958623200f;
  }

  // compute the sin and cos using TMU (faster)
  CommAngle_Cos = cos(CommAngle);
  CommAngle_Sin = sin(CommAngle);

  PosCounter_old = PosCounter_new;
}
