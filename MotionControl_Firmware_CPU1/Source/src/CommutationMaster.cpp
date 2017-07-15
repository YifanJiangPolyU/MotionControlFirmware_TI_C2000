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

const float32_t FACTOR_2PI = 6.28318530717958623200f;
/**
 *  compute commutation angle per count
 */
void CommutationMaster::UpdateCommResolution(void){
  _CommAnglePerCount = FACTOR_2PI / _CountPerRev / _NPolePair;
}

/**
 *  set commutation angle to a new value (typically during initialization)
 *  @param angle      new value of commutation angle
 */
void CommutationMaster::SetCommAngle(float32_t angle){
  _CommAngle = angle;
  _CommAngle_Cos = cos(_CommAngle);
  _CommAngle_Sin = sin(_CommAngle);

  _ControlProcessData->_CommAngle = _CommAngle;
  _ControlProcessData->_CommAngleCosine = _CommAngle_Cos;
  _ControlProcessData->_CommAngleSine = _CommAngle_Sin;
}

/**
 *  update commutation angle using new encoder position
 *  @param PosCount      new value of encoder position
 */
void CommutationMaster::Update(int32_t PosCounter){

  UpdateRotary(PosCounter);

  _ControlProcessData->_CommAngle = _CommAngle;
  _ControlProcessData->_CommAngleCosine = _CommAngle_Cos;
  _ControlProcessData->_CommAngleSine = _CommAngle_Sin;
}

/**
 *  update commutation for a rotary motor
 *  @param PosCount      new value of encoder position
 */
void CommutationMaster::UpdateRotary(int32_t PosCounter){
  _PosCounter_new = PosCounter;

  _CommAngle += (_PosCounter_new-_PosCounter_old) * _CommAnglePerCount;

  // handling commutation angle wrap arround
  // range limited to 0 ~ 2pi
  if(_CommAngle < 0){
    _CommAngle += FACTOR_2PI;
  }else if(_CommAngle > FACTOR_2PI){
    _CommAngle -= FACTOR_2PI;
  }

  // compute the sin and cos using TMU (faster)
  _CommAngle_Cos = cos(_CommAngle);
  _CommAngle_Sin = sin(_CommAngle);

  _PosCounter_old = _PosCounter_new;
}

/**
 *  update commutation for a linear motor
 *  @param PosCount      new value of encoder position
 */
void CommutationMaster::UpdateLinear(int32_t PosCounter){
  _PosCounter_new = PosCounter;

  _PosCounter_old = _PosCounter_new;
}

void CommutationMaster::AccessNPolePair(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _NPolePair = handle->Data.DataUint16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _NPolePair;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CommutationMaster::AccessCountPerRev(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _CountPerRev = handle->Data.DataUint32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint32 = _CountPerRev;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CommutationMaster::AccessPolePitch(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _PolePitch = handle->Data.DataUint16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _PolePitch;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void CommutationMaster::AccessLinearResolution(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _PolePitch = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _PolePitch;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
