/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* Control process data class
*
* implement parameter access functions
*/

#include "ControlProcessData.h"

void ControlProcessData::AccessParameter(ObdAccessHandle * handle){

}

void ControlProcessData::AccessMotorType(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _MotorType = handle->Data.DataInt16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt16[0] = _MotorType;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessControlType(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _ControlType = handle->Data.DataInt16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt16[0] = _ControlType;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessDcLineVoltage(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt32 = _VoltageValueDcLine;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
  }
}

void ControlProcessData::AccessDcLineCurrent(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt32 = _CurrentValueDcLine;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
  }
}

void ControlProcessData::AccessCpuTemperature(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt32 = -1;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
  }
}

void ControlProcessData::AccessPowerStageTemperature(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt32 = -1;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
  }
}

void ControlProcessData::AccessCommutationAngles_Cos(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _CommAngleCosine = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CommAngleCosine;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessCommutationAngles_Sin(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _CommAngleSine = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CommAngleSine;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
