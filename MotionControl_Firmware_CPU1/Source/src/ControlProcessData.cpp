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
#include "CPU1_CLA1_common.h"

const float32_t ADC_RESOLUTION = 4096;
const float32_t ADC_REF_VOLTAGE = 3.0f;

const float32_t CURRENT_SENSE_AMP_GAIN = 10.0f;
const float32_t CURRENT_SENSE_RES_VALUE = 0.01f;

const float32_t CURREN_SENSE_GAIN_PHASE = ADC_REF_VOLTAGE/
                                          ADC_RESOLUTION/
                                          CURRENT_SENSE_AMP_GAIN/
                                          CURRENT_SENSE_RES_VALUE;

const float32_t CURREN_SENSE_GAIN_DCLINE = CURREN_SENSE_GAIN_PHASE;
const float32_t VOLTAGE_SENSE_GAIN_DCLINE = ADC_REF_VOLTAGE/ADC_RESOLUTION/0.125156759468272f;

void ControlProcessData::InitCLAGains(void){
  CLA_CurrentSenseGain_Phase = CURREN_SENSE_GAIN_PHASE;
  CLA_CurrentSenseGain_DcLine = CURREN_SENSE_GAIN_PHASE;
  CLA_VoltageSenseGain = VOLTAGE_SENSE_GAIN_DCLINE;
}

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

void ControlProcessData::AccessControlProcess(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _ControlProcess = handle->Data.DataInt16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataInt16[0] = _ControlProcess;
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

void ControlProcessData::AccessDcLineVoltageUpperLimit(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _DcLineVoltageUpperLimit;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessDcLineVoltageLowerLimit(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _DcLineVoltageLowerLimit;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessDcLineCurrentLimit(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _DcLineCurrentLimit;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessMotorCurrentLimitRMS(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _MotorCurrentLimitRMS = handle->Data.DataUint16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _MotorCurrentLimitRMS;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessMotorCurrentLimitPEAK(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _MotorCurrentLimitPEAK = handle->Data.DataUint16[0];
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _MotorCurrentLimitPEAK;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}


void ControlProcessData::AccessCommutationAngle(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CommAngle;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessCommutationAngle_Cos(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CommAngleCosine;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessCommutationAngle_Sin(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _CommAngleSine;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessDQCurrentSetpoint_D(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _DQCurrentSetpoint.D = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _DQCurrentSetpoint.D;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessData::AccessDQCurrentSetpoint_Q(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      _DQCurrentSetpoint.Q = handle->Data.DataFloat32;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    case SDO_CSS_READ:
      handle->Data.DataFloat32 = _DQCurrentSetpoint.Q;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
