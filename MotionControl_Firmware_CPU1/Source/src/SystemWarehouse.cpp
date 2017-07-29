/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* system warehouse
*
* initialize statically and hold all objects
*/

#include "SystemWarehouse.h"

/**
 *  create globally unique object of SystemWarehouse
 *    call this ONLY ONCE
 */
void CreateSystemWarehouseInstance(void){
  SystemWarehouse::CreateInstance();
}

void SystemWarehouse::CreateInstance(void){
  GetInstance();
}

/**
 *  return a pointer to the globally unique objecte of SystemWarehouse
 */
SystemWarehouse * SystemWarehouse::GetInstance(void){

  #pragma DATA_SECTION("CPU1DataRAM")
  static SystemWarehouse SystemWarehouseInstance;

  return &SystemWarehouseInstance;
}

ControlProcessMaster * SystemWarehouse::_ControlProcessMaster_GetInstance(void){
  return &_ControlProcessMaster;
}

ControlProcessExecuter * SystemWarehouse::_ControlProcessExecuter_GetInstance(void){
  return &_ControlProcessExecuter;
}

CommutationMaster * SystemWarehouse::_CommutationMaster_GetInstance(void){
  return &_CommutationMaster;
}

CommunicationInterface * SystemWarehouse::_CommunicationInterface_GetInstance(void){
  return &_CommunicationInterface;
}

ObjectDictionary  * SystemWarehouse::_ObjectDictionary_GetInstance(void){
  return &_ObjectDictionary;
}

ControlProcessData * SystemWarehouse::_ControlProcessData_GetInstance(void){
  return &_ControlProcessData;
}

CurrentLoopController * SystemWarehouse::_CurrentLoopController_GetInstance(void){
  return &_CurrentLoopController;
}

CurrentControlProcess * SystemWarehouse::_CurrentControlProcess_GetInstance(void){
  return &_CurrentControlProcess;
}

PositionControlProcess * SystemWarehouse::_PositionControlProcess_GetInstance(void){
  return &_PositionControlProcess;
}

CalibrationProcess * SystemWarehouse::_CalibrationProcess_GetInstance(void){
  return &_CalibrationProcess;
}

CurrentLoopSweepSine * SystemWarehouse::_CurrentLoopSweepSine_GetInstance(void){
  return &_CurrentLoopSweepSine;
}
