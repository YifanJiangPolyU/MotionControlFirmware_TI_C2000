/****************************************************************************** 
  * Copyright (C) 2017 by Yifan Jiang                                          * 
  * jiangyi@student.ethz.com                                                   * 
  *                                                                            * 
  * This program is distributed in the hope that it will be useful,            * 
  * but WITHOUT ANY WARRANTY; without even the implied warranty of             * 
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       * 
  ******************************************************************************/ 
  
 /* 
 * Initialize object dictionary entries here 
 * This file is autmatically generated, do not make changes here 
 */ 

#include "ObjectDictionary.h" 
#include "SystemWarehouse.h"

void ObjectDictionary::InitObd(void){ 


  _ObdEntryList[0]._Idx = 2097153;
  _ObdEntryList[0]._AccessType = 0;
  _ObdEntryList[0]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[0]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessDcLineVoltage);

  _ObdEntryList[1]._Idx = 2097154;
  _ObdEntryList[1]._AccessType = 0;
  _ObdEntryList[1]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[1]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessDcLineCurrent);

  _ObdEntryList[2]._Idx = 2097409;
  _ObdEntryList[2]._AccessType = 0;
  _ObdEntryList[2]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[2]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessCpuTemperature);

  _ObdEntryList[3]._Idx = 2097410;
  _ObdEntryList[3]._AccessType = 0;
  _ObdEntryList[3]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[3]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessPowerStageTemperature);

  _ObdEntryList[4]._Idx = 2097665;
  _ObdEntryList[4]._AccessType = 0;
  _ObdEntryList[4]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[4]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessDcLineCurrentLimit);

  _ObdEntryList[5]._Idx = 2097666;
  _ObdEntryList[5]._AccessType = 0;
  _ObdEntryList[5]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[5]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessDcLineVoltageUpperLimit);

  _ObdEntryList[6]._Idx = 2097667;
  _ObdEntryList[6]._AccessType = 0;
  _ObdEntryList[6]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[6]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessDcLineVoltageLowerLimit);

  _ObdEntryList[7]._Idx = 2163713;
  _ObdEntryList[7]._AccessType = 1;
  _ObdEntryList[7]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_ControlProcessData_GetInstance());
  _ObdEntryList[7]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&ControlProcessData::AccessCommutationAngle);

  _ObdEntryList[8]._Idx = 2163714;
  _ObdEntryList[8]._AccessType = 1;
  _ObdEntryList[8]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CommutationMaster_GetInstance());
  _ObdEntryList[8]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CommutationMaster::AccessCountPerRev);

  _ObdEntryList[9]._Idx = 2163715;
  _ObdEntryList[9]._AccessType = 1;
  _ObdEntryList[9]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CommutationMaster_GetInstance());
  _ObdEntryList[9]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CommutationMaster::AccessLinearResolution);

  _ObdEntryList[10]._Idx = 2163716;
  _ObdEntryList[10]._AccessType = 1;
  _ObdEntryList[10]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CommutationMaster_GetInstance());
  _ObdEntryList[10]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CommutationMaster::AccessNPolePair);

  _ObdEntryList[11]._Idx = 2163717;
  _ObdEntryList[11]._AccessType = 1;
  _ObdEntryList[11]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CommutationMaster_GetInstance());
  _ObdEntryList[11]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CommutationMaster::AccessPolePitch);

  _ObdEntryList[12]._Idx = 2163969;
  _ObdEntryList[12]._AccessType = 0;
  _ObdEntryList[12]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopController_GetInstance());
  _ObdEntryList[12]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopController::AccessCurrentControlFrequency);

  _ObdEntryList[13]._Idx = 2163970;
  _ObdEntryList[13]._AccessType = 1;
  _ObdEntryList[13]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopController_GetInstance());
  _ObdEntryList[13]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopController::AccessCurrentLoopGains_Kp);

  _ObdEntryList[14]._Idx = 2163971;
  _ObdEntryList[14]._AccessType = 1;
  _ObdEntryList[14]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopController_GetInstance());
  _ObdEntryList[14]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopController::AccessCurrentLoopGains_Ki);

  _ObdEntryList[15]._Idx = 2164225;
  _ObdEntryList[15]._AccessType = 1;
  _ObdEntryList[15]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[15]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessStartFrequency);

  _ObdEntryList[16]._Idx = 2164226;
  _ObdEntryList[16]._AccessType = 1;
  _ObdEntryList[16]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[16]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessEndFrequency);

  _ObdEntryList[17]._Idx = 2164227;
  _ObdEntryList[17]._AccessType = 1;
  _ObdEntryList[17]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[17]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessRampRate);

  _ObdEntryList[18]._Idx = 2164228;
  _ObdEntryList[18]._AccessType = 1;
  _ObdEntryList[18]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[18]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessExcitationAmplitude);

  _ObdEntryList[19]._Idx = 2164229;
  _ObdEntryList[19]._AccessType = 1;
  _ObdEntryList[19]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[19]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessActivePhase);

  _ObdEntryList[20]._Idx = 2164230;
  _ObdEntryList[20]._AccessType = 1;
  _ObdEntryList[20]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[20]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessSweepSineMode);

  _ObdEntryList[21]._Idx = 2164231;
  _ObdEntryList[21]._AccessType = 1;
  _ObdEntryList[21]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[21]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessExcitationVoltageAngle);

  _ObdEntryList[22]._Idx = 2164232;
  _ObdEntryList[22]._AccessType = 0;
  _ObdEntryList[22]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopSweepSine_GetInstance());
  _ObdEntryList[22]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)>(&CurrentLoopSweepSine::AccessDataLength);

}
