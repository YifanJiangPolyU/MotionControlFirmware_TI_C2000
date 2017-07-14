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

  int i = 0;
  for(i=0; i<MAX_NO_OF_ENTRY; i++){
    _ObdEntryList[i]._Idx = i*5;
  }

//  _InstanceArray[2] = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::_CurrentLoopController_GetInstance());
//  _AccessFunctionArray[2] = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)> (&CurrentLoopController::AccessCurrentLoopGains_Kp);
  _ObdEntryList[2]._Idx = 10;
  _ObdEntryList[2]._Instance = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::GetInstance()->_CurrentLoopController_GetInstance());
  _ObdEntryList[2]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)> (&CurrentLoopController::AccessCurrentLoopGains_Kp);

}
