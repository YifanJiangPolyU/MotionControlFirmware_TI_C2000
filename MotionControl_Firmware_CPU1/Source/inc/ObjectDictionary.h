/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* ObjectDictionary class
*
* implements object dictionary, which is compatible with CANOpen standard (CiA301)
* but is also extendable to make use of higher-BW communication methods
*/

#ifndef OBJECT_DICTIONARY_H
#define OBJECT_DICTIONARY_H

#include "stdint.h"
#include "F28x_Project.h"

#include "CommutationMaster.h"
#include "CiATypeDef.h"
#include "ObjectDictionaryEntryBase.h"
#include "ControlProcessData.h"
#include "CurrentLoopController.h"
#include "CurrentControlProcess.h"
#include "CurrentLoopSweepSine.h"
#include "PositionControlProcess.h"

#define MAX_NO_OF_ENTRY 100

class ObjectDictionary{

public:

  ObjectDictionary(ControlProcessData * ControlProcessDataPtr,
                   CommutationMaster * CommutationMasterPtr,
                   CurrentLoopController * CurrentLoopControllerPtr)
  {
    _CommutationMaster = CommutationMasterPtr;

    uint16_t i;
    for(i=0; i<MAX_NO_OF_ENTRY; i++){
      _InstanceArray[i] = NULL;
      _AccessFunctionArray[i] = NULL;
    }

    _InstanceArray[0] = static_cast<ObjectDictionaryEntryBase*>(CommutationMasterPtr);
    _AccessFunctionArray[0] = static_cast<void (ObjectDictionaryEntryBase::*)(CiA_Message*, CiA_Message*)> (&CommutationMaster::AccessParameter);

    _InstanceArray[1] = static_cast<ObjectDictionaryEntryBase*>(ControlProcessDataPtr);
    _AccessFunctionArray[1] = static_cast<void (ObjectDictionaryEntryBase::*)(CiA_Message*, CiA_Message*)> (&ControlProcessData::AccessParameter);
  }

  ~ObjectDictionary(){}

  void AccessEntry(CiA_Message * msg_in, CiA_Message * msg_out){
    (_InstanceArray[0]->*(_AccessFunctionArray[0]))(msg_in, msg_out);
  }

private:

  CommutationMaster * _CommutationMaster;
  ObjectDictionaryEntryBase * _InstanceArray[ MAX_NO_OF_ENTRY ];
  void (ObjectDictionaryEntryBase::*_AccessFunctionArray[ MAX_NO_OF_ENTRY ])(CiA_Message*, CiA_Message*);


};


#endif
