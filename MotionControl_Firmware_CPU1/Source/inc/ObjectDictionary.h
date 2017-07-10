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
#include "ObjectDictionaryEntry.h"
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
      _ObdEntryList[i]._Idx = i*5;
      _ObdEntryList[i]._Instance = NULL;
      _ObdEntryList[i]._AccessMethod = NULL;
    }

    _ObdEntryList[2]._Instance = static_cast<ObjectDictionaryEntryBase*>(CurrentLoopControllerPtr);
    _ObdEntryList[2]._AccessMethod = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)> (&CurrentLoopController::AccessCurrentLoopGains_Kp);

    //InitObd();
  }

  ~ObjectDictionary(){}

  void AccessEntry(CiA_Message * msg_in, CiA_Message * msg_out);
  void InitObd(void);

private:

  int16_t SearchEntry(uint16_t Idx, uint16_t SubIdx);

  CommutationMaster * _CommutationMaster;

  ObjectDictionaryEntry _ObdEntryList[MAX_NO_OF_ENTRY];

};


#endif
