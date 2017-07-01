/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* ObjectDictionaryEntry class
* ObjectDictionary keeps a list of ObjectDictionaryEntry, which stores
* ptrs to target instance and target access function
*/

#ifndef OBJECT_DICTIONARY_ENTRY_H
#define OBJECT_DICTIONARY_ENTRY_H

#include "CiATypeDef.h"
#include "ObjectDictionaryEntryBase.h"

class ObjectDictionaryEntry{

  public:
    ObjectDictionaryEntry(ObjectDictionaryEntryBase* InstancePtr,
                          uint16_t (ObjectDictionaryEntryBase::*AccessFuncPtr)(CiA_Message*) ){
      _Instance = InstancePtr;
      _AccessFunctionPtr = AccessFuncPtr;
    }

    ~ObjectDictionaryEntry(){}

    uint16_t AccessEntry(CiA_Message * msg){
      return (_Instance->*(_AccessFunctionPtr))(msg);
    }

  private:
    ObjectDictionaryEntryBase * _Instance;
    uint16_t (ObjectDictionaryEntryBase::*_AccessFunctionPtr)(CiA_Message *);
};

#endif
