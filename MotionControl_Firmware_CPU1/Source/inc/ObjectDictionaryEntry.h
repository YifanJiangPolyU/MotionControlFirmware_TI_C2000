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
   ObjectDictionaryEntry(ObjectDictionaryEntryBase * InstancePtr,
                         void (ObjectDictionaryEntryBase::*AccessFunctionPtr)(ObdAccessHandle*)):
    _Instance(InstancePtr),
    _AccessFunction(AccessFunctionPtr)
  {

  }

   ~ObjectDictionaryEntry(){}



   // combination of object index and sub-index
   uint32_t _Idx;

   // whether the obj is RW or RO
   char _AccessType;

   // ptr to calling instance
   ObjectDictionaryEntryBase * _Instance;

   // ptr to access function
   void (ObjectDictionaryEntryBase::*_AccessFunction)(ObdAccessHandle*);
};

#endif
