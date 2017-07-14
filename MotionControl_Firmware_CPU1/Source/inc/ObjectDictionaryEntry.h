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

typedef struct ObjectDictionaryEntryTypeDef{

  uint32_t _Idx;
  char _AccessType;
  char _DataLength;
  ObjectDictionaryEntryBase * _Instance;
  void (ObjectDictionaryEntryBase::*_AccessMethod)(ObdAccessHandle*);

} ObjectDictionaryEntry;

#endif
