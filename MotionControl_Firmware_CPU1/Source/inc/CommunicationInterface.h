/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* communication interface
* collectively handle all communication traffic
*/

#ifndef COMMUNICATION_INTERFACE_H
#define COMMUNICATION_INTERFACE_H

#include "CiATypeDef.h"
#include "ControlProcessMaster.h"
#include "ObjectDictionary.h"

class CommunicationInterface{

  public:
    CommunicationInterface(ControlProcessMaster * ControlProcessMasterPtr,
                           ObjectDictionary * ObjectDictionaryPtr)
    {
      _ControlProcessMaster = ControlProcessMasterPtr;
      _ObjectDictionary = ObjectDictionaryPtr;
    }

    ~CommunicationInterface(){}

  private:
    ObjectDictionary * _ObjectDictionary;
    ControlProcessMaster * _ControlProcessMaster;

};

#endif
