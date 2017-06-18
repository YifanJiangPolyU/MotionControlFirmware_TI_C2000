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

#ifndef SYSTEMWAREHOUSE_h
#define SYSTEMWAREHOUSE_h

#include "ControlProcessMaster.h"
#include "CommutationMaster.h"
#include "CommunicationInterface.h"
#include "ObjectDictionary.h"

class SystemWarehouse{
  public:
    SystemWarehouse():
      _ControlProcessMaster(&_CommutationMaster, &_CommunicationInterface),
      _CommutationMaster(),
      _CommunicationInterface(&_ObjectDictionary),
      _ObjectDictionary(&_CommutationMaster)
      {}

    ~SystemWarehouse(){}

  private:
    ControlProcessMaster _ControlProcessMaster;
    CommutationMaster _CommutationMaster;
    CommunicationInterface _CommunicationInterface;
    ObjectDictionary  _ObjectDictionary;
};


void CreateSystemWarehouseInstance(void);

#endif
