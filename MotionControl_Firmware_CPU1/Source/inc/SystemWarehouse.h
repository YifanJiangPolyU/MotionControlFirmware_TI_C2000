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
#include "ControlProcessExecuter.h"
#include "CommutationMaster.h"
#include "CommunicationInterface.h"
#include "ObjectDictionary.h"
#include "ControlProcessData.h"
#include "CurrentLoopController.h"
#include "Drivers/UartDriver/UartDriver.h"

class SystemWarehouse{
  public:
    SystemWarehouse():
      _ControlProcessMaster(&_CommutationMaster, &_CommunicationInterface,
                            &_ControlProcessData, &_ControlProcessExecuter),
      _ControlProcessExecuter(&_ControlProcessData),
      _CommutationMaster(),
      _CommunicationInterface(&_UartDriver, &_ObjectDictionary, &_ControlProcessData),
      _ObjectDictionary(&_CommutationMaster),
      _CurrentLoopController(&_ControlProcessData),
      _ControlProcessData(),
      _UartDriver()
      {}

    ~SystemWarehouse(){}

  private:
    ControlProcessMaster _ControlProcessMaster;
    ControlProcessExecuter _ControlProcessExecuter;
    CommutationMaster _CommutationMaster;
    CommunicationInterface _CommunicationInterface;
    ObjectDictionary  _ObjectDictionary;
    ControlProcessData _ControlProcessData;
    CurrentLoopController _CurrentLoopController;

    UartDriver _UartDriver;
};


void CreateSystemWarehouseInstance(void);

#endif
