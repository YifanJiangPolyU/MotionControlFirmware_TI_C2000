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
#include "CurrentControlProcess.h"
#include "CurrentLoopSweepSine.h"
#include "PositionControlProcess.h"


#include "Drivers/UartDriver/UartDriver.h"

class SystemWarehouse{
  public:
    SystemWarehouse():
      _ControlProcessMaster(&_CommutationMaster, &_CommunicationInterface,
                            &_ControlProcessData, &_ControlProcessExecuter),
      _ControlProcessExecuter(&_ControlProcessData,
                              &_CurrentLoopController,
                              &_CurrentControlProcess,
                              &_CurrentLoopSweepSine,
                              &_PositionControlProcess),
      _ObjectDictionary(),
      _CommutationMaster(),
      _CommunicationInterface(&_UartDriver, &_ObjectDictionary, &_ControlProcessData),
      _CurrentLoopController(&_ControlProcessData),
      _CurrentControlProcess(&_CurrentLoopController, &_ControlProcessData),
      _PositionControlProcess(&_CurrentLoopController, &_ControlProcessData),
      _CurrentLoopSweepSine(&_CurrentLoopController, &_ControlProcessData),

      _ControlProcessData(),
      _UartDriver()
      {}

    ~SystemWarehouse(){}

    ControlProcessMaster * _ControlProcessMaster_GetInstance(void);
    ControlProcessExecuter * _ControlProcessExecuter_GetInstance(void);
    CommutationMaster * _CommutationMaster_GetInstance(void);
    CommunicationInterface * _CommunicationInterface_GetInstance(void);
    ObjectDictionary  * _ObjectDictionary_GetInstance(void);
    ControlProcessData * _ControlProcessData_GetInstance(void);

    CurrentLoopController * _CurrentLoopController_GetInstance(void);
    CurrentControlProcess * _CurrentControlProcess_GetInstance(void);
    PositionControlProcess * _PositionControlProcess_GetInstance(void);
    CurrentLoopSweepSine * _CurrentLoopSweepSine_GetInstance(void);

    static void CreateInstance(void);
    static SystemWarehouse * GetInstance(void);

  private:
    ControlProcessMaster _ControlProcessMaster;
    ControlProcessExecuter _ControlProcessExecuter;
    CommutationMaster _CommutationMaster;
    CommunicationInterface _CommunicationInterface;
    ObjectDictionary  _ObjectDictionary;
    ControlProcessData _ControlProcessData;

    CurrentLoopController _CurrentLoopController;
    CurrentControlProcess _CurrentControlProcess;
    PositionControlProcess _PositionControlProcess;
    CurrentLoopSweepSine _CurrentLoopSweepSine;

    UartDriver _UartDriver;
};


void CreateSystemWarehouseInstance(void);

#endif
