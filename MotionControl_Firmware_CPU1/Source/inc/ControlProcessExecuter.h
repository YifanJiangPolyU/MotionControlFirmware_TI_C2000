/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define the control process executer class
*/

#ifndef _CONTROL_PROCESS_EXECUTER_H
#define _CONTROL_PROCESS_EXECUTER_H

#include "stdint.h"
#include "SysDef.h"
#include "ControlProcessData.h"
#include "ControlProcessBase.h"
#include "CurrentLoopController.h"
#include "CurrentControlProcess.h"
#include "PositionControlProcess.h"
#include "CurrentLoopSweepSine.h"

class ControlProcessExecuter {

  public:
    ControlProcessExecuter(ControlProcessData * ControlProcessDataPtr,
                           CurrentLoopController * CurrentLoopControllerPtr,
                           CurrentControlProcess * CurrentControlProcessPtr,
                           CurrentLoopSweepSine * CurrentLoopSweepSinePtr,
                           PositionControlProcess * PositionControlProcessPtr);

    ~ControlProcessExecuter(){}

    void StartProcess(uint16_t ProcessID);
    void TerminateProcess(void);
    bool ExecuteProcess(void);
    bool GetProcessStatus(void);

  private:

    uint16_t _ActiveProcessID;
    bool _ProcessRunning;

    ControlProcessData * _ControlProcessData;
    ControlProcessBase * _ProcessArray[20];


};

#endif
