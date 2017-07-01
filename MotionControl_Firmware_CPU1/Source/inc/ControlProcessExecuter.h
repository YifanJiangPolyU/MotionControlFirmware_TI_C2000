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
#include "ControlProcessData.h"
#include "ControlProcessBase.h"
#include "CurrentLoopController.h"
#include "CurrentControlProcess.h"
#include "PositionControlProcess.h"
#include "CurrentLoopSweepSine.h"

/**
 *  define process IDs
 *  IMPORTANT: process ID MUST match the index of the corresponding process class
 *  in the _ProcessArray
 */
#define PROCESS_NONE        0     // no process
#define PROCESS_CURRENT     1     // current control
#define PROCESS_SPEED       2     // speed control (rotary only)
#define PROCESS_POSITION    3     // position control
#define PROCESS_FORCE       4     // force or torque control
#define PROCESS_CLSW        10    // current loop sweepsine test process
#define PROCESS_PLSW        11    // position loop sweepsine test process
#define PROCESS_POLARITY    12    // polarity test process

class ControlProcessExecuter {

  public:
    ControlProcessExecuter(ControlProcessData * ControlProcessDataPtr):
      _ActiveProcessID(PROCESS_NONE),
      _ProcessRunning(false),
      _CurrentLoopController(ControlProcessDataPtr),
      _CurrentControlProcess(&_CurrentLoopController, ControlProcessDataPtr),
      _PositionControlProcess(&_CurrentLoopController, ControlProcessDataPtr),
      _CurrentLoopSweepSine(&_CurrentLoopController, ControlProcessDataPtr)
    {
      _ControlProcessData = ControlProcessDataPtr;

      uint16_t i;
      for(i=0; i<20; i++){
        _ProcessArray[i] = NULL;
      }

      // IMPORTANT: index of process classes MUST match with the process ID
      _ProcessArray[1] = &_CurrentControlProcess;
      _ProcessArray[3] = &_PositionControlProcess;
      _ProcessArray[10] = &_CurrentLoopSweepSine;
    }

    ~ControlProcessExecuter(){}

    void StartProcess(uint16_t ProcessID);
    void TerminateProcess(void);
    bool ExecuteProcess(void);

  private:

    uint16_t _ActiveProcessID;
    bool _ProcessRunning;

    ControlProcessData * _ControlProcessData;
    ControlProcessBase * _ProcessArray[20];

    CurrentLoopController _CurrentLoopController;
    CurrentControlProcess _CurrentControlProcess;
    PositionControlProcess _PositionControlProcess;
    CurrentLoopSweepSine _CurrentLoopSweepSine;

};

#endif
