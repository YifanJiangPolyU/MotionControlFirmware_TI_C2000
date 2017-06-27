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

/**
 *  define process IDs
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
      _CurrentLoopController(ControlProcessDataPtr)
    {
      _ControlProcessData = ControlProcessDataPtr;

      _ProcessArray[0] = NULL;
      _ProcessArray[1] = &_CurrentLoopController;
      _ProcessArray[2] = NULL;
      _ProcessArray[3] = NULL;
      _ProcessArray[4] = NULL;
      _ProcessArray[5] = NULL;
      _ProcessArray[6] = NULL;
      _ProcessArray[7] = NULL;
      _ProcessArray[8] = NULL;
      _ProcessArray[9] = NULL;
      _ProcessArray[10] = NULL;
      _ProcessArray[11] = NULL;
      _ProcessArray[12] = NULL;
      _ProcessArray[13] = NULL;
      _ProcessArray[14] = NULL;
      _ProcessArray[15] = NULL;
      _ProcessArray[16] = NULL;
      _ProcessArray[17] = NULL;
      _ProcessArray[18] = NULL;
      _ProcessArray[19] = NULL;
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

};

#endif
