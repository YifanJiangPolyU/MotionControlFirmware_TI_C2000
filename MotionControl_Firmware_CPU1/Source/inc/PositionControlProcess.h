/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define the current control process class
*/

#ifndef _POSITION_CONTROL_PROCESS_H
#define _POSITION_CONTROL_PROCESS_H

#include "ControlProcessData.h"
#include "CurrentLoopController.h"

class PositionControlProcess : public ControlProcessBase{

  public:
    PositionControlProcess(CurrentLoopController * CurrentLoopControllerPtr,
                          ControlProcessData * ControlProcessDataPtr)
    {
      _CurrentLoopController = CurrentLoopControllerPtr;
      _ControlProcessData = ControlProcessDataPtr;
    }

    ~PositionControlProcess(){}

    virtual void Execute(void){

    }

    virtual void Reset(void){

    }

  private:
    CurrentLoopController * _CurrentLoopController;
    ControlProcessData * _ControlProcessData;

};

#endif
