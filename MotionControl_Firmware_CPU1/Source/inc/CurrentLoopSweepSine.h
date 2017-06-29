/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* current loop sweep sine class
*
* Control the execution of current loop sweep sine, to obtain the frequency
* domain response of the current loop. This is the sweep sine process
* controller and setpoint generator only, current loop is executed by CLA.
*/

#ifndef _CURRENT_LOOP_SWEEPSINE_H
#define _CURRENT_LOOP_SWEEPSINE_H

#include "ControlProcessData.h"
#include "CurrentLoopController.h"

class CurrentLoopSweepSine : public ControlProcessBase{

  public:
    CurrentLoopSweepSine(CurrentLoopController * CurrentLoopControllerPtr,
                          ControlProcessData * ControlProcessDataPtr)
    {
      _CurrentLoopController = CurrentLoopControllerPtr;
      _ControlProcessData = ControlProcessDataPtr;
    }

    ~CurrentLoopSweepSine(){}

    virtual void Execute(void){

    }

    virtual void Reset(void){

    }

  private:
    CurrentLoopController * _CurrentLoopController;
    ControlProcessData * _ControlProcessData;

};

#endif
