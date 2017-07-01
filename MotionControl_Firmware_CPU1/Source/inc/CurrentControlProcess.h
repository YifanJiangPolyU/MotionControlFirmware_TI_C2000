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

#ifndef _CURRENT_CONTROL_PROCESS_H
#define _CURRENT_CONTROL_PROCESS_H

#include "ControlTypeDef.h"
#include "ControlProcessData.h"
#include "CurrentLoopController.h"
#include "ObjectDictionaryEntryBase.h"

class CurrentControlProcess : public ControlProcessBase, public ObjectDictionaryEntryBase{

  public:
    CurrentControlProcess(CurrentLoopController * CurrentLoopControllerPtr,
                          ControlProcessData * ControlProcessDataPtr)
    {
      _CurrentLoopController = CurrentLoopControllerPtr;
      _ControlProcessData = ControlProcessDataPtr;
    }

    ~CurrentControlProcess(){}

    virtual void Execute(void);

    virtual void Reset(void){
      _CurrentLoopController->Reset();
    }

    virtual void AccessObject(void){

    }

    uint16_t AccessParameter(CiA_Message * msg){
      return 3;
    }

  private:
    CurrentLoopController * _CurrentLoopController;
    ControlProcessData * _ControlProcessData;

};

#endif
