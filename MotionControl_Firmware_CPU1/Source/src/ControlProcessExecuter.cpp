/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement the control process executer class
*/

#include "ControlProcessExecuter.h"

ControlProcessExecuter::ControlProcessExecuter(ControlProcessData * ControlProcessDataPtr,
                       CurrentLoopController * CurrentLoopControllerPtr,
                       CurrentControlProcess * CurrentControlProcessPtr,
                       CurrentLoopSweepSine * CurrentLoopSweepSinePtr,
                       PositionControlProcess * PositionControlProcessPtr,
                       CalibrationProcess * CalibrationProcessPtr):
  _ActiveProcessID(PROCESS_NONE),
  _ProcessRunning(false)
{
  _ControlProcessData = ControlProcessDataPtr;

  uint16_t i;
  for(i=0; i<20; i++){
    _ProcessArray[i] = NULL;
  }

  // IMPORTANT: index of process classes MUST match with the process ID (SysDef.h)
  _ProcessArray[1] = CurrentControlProcessPtr;
  _ProcessArray[3] = PositionControlProcessPtr;
  _ProcessArray[10] = CurrentLoopSweepSinePtr;
  _ProcessArray[13] = CalibrationProcessPtr;
}


/**
 *  set the next process to be executed
 *  @param ProcessID   ID of the next active process
 */
void ControlProcessExecuter::StartProcess(uint16_t ProcessID){
  if(_ProcessArray[ProcessID] != NULL){
    // execute process if it exists
    _ProcessRunning = true;
    _ActiveProcessID = ProcessID;
    _ProcessArray[_ActiveProcessID]->Initialize();
    _ProcessArray[_ActiveProcessID]->Reset();
  }
}

/**
 *  Terminate currently active process
 *  doesn't do anything if no process is active
 */
void ControlProcessExecuter::TerminateProcess(void){
  if(_ProcessRunning == true){
    _ActiveProcessID = PROCESS_NONE;
    _ProcessRunning = false;
    _ProcessArray[_ActiveProcessID]->Terminate();
  }
}

/**
 *  execute the current active process
 *  @retval  true, if the process is terminated or is completed
 *                 ControlProcessMaster should switch to STATE_PREOP
 *           false, if the process is going to remain active
 *                  ControlProcessMaster should remain in STATE_OP
 */
#pragma CODE_SECTION(".TI.ramfunc");
bool ControlProcessExecuter::ExecuteProcess(void){

  if(_ProcessRunning){
    _ProcessArray[_ActiveProcessID]->Execute();

    // check for process completion
    if(_ProcessArray[_ActiveProcessID]->_GetShouldQuitStatus()){
      TerminateProcess();
    }
  }

  return (! _ProcessRunning);
}

/**
 *  get process status
 *  @retval  true, if the process is running
 *           false, if the process is not running
 */
#pragma CODE_SECTION(".TI.ramfunc");
bool ControlProcessExecuter::GetProcessStatus(void){
  return _ProcessRunning;
}
