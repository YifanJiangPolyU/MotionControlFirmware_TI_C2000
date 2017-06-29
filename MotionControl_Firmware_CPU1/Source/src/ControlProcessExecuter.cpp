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

/**
 *  set the next process to be executed
 *  @param ProcessID   ID of the next active process
 */
void ControlProcessExecuter::StartProcess(uint16_t ProcessID){
  if(_ProcessArray[ProcessID] != NULL){
    // execute process if it exists
    _ProcessRunning = true;
    _ActiveProcessID = ProcessID;
  }
}

/**
 *  terminate currently active process
 *  doesn't do anything if no process is active
 */
void ControlProcessExecuter::TerminateProcess(void){
  if(_ProcessRunning == true){
    _ActiveProcessID = PROCESS_NONE;
    _ProcessRunning = false;
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
  }

  return (! _ProcessRunning);
}
