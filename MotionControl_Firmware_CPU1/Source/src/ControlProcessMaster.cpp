/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/**
 * Control process master class
 *   contains the main state machine of CPU1, handling control processes
 */

#include "ControlProcessMaster.h"

/**
 *  pointer to globally unique object of ControlProcessMaster
 */
#pragma DATA_SECTION("CPU1DataRAM")
static ControlProcessMaster * This;

/**
 *  Constructor
 */
ControlProcessMaster::ControlProcessMaster(CommutationMaster * CommutationMasterPtr,
                                           CommunicationInterface * CommunicationInterfacePtr):
  _state(STATE_IDEL),
  CycleCounter(0)
  {
    This = this;
    _CommutationMaster = CommutationMasterPtr;
    _CommunicationInterface = CommunicationInterfacePtr;
  }

/**
 * execute ControlProcessMaster
 */
 #pragma CODE_SECTION(".TI.ramfunc");
void ControlProcessMaster::Execute(void){

    // poll coummunication interface
    _CommunicationInterface->ExecuteReception();

    // update cycle counter to synchronize activities
    if(CycleCounter==3){
      CycleCounter = 0;
    } else {
      CycleCounter++;
    }
}

/**
 *  C warper to call ControlProcessMaster from ISR
 */
#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void CallControlProcessMaster(void){
  This->Execute();
}
