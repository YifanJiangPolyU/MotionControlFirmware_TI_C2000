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
  _State(STATE_IDEL),
  _CiA_State(STATE_CIA_PREOP),
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

  bool NmtUpdated = false;

  _CommunicationInterface->SetCiaMsgBuffer(&_CiA_MsgBuffer, &_CiA_SdoBuffer,
                                           &_CiA_PdoBuffer);
  // poll coummunication interface
  _CommunicationInterface->ExecuteReception();
  NmtUpdated = _CommunicationInterface->CheckNmtUpdate();

  // execute commutation angle calculation

  // update state machine
  switch(_State){
    case STATE_IDEL:
      break;
    case STATE_ENABLE:
      break;
    case STATE_ERROR:
      break;
    case STATE_CLSW:
      break;
    case STATE_PLSW:
      break;
    case STATE_POLARITY:
      break;
  }

  // update cycle counter to synchronize activities
  if(CycleCounter==3){
    CycleCounter = 0;
    _CommunicationInterface->ExecuteTransmission();
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
