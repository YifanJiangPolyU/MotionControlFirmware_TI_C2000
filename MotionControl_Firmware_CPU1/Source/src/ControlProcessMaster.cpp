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
#include "Drivers/PowerStageControl/PowerStageControl.h"
#include "Drivers/EncoderDriver/EncoderDriver.h"

/**
 *  pointer to globally unique object of ControlProcessMaster
 */
#pragma DATA_SECTION("CPU1DataRAM")
static ControlProcessMaster * ControlProcessMasterPtr;

/**
 *  Constructor
 */
ControlProcessMaster::ControlProcessMaster(CommutationMaster * CommutationMasterPtr,
                                           CommunicationInterface * CommunicationInterfacePtr,
                                           ControlProcessData * ControlProcessDataPtr,
                                           ControlProcessExecuter * ControlProcessExecuterPtr):
  _State(STATE_NOT_READY),
  _NmtUpdated(false),
  _NmtNewState(0),
  _CycleCounter(0)
  {
    ControlProcessMasterPtr = this;
    _CommutationMaster = CommutationMasterPtr;
    _CommunicationInterface = CommunicationInterfacePtr;
    _ControlProcessData = ControlProcessDataPtr;
    _ControlProcessExecuter = ControlProcessExecuterPtr;
  }

/**
 * execute ControlProcessMaster
 */
 #pragma CODE_SECTION(".TI.ramfunc");
void ControlProcessMaster::Execute(void){

  // Get data from current controller and ADC
  UpdateProcessData();

  //_CommunicationInterface->SetCiaMsgBuffer(&_CiA_MsgBuffer);

  // execute commutation angle calculation

  // check for errors

  // update state machine
  UpdateMotionControlState();

  // execute process
  _ControlProcessExecuter->ExecuteProcess();

/*
  switch(_State){
    case STATE_PREOP:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_TO_OP){
          PwrEnable();
          _ControlProcessExecuter->StartProcess(_ControlProcessData->_ActiveProcess);
          _State = STATE_PREOP;
        }
      }
      break;
    case STATE_OP:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_TO_PREOP){
          PwrDisable();
          _State = STATE_PREOP;
        } else if(_NmtNewState==NMT_TO_STOP){
          PwrDisable();
          _State = STATE_STOPPED;
        }
      }

      _ControlProcessExecuter->ExecuteProcess();
      break;
    case STATE_STOPPED:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_TO_PREOP){
          _State = STATE_PREOP;
        }
      }
      break;
    default:
      break;
  }
*/

  // poll coummunication interface
  _CommunicationInterface->ExecuteReception();
  _NmtUpdated = _CommunicationInterface->CheckNmtUpdate(&_NmtNewState);

  // transmit data
  _CommunicationInterface->ExecuteTransmission();

  // update cycle counter to synchronize activities
  if(_CycleCounter==MASTER_CYCLE_PRESCALE-1){
    _CycleCounter = 0;
  } else {
    _CycleCounter++;
  }

  // update synchronization flag
  _ControlProcessData->_SyncFlag = _CycleCounter;
}

/**
 *  Update the ControlProcessMaster state machine
 *  State definition compatible with CiA 402 standard
 */
#pragma CODE_SECTION(".TI.ramfunc");
void ControlProcessMaster::UpdateMotionControlState(void){
  switch (_State) {
    case STATE_NOT_READY:
      _State = STATE_READY;
      break;
    case STATE_READY:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_SWITCH_ON){
          PwrEnable();
          _ControlProcessExecuter->StartProcess(_ControlProcessData->_ControlProcess);
          _State = STATE_SWITCHED_ON;
        } else if(_NmtNewState==NMT_TEST_CLSW){
          PwrEnable();
          _ControlProcessExecuter->StartProcess(PROCESS_CLSW);
          _State = STATE_SWITCHED_ON;
        } else if(_NmtNewState==NMT_TEST_PLSW){
          PwrEnable();
          _ControlProcessExecuter->StartProcess(PROCESS_PLSW);
          _State = STATE_SWITCHED_ON;
        } else if(_NmtNewState==NMT_TEST_POLARITY){
          PwrEnable();
          _ControlProcessExecuter->StartProcess(PROCESS_POLARITY);
          _State = STATE_SWITCHED_ON;
        }
      }
      break;
    case STATE_SWITCHED_ON:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_ENABLE_OP){
          _ControlProcessData->_OperationEnabled = true;
          _State = STATE_OPERATION;
        } else if(_NmtNewState==NMT_SWITCH_OFF){
          PwrDisable();
          _ControlProcessExecuter->TerminateProcess();
          _State = STATE_READY;
        }
      }

      if(!_ControlProcessExecuter->GetProcessStatus()){
        _State = STATE_READY;
      }
      break;
    case STATE_OPERATION:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_SWITCH_OFF){
          PwrDisable();
          _ControlProcessExecuter->TerminateProcess();
          _ControlProcessData->_OperationEnabled = false;
          _State = STATE_READY;
        } else if(_NmtNewState==NMT_DISABLE_OP){
          _ControlProcessData->_OperationEnabled = false;
          _State = STATE_SWITCHED_ON;
        } else if(_NmtNewState==NMT_QUICK_STOP){
          _ControlProcessData->_OperationEnabled = false;
          _State = STATE_QUICK_STOP;
        }
      }
      break;
    case STATE_QUICK_STOP:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
      }
      break;
    case STATE_FAULT:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NME_RESET_FAULT){

        }
      }
      break;
    default:
      // should not get here
      break;
  }
}

/**
 *  pass ptr to the correct current sample buffer to the control process
 *  master.
 *  @ param bufA   ptr to phase A current sample buffer
 *  @ param bufB   ptr to phase B current sample buffer
 */
#pragma CODE_SECTION(".TI.ramfunc");
void ControlProcessMaster::SetCurrentValueBuffer(float32_t * bufA, float32_t * bufB){
  _ControlProcessData->_CurrentValueBufferPhaseA = bufA;
  _ControlProcessData->_CurrentValueBufferPhaseB = bufB;
}

/**
 *  get data from ADC buffer
 */
#pragma CODE_SECTION(".TI.ramfunc");
void ControlProcessMaster::UpdateProcessData(void){

  _ControlProcessData->_CurrentActualValue.A = *(_ControlProcessData->_CurrentValueBufferPhaseA+2);
  _ControlProcessData->_CurrentActualValue.B = *(_ControlProcessData->_CurrentValueBufferPhaseB+2);

  _ControlProcessData->_Position = GetEncoder1Position();
}

/**
 *  C warper to call ControlProcessMaster from ISR
 */
#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void CallControlProcessMaster(void){

  ControlProcessMasterPtr->SetCurrentValueBuffer(CLA_SampleBufferPtrA, CLA_SampleBufferPtrB);
  ControlProcessMasterPtr->Execute();
}


void AccessMotionControlState(ObdAccessHandle * handle){

}
