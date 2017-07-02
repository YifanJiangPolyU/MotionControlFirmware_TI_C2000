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
/**
 *  pointer to globally unique object of ControlProcessMaster
 */
#pragma DATA_SECTION("CPU1DataRAM")
static ControlProcessMaster * This;

/**
 *  Constructor
 */
ControlProcessMaster::ControlProcessMaster(CommutationMaster * CommutationMasterPtr,
                                           CommunicationInterface * CommunicationInterfacePtr,
                                           ControlProcessData * ControlProcessDataPtr,
                                           ControlProcessExecuter * ControlProcessExecuterPtr):
  _State(STATE_STOPPED),
  _NmtUpdated(false),
  _NmtNewState(0),
  _CycleCounter(0)
  {
    This = this;
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
  GetData();

  //_CommunicationInterface->SetCiaMsgBuffer(&_CiA_MsgBuffer);

  // poll coummunication interface
  _CommunicationInterface->ExecuteReception();
  _NmtUpdated = _CommunicationInterface->CheckNmtUpdate(&_NmtNewState);

  // execute commutation angle calculation

  // check for errors

  // update state machine
  switch(_State){
    case STATE_PREOP:
      // mode switch by NMT
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_TO_OP){
          _State = STATE_OP;
        } else if(_NmtNewState==NMT_TO_STOP){
          PwrDisable();
          _State = STATE_STOPPED;
        }
      }

      _ControlProcessExecuter->ExecuteProcess();
      break;
    case STATE_OP:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_TO_PREOP){
          _State = STATE_PREOP;
        } else if(_NmtNewState==NMT_TO_STOP){
          _State = STATE_STOPPED;
        }
      }

      _ControlProcessExecuter->ExecuteProcess();
      break;
    case STATE_STOPPED:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_TO_PREOP){
          PwrEnable();
          _ControlProcessExecuter->StartProcess(PROCESS_CURRENT);
          _State = STATE_PREOP;
        }
      }
      break;
    case STATE_ERROR:
      if(_NmtUpdated==true){
        _NmtUpdated = false;
        if(_NmtNewState==NMT_TO_STOP){
          _State = STATE_STOPPED;
        }
      }
      break;
  }

  // update cycle counter to synchronize activities
  if(_CycleCounter==MASTER_CYCLE_PRESCALE-1){
    _CycleCounter = 0;
  } else {
    _CycleCounter++;
  }

  _CommunicationInterface->ExecuteTransmission(_CycleCounter);
}

/**
 *  pass ptr to the correct current sample buffer to the control process
 *  master.
 *  @ param bufA   ptr to phase A current sample buffer
 *  @ param bufB   ptr to phase B current sample buffer
 */
void ControlProcessMaster::SetCurrentValueBuffer(uint16_t * bufA, uint16_t * bufB){
  _ControlProcessData->_CurrentValueBufferPhaseA = bufA;
  _ControlProcessData->_CurrentValueBufferPhaseB = bufB;
}

/**
 *  get data from ADC buffer
 */
void ControlProcessMaster::GetData(void){
  _ControlProcessData->_CurrentValuePhaseA[_CycleCounter] =
                      *(_ControlProcessData->_CurrentValueBufferPhaseA+2);
  _ControlProcessData->_CurrentValuePhaseB[_CycleCounter] =
                      *(_ControlProcessData->_CurrentValueBufferPhaseB+2);
}

/**
 *  C warper to call ControlProcessMaster from ISR
 */
#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void CallControlProcessMaster(void){
  if(CLA_SampleBufferActiveHalf==0){
    This->SetCurrentValueBuffer(&(CLA_SampleBufferA[10]), &(CLA_SampleBufferB[10]));
  } else if(CLA_SampleBufferActiveHalf==1) {
    This->SetCurrentValueBuffer(&(CLA_SampleBufferA[0]), &(CLA_SampleBufferB[0]));
  }

  This->Execute();
}
