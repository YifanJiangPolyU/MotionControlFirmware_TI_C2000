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
#include "Drivers/GpioDriver/GpioDriver.h"


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
  _CycleCounter(0),
  _CurrentSquaredFiltered(0.0f),
  _MotorCurrentLimitTimeConstant(100)
  {
    ControlProcessMasterPtr = this;
    _CommutationMaster = CommutationMasterPtr;
    _CommunicationInterface = CommunicationInterfacePtr;
    _ControlProcessData = ControlProcessDataPtr;
    _ControlProcessExecuter = ControlProcessExecuterPtr;

    _MovingAverageFactor1 = 1.0f / ((float32_t)_MotorCurrentLimitTimeConstant * 10.0f);
    _MovingAverageFactor2 = 1.0f - _MovingAverageFactor1;

    _StatusReg.all = 0;
    _StatusReg.bit.State = STATE_NOT_READY;
  }

/**
 * execute ControlProcessMaster
 */
char EnablePwrTest = 0;
char EnablePwrTestState = 0;
#pragma CODE_SECTION(".TI.ramfunc");
void ControlProcessMaster::Execute(void){

  // Get data from current controller and ADC
  UpdateProcessData();

  // testing PwrEnable
  // should be removed after test
  switch (EnablePwrTestState) {
    case 0:
      if(EnablePwrTest==1){
        SetErrorLed();
        PwrEnable();
        EnablePwrTestState = 1;
      }
      break;
    case 1:
      if(EnablePwrTest==0){
        ClearErrorLed();
        PwrDisable();
        EnablePwrTestState = 0;
      }
      break;
  }

  //_CommunicationInterface->SetCiaMsgBuffer(&_CiA_MsgBuffer);

  // execute commutation angle calculation

  // check for errors
  //CheckCurrentOverload();

  // update state machine
  UpdateMotionControlState();

  // execute process
  _ControlProcessExecuter->ExecuteProcess();

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
 *  periodically check RMS current of the motor
 */
#pragma CODE_SECTION(".TI.ramfunc");
void ControlProcessMaster::CheckCurrentOverload(void){

  static uint16_t RmsUpdateCounter = 0;

  float32_t CurrentSquared = _ControlProcessData->_StatorCurrent.Alpha * 0.001;
  float32_t CurrentSquaredBeta = _ControlProcessData->_StatorCurrent.Beta * 0.001;
  CurrentSquared *= CurrentSquared;
  CurrentSquaredBeta *= CurrentSquaredBeta;
  CurrentSquared += CurrentSquaredBeta;

  // check if peak current limit is exceeded
  if((CurrentSquared >= _ControlProcessData->_MotorCurrentLimitPEAKSquared) &&
     (_State != STATE_FAULT)){
    _State = STATE_FAULT;
    PwrDisable();
    SetErrorLed();

    _StatusReg.bit.State = _State;
    _StatusReg.bit.ErrOverCurrentPeak = 1;
  }

  RmsUpdateCounter += 1;

  if(RmsUpdateCounter == 3199){

    RmsUpdateCounter = 0;

    // 1st order IIR filter approximating moving average
    _CurrentSquaredFiltered *= _MovingAverageFactor2;
    _CurrentSquaredFiltered += _MovingAverageFactor1 * CurrentSquared;

    // check if RMS current limit has been exceeded
    if((_CurrentSquaredFiltered >= _ControlProcessData->_MotorCurrentLimitRMSSquared) &&
       (_State != STATE_FAULT)){
      // rise an error
      _State = STATE_FAULT;
      PwrDisable();
      SetErrorLed();

      _StatusReg.bit.State = _State;
      _StatusReg.bit.ErrOverCurrentRms = 1;
    }
  }

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
        } else if(_NmtNewState==NMT_TEST_CALIBRATION) {
          _ControlProcessExecuter->StartProcess(PROCESS_CALIBRATION);
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
          ClearErrorLed();
          _State = STATE_READY;

          // clear error register
          _StatusReg.all = 0;
          _StatusReg.bit.State = _State;
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
  _ControlProcessData->UpdateMeasurements();
}

/**
 *  signal ControlProcessMaster to enter fault state
 */
void ControlProcessMaster::SignalErrorState(void){

  _State = STATE_FAULT;

  // disable power output
  PwrDisable();

  // turn on Error LED
  SetErrorLed();
}

/**
 *  C warper to call ControlProcessMaster from ISR
 */
#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void CallControlProcessMaster(void){

  ControlProcessMasterPtr->SetCurrentValueBuffer(CLA_SampleBufferPtrA, CLA_SampleBufferPtrB);
  ControlProcessMasterPtr->Execute();
}

/**
 *  C warper to call ControlProcessMaster SignalErrorState from ISR
 */
#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void SignalControlProcessMasterError(void){
  ControlProcessMasterPtr->SignalErrorState();
}



void ControlProcessMaster::AccessMotionControlState(ObdAccessHandle * handle){

}

void ControlProcessMaster::AccessSystemStatusReg(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      handle->AccessResult = OBD_ACCESS_ERR_WRITE;
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _StatusReg.all;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}

void ControlProcessMaster::AccessMotorCurrentLimitTimeConstant(ObdAccessHandle * handle){
  switch (handle->AccessType) {
    case SDO_CSS_WRITE:
      if(handle->Data.DataUint16[0]<=6000){
        _MotorCurrentLimitTimeConstant = handle->Data.DataUint16[0];
        _MovingAverageFactor1 = 1.0f / ((float32_t)_MotorCurrentLimitTimeConstant * 10.0f);
        _MovingAverageFactor2 = 1.0f - _MovingAverageFactor1;
        handle->AccessResult = OBD_ACCESS_SUCCESS;
      } else {
        handle->AccessResult = OBD_ACCESS_ERR_DATA_RANGE;
      }
      break;
    case SDO_CSS_READ:
      handle->Data.DataUint16[0] = _MotorCurrentLimitTimeConstant;
      handle->AccessResult = OBD_ACCESS_SUCCESS;
      break;
    default:
      break;
  }
}
