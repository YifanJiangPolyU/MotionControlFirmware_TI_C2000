/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* Control process master class
*
* Contains the main state machine of CPU1, handling control processes
*/

#ifndef CONTROL_PROCESS_MASTER_H
#define CONTROL_PROCESS_MASTER_H

#include "stdint.h"

#include "CPU1_CLA1_common.h"
#include "CommutationMaster.h"
#include "CommunicationInterface.h"
#include "ControlProcessData.h"
#include "CurrentLoopController.h"
#include "ControlProcessExecuter.h"
#include "ObjectDictionaryEntryBase.h"

#define MASTER_CYCLE_PRESCALE    4

class ControlProcessMaster: public ObjectDictionaryEntryBase{

  public:

/*
    enum ControlProcessMaster_STATES {
      STATE_PREOP,    // enabled
      STATE_OP,       // enabled and running
      STATE_STOPPED  // disabled
    };
*/
    enum ControlProcessMaster_STATES {
      STATE_NOT_READY,
      STATE_READY,
      STATE_SWITCHED_ON,
      STATE_OPERATION,
      STATE_QUICK_STOP,
      STATE_FAULT
    };

    ControlProcessMaster(CommutationMaster * CommutationMasterPtr,
                         CommunicationInterface * CommunicationInterfacePtr,
                         ControlProcessData * ControlProcessDataPtr,
                         ControlProcessExecuter * ControlProcessExecuterPtr);

    ~ControlProcessMaster(){}

    void SetCurrentValueBuffer(float32_t * bufA, float32_t * bufB);
    void UpdateProcessData(void);
    void CheckCurrentOverload(void);
    void SignalErrorState(void);
    void Execute(void);

    void AccessMotionControlState(ObdAccessHandle * handle);
    void AccessSystemStatusReg(ObdAccessHandle * handle);
    void AccessMotorCurrentLimitTimeConstant(ObdAccessHandle * handle);

    // define system status and error flags
    typedef struct StatusRefBit_typedef{
      uint16_t State :                3;
      uint16_t ErrOverCurrentPeak :   1;
      uint16_t ErrOverCurrentRms :    1;
      uint16_t ErrOverTemperature :   1;
      uint16_t ErrUnderVoltage :      1;
      uint16_t ErrOverVoltage :       1;
      uint16_t ErrPowerStage :        1;
      uint16_t ErrSoftware :          1;
      uint16_t ErrHardware :          1;
      uint16_t ErrCommunication :     1;
      uint16_t reserve :              5;
    } StatusRefBit;

    typedef union SystemStatusRegTypedef{
      StatusRefBit bit;
      uint16_t all;
    }SystemStatusReg;

  private:
    void UpdateMotionControlState(void);

    //enum ControlProcessMaster_STATES _State;
    enum ControlProcessMaster_STATES        _State;

    CommutationMaster * _CommutationMaster;
    CommunicationInterface * _CommunicationInterface;
    ControlProcessData * _ControlProcessData;
    ControlProcessExecuter * _ControlProcessExecuter;

    uint16_t _CycleCounter;
    uint16_t _NmtNewState;
    bool _NmtUpdated;
    SystemStatusReg _StatusReg;

    CiA_Message _CiA_MsgBuffer;
    CiA_SdoMessage _CiA_SdoBuffer;
    CiA_PdoMessage _CiA_PdoBuffer;

    float32_t _CurrentSquaredFiltered;

    // time window to calculate RMS current, unit: x100 ms
    // RMS current MUST be updated at 100Hz
    uint16_t _MotorCurrentLimitTimeConstant;
    float32_t _MovingAverageFactor1;
    float32_t _MovingAverageFactor2;


};

#endif
