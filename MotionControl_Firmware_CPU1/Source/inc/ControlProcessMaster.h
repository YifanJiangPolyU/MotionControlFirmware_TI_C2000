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

#define MASTER_CYCLE_PRESCALE    4

class ControlProcessMaster{

  public:

    enum ControlProcessMaster_STATES {
      STATE_PREOP,    // enabled
      STATE_OP,       // enabled and running
      STATE_STOPPED,  // disabled
      STATE_ERROR     // error
    };

    ControlProcessMaster(CommutationMaster * CommutationMasterPtr,
                         CommunicationInterface * CommunicationInterfacePtr,
                         ControlProcessData * ControlProcessDataPtr,
                         CurrentLoopController * CurrentLoopControllerPtr);

    ~ControlProcessMaster(){}

    void SetCurrentValueBuffer(uint16_t * bufA, uint16_t * bufB);
    void GetData(void);
    void Execute(void);

    typedef struct ControlProcessMaster_Status_typedef{
      uint16_t _InternalState:  3;
      uint16_t _CiaState:       1;
      uint16_t _Error:          12;
    } ControlProcessMaster_Status;

  private:
    enum ControlProcessMaster_STATES _State;

    CommutationMaster * _CommutationMaster;
    CommunicationInterface * _CommunicationInterface;
    ControlProcessData * _ControlProcessData;
    CurrentLoopController * _CurrentLoopController;

    uint16_t CycleCounter;
    uint16_t _NmtNewState;
    bool _NmtUpdated;
    ControlProcessMaster_Status _Status;

    CiA_Message _CiA_MsgBuffer;
    CiA_SdoMessage _CiA_SdoBuffer;
    CiA_PdoMessage _CiA_PdoBuffer;


};

#endif
