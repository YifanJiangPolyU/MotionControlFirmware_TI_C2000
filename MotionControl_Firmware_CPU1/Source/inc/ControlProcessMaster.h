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

#include "CommutationMaster.h"
#include "CommunicationInterface.h"


class ControlProcessMaster{

  public:

    enum ControlProcessMaster_CIA_STATES {
      STATE_CIA_STOP,
      STATE_CIA_PREOP,
      STATE_CIA_OP
    };

    enum ControlProcessMaster_STATES {
      STATE_IDEL,
      STATE_ENABLE,
      STATE_CLSW,
      STATE_PLSW,
      STATE_POLARITY,
      STATE_ERROR
    };

    ControlProcessMaster(CommutationMaster * CommutationMasterPtr,
                         CommunicationInterface * CommunicationInterfacePtr);

    ~ControlProcessMaster(){}

    void Execute(void);

    typedef struct ControlProcessMaster_Status_typedef{
      uint16_t _InternalState:  3;
      uint16_t _CiaState:       1;
      uint16_t _Error:          12;
    } ControlProcessMaster_Status;

  private:
    enum ControlProcessMaster_STATES _State;
    enum ControlProcessMaster_CIA_STATES _CiA_State;

    CommutationMaster * _CommutationMaster;
    CommunicationInterface * _CommunicationInterface;

    uint16_t CycleCounter;
    uint16_t _NmtNewState;
    bool _NmtUpdated;
    ControlProcessMaster_Status _Status;

    CiA_Message _CiA_MsgBuffer;
    CiA_SdoMessage _CiA_SdoBuffer;
    CiA_PdoMessage _CiA_PdoBuffer;


};

#endif
