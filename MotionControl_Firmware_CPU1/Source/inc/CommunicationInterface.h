/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* communication interface
* collectively handle all communication traffic
*/

#ifndef COMMUNICATION_INTERFACE_H
#define COMMUNICATION_INTERFACE_H

#include "CiATypeDef.h"
#include "ObjectDictionary.h"
#include "ControlProcessData.h"
#include "Drivers/UartDriver/UartDriver.h"

extern "C" {

  void OsTaskWarper_HandleObdAccess(void);
}

class CommunicationInterface{

  public:
    CommunicationInterface(UartDriver * UartDriverPtr,
                           ObjectDictionary * ObjectDictionaryPtr,
                           ControlProcessData * ControlProcessDataPtr);

    ~CommunicationInterface(){}

    void SetCiaMsgBuffer(CiA_Message* msgptr);
    void ExecuteTransmission(uint16_t CycleCounter);
    void ExecuteReception(void);

    void HandleObdAccess(void);
    bool CheckNmtUpdate(uint16_t * mnt_state);
    bool CheckPdoUpdate(void);

    CiA_Message * ciamsg;
    CiA_Message _MsgBuffer;

  private:

    UartDriver * _UartDriver;
    ObjectDictionary * _ObjectDictionary;
    ControlProcessData * _ControlProcessData;

    bool _NmtUpdated;
    bool _PdoUpdated;
    uint16_t _NmtNewState;

    bool _SdoReplyPending;
    CiA_Message _SodReplyMsg;

};

#endif
