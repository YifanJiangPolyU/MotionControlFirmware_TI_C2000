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
#include "PdoMaster.h"
#include "ObjectDictionary.h"
#include "ControlProcessData.h"
#include "Drivers/UartDriver/UartDriver.h"

extern "C" {
  void OsTaskWarper_HandleObdAccess(void);
}

#define COMM_REPORT_RATE_8000    1
#define COMM_REPORT_RATE_4000    2
#define COMM_REPORT_RATE_2000    4
#define COMM_REPORT_RATE_1000    8
#define COMM_REPORT_RATE_500     16
#define COMM_REPORT_RATE_250     32
#define COMM_REPORT_RATE_125     64

class CommunicationInterface{

  public:
    CommunicationInterface(UartDriver * UartDriverPtr,
                           ObjectDictionary * ObjectDictionaryPtr,
                           ControlProcessData * ControlProcessDataPtr,
                           PdoMaster * PdoMasterPtr);

    ~CommunicationInterface(){}

    void SetCiaMsgBuffer(CiA_Message* msgptr);
    void ExecuteTransmission(void);
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
    PdoMaster * _PdoMaster;

    bool _NmtUpdated;
    bool _PdoUpdated;
    uint16_t _NmtNewState;

    bool _SdoReplyPending;
    CiA_Message _SodReplyMsg;

    uint16_t _TxCounter;
    uint16_t _TxRatePrescale;

};

#endif
