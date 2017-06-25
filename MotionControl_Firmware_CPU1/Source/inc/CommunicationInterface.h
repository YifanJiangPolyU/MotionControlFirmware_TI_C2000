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

class CommunicationInterface{

  public:
    CommunicationInterface(UartDriver * UartDriverPtr,
                           ObjectDictionary * ObjectDictionaryPtr,
                           ControlProcessData * ControlProcessDataPtr)
    {
      _UartDriver = UartDriverPtr;
      _ObjectDictionary = ObjectDictionaryPtr;
      _ControlProcessData = ControlProcessDataPtr;
      _NmtUpdated = false;
      _PdoUpdated = false;
      _NmtNewState = 0x00;
    }

    ~CommunicationInterface(){}

    void SetCiaMsgBuffer(CiA_Message* msgptr, CiA_SdoMessage* sdoptr, CiA_PdoMessage* pdoptr);
    void ExecuteTransmission(void);
    void ExecuteReception(void);

    bool CheckNmtUpdate(uint16_t * mnt_state);

  private:
    UartDriver * _UartDriver;
    ObjectDictionary * _ObjectDictionary;
    ControlProcessData * _ControlProcessData;

    bool _NmtUpdated;
    bool _PdoUpdated;
    uint16_t _NmtNewState;

    CiA_Message * ciamsg;
    CiA_SdoMessage * sdomsg;
    CiA_PdoMessage * pdomsg;

};

#endif
