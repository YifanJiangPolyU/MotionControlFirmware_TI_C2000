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

#include "CommunicationInterface.h"

/**
 *  initialize the CiA message buffers
 *  @param msgptr    pointer to CiA_Message
 *  @param sdoptr
 *  @param pdoptr
 *  @retval None
 */
void CommunicationInterface::SetCiaMsgBuffer(CiA_Message * msgptr,
                                             CiA_SdoMessage * sdoptr,
                                             CiA_PdoMessage * pdoptr)
{
  ciamsg = msgptr;
  sdomsg = sdoptr;
  pdomsg = pdoptr;
}

/**
 *  transmits message(s) over communication interface
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CommunicationInterface::ExecuteTransmission(void){
  CiA_Message msg;
  msg.CANID = 123;
  msg.Length = 8;
  memcpy(&(msg.Data[0]), &(_ControlProcessData->_CurrentValuePhaseA[0]),
         4*sizeof(uint16_t));
  _UartDriver->SendMessage(&msg);

}

/**
 *  check and process messages received over communication interface
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CommunicationInterface::ExecuteReception(void){

  uint16_t msgcnt = _UartDriver->ExecuteParsing(ciamsg);
  if(msgcnt > 0 ){

    if(ciamsg->CANID == CANID_NMT){
      // handle CANOpen NMT protocol
      _ObjectDictionary->AccessEntry(ciamsg);
      _NmtNewState = __byte_uint16_t(ciamsg->Data, 0);
      _NmtUpdated = true;

    } else if((ciamsg->CANID-NODE_ID)==CANID_SDO_RX) {
      // handle CANOpen SDO

    } else if((ciamsg->CANID-NODE_ID)==CANID_PDO_RX) {
      // handle CANOpen PDO
      _PdoUpdated = true;
    } else {

    }
  }

  ciamsg->reserve = 0x0A;
}

/**
 *  retrive state transition cmd from NMT message
 *  @param mnt_state  buffer to hold new state
 *  @retval true if new NMT message is received, false otherwise
 */
#pragma CODE_SECTION(".TI.ramfunc");
bool CommunicationInterface::CheckNmtUpdate(uint16_t * mnt_state){
  if(_NmtUpdated == true){
    *mnt_state = _NmtNewState;
    _NmtUpdated = false;
    return true;
  }

  return false;
}

/**
 *  retrive state transition cmd from PDO message
 *  @param
 *  @retval true if new PDO message is received, false otherwise
 */
#pragma CODE_SECTION(".TI.ramfunc");
bool CommunicationInterface::CheckPdoUpdate(void){
  if(_PdoUpdated == true){
    _PdoUpdated = false;
    return true;
  }

  return false;
}
