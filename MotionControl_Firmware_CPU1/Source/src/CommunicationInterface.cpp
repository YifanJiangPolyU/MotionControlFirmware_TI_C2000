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
void CommunicationInterface::SetCiaMsgBuffer(CiA_Message * msgptr)
{
  ciamsg = msgptr;
}

/**
 *  transmits message(s) over communication interface
 *  param CycleCounter   keep transmisson synchronized to ControlProcessMaster
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CommunicationInterface::ExecuteTransmission(uint16_t CycleCounter){
  CiA_Message msg;

  switch (CycleCounter) {
    case 0:
      // transmit PDO status report
      msg.Common.CANID = 123;
      msg.Common.Length = 8;
      memcpy(&(msg.Common.Data[0]), &(_ControlProcessData->_CurrentValuePhaseA[0]),
             4*sizeof(uint16_t));
      _UartDriver->SendMessage(&msg);
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      // transmit SDO reply, if any
      if(_SdoReplyPending==true){
        _SdoReplyPending = false;

      }
      break;
    default:
      break;
  }

}

/**
 *  check and process messages received over communication interface
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CommunicationInterface::ExecuteReception(void){

  uint16_t msgcnt = _UartDriver->ExecuteParsing(ciamsg);
  if(msgcnt > 0 ){

    if(ciamsg->Common.CANID == CANID_NMT){
      // handle CANOpen NMT protocol
      _NmtNewState = __byte_uint16_t(ciamsg->Common.Data, 0);
      _NmtUpdated = true;

    } else if(ciamsg->Common.CANID == CANID_SYNC){
      // handle sync msg

    } else if((ciamsg->Common.CANID-NODE_ID)==CANID_SDO_RX) {
      // handle CANOpen SDO
      _ObjectDictionary->AccessEntry(ciamsg, &_SodReplyMsg);
      _SdoReplyPending = true;

    } else if((ciamsg->Common.CANID-NODE_ID)==CANID_PDO_RX) {
      // handle CANOpen PDO
      _PdoUpdated = true;
    } else {
      // default, nothing
    }
  }

  //ciamsg->reserve = 0x0A;
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
