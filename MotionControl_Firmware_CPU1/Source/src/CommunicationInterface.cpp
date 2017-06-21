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
 *  transmits message(s) over communication interface
 */
void CommunicationInterface::ExecuteTransmission(void){

}

/**
 *  check and process messages received over communication interface
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CommunicationInterface::ExecuteReception(void){

  CiA_Message msg;
  uint16_t msgcnt = _UartDriver->ExecuteParsing(&msg);
  if(msgcnt > 0 ){

    if(msg.CANID == CANID_NMT){
      // handle CANOpen NMT protocol
    } else if((msg.CANID-NODE_ID)==CANID_SDO_RX) {
      // handle CANOpen SDO
      CiA_SdoMessage sdomsg;
      memcpy(&(sdomsg.Ctrl), &(msg.Data[0]), sizeof(uint32_t));
      memcpy(&(sdomsg.Data[0]), &(msg.Data[2]), 4*sizeof(uint16_t));

    } else if((msg.CANID-NODE_ID)==CANID_PDO_RX) {
      // handle CANOpen PDO
      CiA_PdoMessage pdomsg;
      memcpy(&(pdomsg.Data[0]), &(msg.Data[0]), 6*sizeof(uint16_t));
    } else {
      msg.reserve = 0x0A;
    }
  }

}
