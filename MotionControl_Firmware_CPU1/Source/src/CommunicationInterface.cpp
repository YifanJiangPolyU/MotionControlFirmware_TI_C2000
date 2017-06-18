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

  if((_UartDriver->GetBufferLevel()) > 0){
    _UartDriver->GetMessage(&msg);

    if(msg.CANID == CANID_NMT){
      // handle CANOpen NMT protocol
    } else if((msg.CANID-NODE_ID)==CANID_SDO_RX) {
      // handle CANOpen SDO
    } else if((msg.CANID-NODE_ID)==CANID_PDO_RX) {
      // handle CANOpen PDO
    } else {

    }
  }

}
