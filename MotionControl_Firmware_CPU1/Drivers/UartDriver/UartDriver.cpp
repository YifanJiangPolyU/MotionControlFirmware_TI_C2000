/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/**
 * handling traffic from UART
 */

#include "UartDriver.h"


UartDriver::UartDriver():
  _state(STATE_IDEL)
{

}

UartDriver::~UartDriver(){

}

/**
 *  extract CiA message from UART data
 *  must be called periodically, at 32kHz
 *
 * @param msg   buffer to hold extracted message
 * @retval      1 if new message is ready, 0 otherwise
 */
#pragma CODE_SECTION(".TI.ramfunc");
uint16_t UartDriver::ExecuteParsing(CiA_Message * msg){

  uint16_t fifo_counter = 0;
  static uint16_t data_counter = 0;
  static uint16_t data_length = 0;
  uint16_t tmp = 0;
  uint16_t retval = 0;

  fifo_counter = SciaRegs.SCIFFRX.bit.RXFFST;
  if(fifo_counter>0){
    for( ; fifo_counter>0; fifo_counter--){
      tmp = SciaRegs.SCIRXBUF.bit.SAR;
      switch (_state) {
        case STATE_IDEL:
          if(tmp == SOF_PATTERN){
            _state = STATE_CANIDH;
          }
          break;
        case STATE_CANIDH:
          __byte_uint16_t(msg->CANID, 1) = tmp;
          _state = STATE_CANIDL;
          break;
        case STATE_CANIDL:
          __byte_uint16_t(msg->CANID, 0) = tmp;
          _state = STATE_LEN;
          break;
        case STATE_LEN:
          if(tmp>0 && tmp<12){
            data_length = tmp;
            data_counter = 0;
            msg->Length = tmp;
            _state = STATE_DATA;
          } else {
            //data length error
            _state = STATE_IDEL;
          }
          break;
        case STATE_DATA:
          __byte_uint16_t(msg->Data, data_counter++) = tmp;
          if(data_counter == data_length){
            _state = STATE_EOF;
          }
          break;
        case STATE_EOF:
          if(tmp == EOF_PATTERN){
            retval = 1;
          } else {
            // frame error
          }
          _state = STATE_IDEL;
          break;
      }
    }
  }

  return retval;
}

/**
 *  send CAN-compatible data frame over UART
 */
void UartDriver::SendMessage(CiA_Message * msg){

  SciaRegs.SCITXBUF.all = SOF_PATTERN;

  // transmit CANID
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->CANID, 1); // CANID high
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->CANID, 0); // CANID low

  // transmit CAN data
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 0);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 1);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 2);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 3);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 4);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 5);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 6);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 7);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 8);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 9);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 10);
  SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, 11);

  SciaRegs.SCITXBUF.all = EOF_PATTERN;
}
