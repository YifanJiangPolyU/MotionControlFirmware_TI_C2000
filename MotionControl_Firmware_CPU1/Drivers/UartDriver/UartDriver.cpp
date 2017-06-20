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

  static uint16_t fifo_counter = 0;
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
          __byte_uint16_t(msg_buffer.CANID, 1) = tmp;
          _state = STATE_CANIDL;
          break;
        case STATE_CANIDL:
          __byte_uint16_t(msg_buffer.CANID, 0) = tmp;
          _state = STATE_LEN;
          break;
        case STATE_LEN:
          if(tmp>0 && tmp<12){
            data_length = tmp;
            data_counter = 0;
            msg_buffer.Length = tmp;
            _state = STATE_DATA;
          } else {
            //data length error
            _state = STATE_IDEL;
          }
          break;
        case STATE_DATA:
          __byte_uint16_t(msg_buffer.Data, data_counter++) = tmp;
          if(data_counter == data_length){
            _state = STATE_EOF;
          }
          break;
        case STATE_EOF:
          if(tmp == EOF_PATTERN){
            retval = 1;
            memcpy(msg, &msg_buffer, sizeof(CiA_Message));
          } else {
            // frame error
            GpioDataRegs.GPBDAT.bit.GPIO34 = 0;
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
 * @param msg   ptr to message to be sent
 * @retval      1 if sent successfully, 0 if tx buffer too full
 */
#pragma CODE_SECTION(".TI.ramfunc");
uint16_t UartDriver::SendMessage(CiA_Message * msg){

  uint16_t retval = 0;
  uint16_t tx_length = msg->Length;
  uint16_t tx_counter = 0;

  if((11-SciaRegs.SCIFFTX.bit.TXFFST)>=tx_length){

    SciaRegs.SCITXBUF.all = SOF_PATTERN;

    // transmit CANID
    SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->CANID, 1); // CANID high
    SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->CANID, 0); // CANID low

    // transmit CAN data length
    SciaRegs.SCITXBUF.all = tx_length;

    // transmit CAN data
    for(tx_counter=0; tx_counter<tx_length; tx_counter++){
      SciaRegs.SCITXBUF.all = __byte_uint16_t(msg->Data, tx_counter);
    }

    SciaRegs.SCITXBUF.all = EOF_PATTERN;

    retval = 1;
  } else {
    // TX buffer too full
    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;
  }

  return retval;
}
