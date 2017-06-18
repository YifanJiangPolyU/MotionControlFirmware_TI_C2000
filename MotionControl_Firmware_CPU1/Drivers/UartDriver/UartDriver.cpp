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

static UartDriver * This;

UartDriver::UartDriver(){
  This = this;
  MessageBuffer_ReadIdx = 0;
  MessageBuffer_WriteIdx = 0;
}

UartDriver::~UartDriver(){

}

/**
 *  C warper to call UartDriver from ISR
 */
#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void CallUartDriverExecuteParsing(void){
  This->ExecuteParsing();
}

/**
 *  extract CiA message from UART data
 *  data pkg has to be 16-byte long, and aligned to
 *  the 16-byte SCI Rx FIFO.
 */
#pragma CODE_SECTION(".TI.ramfunc");
void UartDriver::ExecuteParsing(void){

  // check for error
  if(SciaRegs.SCIRXST.bit.RXERROR){
    // turn on red LED to indicate error
    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;

  } else {
    __byte_uint16_t(SOF_EOF, 0) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 0) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 1) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 2) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 3) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 4) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 5) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 6) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 7) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 8) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 9) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 10) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 11) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 12) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 13) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(RawDataBuffer[0], 14) = SciaRegs.SCIRXBUF.bit.SAR;
    __byte_uint16_t(SOF_EOF, 1) = SciaRegs.SCIRXBUF.bit.SAR;

    // simple checking to ensure we've got a complete frame
    if( (__byte_uint16_t(SOF_EOF, 0) == SOF_PATTERN) &&
        (__byte_uint16_t(SOF_EOF, 1) == EOF_PATTERN) ){

        }

  }

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

/**
 *  get the next message
 *  @param buffer: buffer to hold message
 *                 doesn't change anything if no new message available
 */
#pragma CODE_SECTION(".TI.ramfunc");
void UartDriver::GetMessage(CiA_Message * buffer){

}

/**
 *  get the number of messages in buffer
 *  @retval number of messages in buffer
 */
inline uint16_t UartDriver::GetBufferLevel(){
  return MessageBuffer_Level;
}
