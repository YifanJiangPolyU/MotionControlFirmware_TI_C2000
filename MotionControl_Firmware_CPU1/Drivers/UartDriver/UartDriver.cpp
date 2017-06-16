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
    // copy data from fifo to data buffer
    RawDataBuffer[0] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[1] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[2] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[3] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[4] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[5] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[6] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[7] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[8] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[9] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[10] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[11] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[12] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[13] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[14] = SciaRegs.SCIRXBUF.bit.SAR;
    RawDataBuffer[15] = SciaRegs.SCIRXBUF.bit.SAR;

  }



}
