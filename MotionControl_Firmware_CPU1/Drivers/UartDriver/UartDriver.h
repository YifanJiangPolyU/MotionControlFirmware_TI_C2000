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

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "stdint.h"
#include "string.h"
#include "F28x_Project.h"
#include "CiATypeDef.h"
#include "DataTypeHelper.h"

// start-of-frame and end-of-frame patterns
#define SOF_PATTERN 0xEF
#define EOF_PATTERN 0xDB

#define MSG_BUFFER_SIZE 5

class UartDriver{
  public:
    UartDriver();
    ~UartDriver();

    void ExecuteParsing(void);
    void SendMessage(CiA_Message * msg);
    void GetMessage(CiA_Message * msg);
    inline uint16_t GetBufferLevel();

  private:
    uint16_t SOF_EOF;
    uint16_t RawDataBuffer[7];
    CiA_Message MessageBuffer[MSG_BUFFER_SIZE+1];

    uint16_t MessageBuffer_ReadIdx;
    uint16_t MessageBuffer_WriteIdx;
    uint16_t MessageBuffer_Level;
};

#endif
