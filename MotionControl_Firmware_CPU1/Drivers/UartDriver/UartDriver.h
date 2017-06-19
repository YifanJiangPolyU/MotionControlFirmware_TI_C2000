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

#define MSG_BUFFER_SIZE            5
#define MSG_BUFFER_SIZE_MINUS_ONE  4

class UartDriver{
  public:
    UartDriver();
    ~UartDriver();

    uint16_t ExecuteParsing(CiA_Message * msg);
    void SendMessage(CiA_Message * msg);

    enum MessageParser_STATES {
      STATE_IDEL,
      STATE_LEN,
      STATE_CANIDH,
      STATE_CANIDL,
      STATE_DATA,
      STATE_EOF
    };

  private:

    enum MessageParser_STATES _state;

};

#endif
