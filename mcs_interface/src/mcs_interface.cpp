/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

 /**
  *  implement the mcs_interface node
  */

#include "stdio.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "errno.h"
#include "fcntl.h"
#include "termios.h"

#include "ros/ros.h"
#include "CANOpen/CANOpenDataTypeDef.h"
#include "CANOpen/PdoTypeDef.h"

#include "std_msgs/String.h"
#include "mcs_interface/CiA_SdoMessage.h"
#include "mcs_interface/CiA_PdoMessage.h"
#include "mcs_interface/CiA_NmtMessage.h"

// function prototypes
void InitSerialPort(void);
void SerialPoll(void);
uint8_t CANOpenMsgToSendBuffer(uint8_t * buffer, CiA_Message * msg);
bool Send(uint8_t* data, uint8_t len);
void ProcessMessage(CiA_Message* msg);
void ProcessSdoMessage(CiA_Message* msg);
void ProcessPdoMessage(CiA_Message* msg);

void SdoRequestCallback(const mcs_interface::CiA_SdoMessage::ConstPtr& msg);
void PdoRequestCallback(const mcs_interface::CiA_PdoMessage::ConstPtr& msg);
void NmtRequestCallback(const mcs_interface::CiA_NmtMessage::ConstPtr& msg);

// globally define publisher
ros::Publisher SdoReply_pub;
ros::Publisher Pdo_pub;
ros::Subscriber SdoRequest_sub;
ros::Subscriber Pdo_sub;
ros::Subscriber Nmt_sub;
ros::Subscriber Sync_sub;

// global uart R/W handle
int uart_handle;

int main(int argc, char **argv){

  ros::init(argc, argv, "mcs_interface");
  ros::NodeHandle node;

  // initialize publisher
  SdoReply_pub = node.advertise<mcs_interface::CiA_SdoMessage>("SdoReply", 20);
  Pdo_pub = node.advertise<mcs_interface::CiA_PdoMessage>("Pdo", 20);

  // initialize subscriber
  SdoRequest_sub = node.subscribe("SdoRequest", 5, SdoRequestCallback);
  Pdo_sub = node.subscribe("Pdo", 20, PdoRequestCallback);
  Nmt_sub = node.subscribe("Nmt", 5, NmtRequestCallback);
  Sync_sub = node.subscribe("Sync", 5, SdoRequestCallback);

  InitSerialPort();

  while(ros::ok()){
    SerialPoll();
    ros::spinOnce();
    usleep (100);
  }
}

/**
 *  callback function to transmit a SDO message over uart
 */
void SdoRequestCallback(const mcs_interface::CiA_SdoMessage::ConstPtr& msg){

  printf("recevied request\n");
  CiA_Message msg_buf;
  uint8_t send_buffer[16];
  uint8_t len;

  msg_buf.Sdo.CANID = NODE_ID + CANID_SDO_RX;
  msg_buf.Sdo.SdoIdx = (uint16_t)((msg->Idx)>>8);
  msg_buf.Sdo.SdoSubIdx = (uint8_t)((msg->Idx) & 0x000000FF);
  msg_buf.Sdo.SdoCtrl_ccs = msg->AccessType;
  msg_buf.Sdo.SdoAccessResult = 0;
  msg_buf.Sdo.Data[0] = msg->Data[0];
  msg_buf.Sdo.Data[1] = msg->Data[1];
  msg_buf.Sdo.Length = msg->Length;

  len = CANOpenMsgToSendBuffer(send_buffer, &msg_buf);
  Send(send_buffer, len);
}

/**
 *  callback function to transmit a PDO message over uart
 */
void PdoRequestCallback(const mcs_interface::CiA_PdoMessage::ConstPtr& msg){

}

/**
 *  callback function to transmit a NMT message over uart
 */
void NmtRequestCallback(const mcs_interface::CiA_NmtMessage::ConstPtr& msg){

  printf("recevied request\n");
  CiA_Message msg_buf;
  uint8_t send_buffer[16];
  uint8_t len;

  msg_buf.Nmt.CANID = CANID_NMT;
  msg_buf.Nmt.State = msg->State;
  msg_buf.Nmt.NodeID = msg->NodeID;
  msg_buf.Nmt.Length = 2;

  len = CANOpenMsgToSendBuffer(send_buffer, &msg_buf);
  Send(send_buffer, len);
}

/**
 *  Copy CiA_Message into linear uint8_t array, ready to be transmitted
 *  over UART.
 *  @param buffer     ptr to buffer
 *  @param msg        ptr to CiA_Message msg to be transmitted
 *  @retval           length of data in the buffer; 0 if error occurs.
 */
uint8_t CANOpenMsgToSendBuffer(uint8_t * buffer, CiA_Message * msg){

  uint8_t retval = 0;

  buffer[0] = CIA_SOF_PATTERN;                // start of frame

  buffer[1] = (msg->Common.CANID)>>8;         // CANID higher 8 bits
  buffer[2] = (msg->Common.CANID) & 0x0F;     // CANID lower 8 bits

  uint8_t Length = msg->Common.Length;
  buffer[3] = Length;

  uint8_t i;
  if(Length<=11){
    retval = Length + 5;
    for(i=0; i<Length; i++){
      buffer[i+4] = msg->Common.Data[i];
    }
  } else {
    // data length error
    retval = 0;
  }

  buffer[i+4] = CIA_EOF_PATTERN;        // end of frame
  return retval;
}

/**
 *  Sending data over uart
 *  @param data      ptr to data to be sent
 *  @param len       length of data (number of bytes)
 *  @retval          true if successfull, false if not
 */
bool Send(uint8_t * data, uint8_t len){

  // send package over serial port
  // modify to match your platform
  int count = write(uart_handle, data, len);

  // error
  // modify to match your platform
  if (count < 0){
    return false;
  }
  return true;
}

/**
 *  polling the serial port
 *  @param pub       ros::Publisher to which to publish the msg
 */
void SerialPoll(void){

  enum MessageParser_STATES {
        STATE_IDEL,
        STATE_LEN,
        STATE_CANIDH,
        STATE_CANIDL,
        STATE_DATA,
        STATE_EOF
      };

  static uint16_t fifo_counter = 0;
  static uint16_t data_counter = 0;
  static uint16_t data_length = 0;
  static enum MessageParser_STATES _state = STATE_IDEL;

  static uint16_t CANID;
  static CiA_Message MsgBuffer;
  static uint8_t len = 0;

  uint8_t buf[255];
  int j = 0;

    //try reading
    len = read(uart_handle, buf, 255);
    if(len>0){
        // successfully read data, start processing
        for(j=0; j<len; j++){
          uint8_t tmp = buf[j];
          switch (_state) {
            case STATE_IDEL:
              if(tmp == CIA_SOF_PATTERN){
                _state = STATE_CANIDH;
              }
              break;
          case STATE_CANIDH:
            CANID = tmp<<8;
            _state = STATE_CANIDL;
            break;
          case STATE_CANIDL:
            CANID |= tmp;
            MsgBuffer.Common.CANID = CANID;
            _state = STATE_LEN;
            break;
          case STATE_LEN:
            if(tmp>0 && tmp<=12){
              data_length = tmp;
              data_counter = 0;
              MsgBuffer.Common.Length = data_length;
              _state = STATE_DATA;
            } else {
             //data length error
             printf("SerialPoll: data length error.\n");
              _state = STATE_IDEL;
            }
            break;
          case STATE_DATA:
            MsgBuffer.Common.Data[data_counter++] = tmp;
            if(data_counter == data_length){
              _state = STATE_EOF;
            }
            break;
          case STATE_EOF:
            if(tmp == CIA_EOF_PATTERN){
              ProcessMessage(&MsgBuffer);
            } else {
              // frame error
              printf("SerialPoll: data framing error.\n");
            }
            _state = STATE_IDEL;
          break;
        }
      }
    }
}

/**
 *  publish received msgs to corresponding topics
 *  @param msg       ptr to received CiA message
 */
void ProcessMessage(CiA_Message * msg){

  if((msg->Common.CANID-NODE_ID)==CANID_SDO_TX){
    ProcessSdoMessage(msg);
  } else if((msg->Common.CANID-NODE_ID)==CANID_PDO_TX){
    ProcessPdoMessage(msg);
  }
}

/**
 *  process received sdo message
 *  @param msg       ptr to received CiA sdo message
 */
void ProcessSdoMessage(CiA_Message* msg){

  mcs_interface::CiA_SdoMessage msg_pub;
  ObdAccessHandle handle;

  msg_pub.Idx = (msg->Sdo.SdoIdx<<8)|(msg->Sdo.SdoSubIdx);
  msg_pub.AccessType = msg->Sdo.SdoCtrl_ccs;
  msg_pub.AccessResult = msg->Sdo.SdoAccessResult;
  msg_pub.Data[0] = msg->Sdo.Data[0];
  msg_pub.Data[1] = msg->Sdo.Data[1];

  handle.AccessResult = msg->Sdo.SdoAccessResult;
  handle.Data.DataInt16[0] = msg->Sdo.Data[0];
  handle.Data.DataInt16[1] = msg->Sdo.Data[1];

  SdoReply_pub.publish(msg_pub);

  if(handle.AccessResult==0){
    printf("sdo access successful, value: %f\n", handle.Data.DataFloat32 );
  }else{
    printf("sdo access failed: %d\n", handle.AccessResult);
  }

}

/**
 *  process received sdo message
 *  @param msg       ptr to received CiA sdo message
 */
void ProcessPdoMessage(CiA_Message* msg){

  mcs_interface::CiA_PdoMessage msg_pub;

  for(int i=0; i<5; i++){
    msg_pub.Data[i] = msg->Pdo.Data[i];
  }

  msg_pub.PDO_ID = msg->Pdo.PDO_ID;
  Pdo_pub.publish(msg_pub);

}

/**
 *  initialize serial port
 */
void InitSerialPort(void){

  char _port[100];
  strcpy(&(_port[0]), "/dev/ttyUSB0");
  // if(argc == 2){
  //   strcpy(&(_port[0]), (const char*)(argv[1]));
  // }

  struct termios options;
  uart_handle = open((const char*)(_port), O_RDWR | O_NOCTTY | O_NDELAY);
  tcgetattr(uart_handle, &options);
  options.c_cflag = B2000000 | CS8 | CLOCAL | CREAD;  //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  options.c_cc[VMIN] = 0;     // minimum char packing
  options.c_cc[VTIME] = 0;  // read timeout: 10s
  tcflush(uart_handle, TCIFLUSH);
  tcsetattr(uart_handle, TCSANOW, &options);
}
