/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

 /**
  *  implement a node to execute current controllwe sweepsine test
  */

#include "stdio.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "errno.h"
#include "fcntl.h"
#include "termios.h"

#include "ros/ros.h"
#include "CANOpen/CANOpenDataTypeDef.h"

#include "std_msgs/String.h"
#include "mcs_interface/CiA_SdoMessage.h"
#include "mcs_interface/CiA_PdoMessage.h"
#include "mcs_interface/CiA_NmtMessage.h"


volatile bool terminate;
volatile bool ReceivedDataSize;
volatile bool ReceivedData;
volatile uint16_t PkgCounter;
volatile uint16_t NumberOfPkg;

void SdoReplyCallback(const mcs_interface::CiA_SdoMessage::ConstPtr& msg){

  NumberOfPkg = msg->Data[0];
  ReceivedDataSize = true;
}

void PdoCallback(const mcs_interface::CiA_PdoMessage::ConstPtr& msg){
  if(PkgCounter<NumberOfPkg){
    PkgCounter += 1;
  } else {
    ReceivedData = true;
  }
}

void RequestDataSize(ros::Publisher sdo_pub){

  mcs_interface::CiA_SdoMessage SdoMsg;
  SdoMsg.Idx = 0x00A;
  SdoMsg.AccessType = SDO_CSS_READ;
  SdoMsg.AccessResult = 0;
  SdoMsg.Data[0] = 0;
  SdoMsg.Data[1] = 0;
  SdoMsg.Length = 10;
  usleep (1000000);
  sdo_pub.publish(SdoMsg);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "mcs_clsw");
  ros::NodeHandle node;

  terminate = false;
  ReceivedDataSize = false;
  PkgCounter = 0;
  NumberOfPkg = 0;

  // initialize publishers and subscribers
  ros::Publisher nmt_pub = node.advertise<mcs_interface::CiA_NmtMessage>("Nmt", 20);
  ros::Publisher sdo_pub = node.advertise<mcs_interface::CiA_SdoMessage>("SdoRequest", 20);
  ros::Subscriber sdo_sub = node.subscribe("SdoReply", 5, SdoReplyCallback);
  ros::Subscriber pdo_sub = node.subscribe("Pdo", 100, PdoCallback);

  enum CLSW_STATE {
    STATE_ASK_CNT,
    STATE_GET_CNT,
    STATE_START,
    STATE_GET_DATA,
    STATE_ANALYZE,
    STATE_COMPLETE
  };

  enum CLSW_STATE _state = STATE_ASK_CNT;

  printf("Current Loop Sweepsine Test\n");

  while(ros::ok() && (!terminate)){

    switch (_state) {
      case STATE_ASK_CNT:
        printf("    Geting data size.\n");
        // send SDO request to ask for number of samples (data size)
        RequestDataSize(sdo_pub);
        ReceivedDataSize = false;
        _state = STATE_GET_CNT;
        break;
      case STATE_GET_CNT:
        // wait for the reception of SDO reply
        if(ReceivedDataSize==true){
          _state = STATE_START;
        }
        break;
      case STATE_START :
        printf("    Starting current loop sweepsine.\n");
        // send NMT to start CLSW process

        ReceivedData = false;
        _state = STATE_GET_DATA;
        printf("    Collecting data ...\n");
        break;
      case STATE_GET_DATA :
        // collect data from controller
        if(ReceivedData==true){
          _state = STATE_ANALYZE;
        }
        break;
      case STATE_ANALYZE :
        // analyze data
        break;
      case STATE_COMPLETE :
        // terminate
        printf("    Current loop sweepsine complete.\n");
        terminate = true;
        break;
      default :
        break;
    }

    ros::spinOnce();
  }
}
