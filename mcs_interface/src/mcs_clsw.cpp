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
#include "iostream"
#include "fstream"
#include "sstream"
#include "string"

#include "ros/ros.h"
#include "CANOpen/CANOpenDataTypeDef.h"
#include "CANOpen/PdoTypeDef.h"

#include "std_msgs/String.h"
#include "mcs_interface/CiA_SdoMessage.h"
#include "mcs_interface/CiA_PdoMessage.h"
#include "mcs_interface/CiA_NmtMessage.h"


const float StartFrequency = 50;
const float EndFrequency = 5000;
const float RampRate = 4950;

enum CLSW_STATE {
  STATE_SET_FSTART,
  STATE_SET_FEND,
  STATE_SET_RATE,
  STATE_ASK_CNT,
  STATE_GET_CNT,
  STATE_START,
  STATE_GET_DATA,
  STATE_ANALYZE,
  STATE_COMPLETE
};

enum CLSW_STATE _state;

volatile bool terminate;
volatile bool SetStartFrequencySuccess;
volatile bool SetEndFrequencySuccess;
volatile bool SetRampRateSuccess;
volatile bool ReceivedDataSize;
volatile bool DataRxComplete;
volatile uint32_t PkgCounter;
volatile uint32_t NumberOfPkg;

ros::Publisher nmt_pub;
ros::Publisher sdo_pub;
ros::Subscriber sdo_sub;
ros::Subscriber pdo_sub;

std::ofstream OutputFile;


void SdoReplyCallback(const mcs_interface::CiA_SdoMessage::ConstPtr& msg);
void PdoCallback(const mcs_interface::CiA_PdoMessage::ConstPtr& msg);
void SetStartFrequency(void);
void SetEndFrequency(void);
void SetRampRate(void);
void RequestDataSize(void);

int main(int argc, char **argv){

  ros::init(argc, argv, "mcs_clsw");
  ros::NodeHandle node;

  terminate = false;
  SetStartFrequencySuccess = false;
  SetEndFrequencySuccess = false;
  SetRampRateSuccess = false;
  ReceivedDataSize = false;
  DataRxComplete = false;
  PkgCounter = 0;
  NumberOfPkg = 0;

  // initialize publishers and subscribers
  nmt_pub = node.advertise<mcs_interface::CiA_NmtMessage>("Nmt", 20);
  sdo_pub = node.advertise<mcs_interface::CiA_SdoMessage>("SdoRequest", 20);
  sdo_sub = node.subscribe("SdoReply", 5, SdoReplyCallback);
  pdo_sub = node.subscribe("Pdo", 100, PdoCallback);
  usleep(1000000);

  OutputFile.open("/home/yifan/catkin_ws/src/mcs/mcs_interface/SweepSineData.txt",
                    std::ofstream::out | std::ofstream::trunc);


  _state = STATE_SET_FSTART;

  printf("Current Loop Sweepsine Test\n");

  while(ros::ok() && (!terminate)){

    switch (_state) {
      case STATE_SET_FSTART:
        printf("    Setting start frequency.\n");
        SetStartFrequency();
        _state = STATE_SET_FEND;
        break;
      case STATE_SET_FEND:
        if(SetStartFrequencySuccess==true){
          printf("    Setting end frequency.\n");
          SetEndFrequency();
          _state = STATE_SET_RATE;
        }
        break;
      case STATE_SET_RATE:
        if(SetEndFrequencySuccess==true){
          printf("    Setting frequency ramp rate.\n");
          SetRampRate();
          _state = STATE_ASK_CNT;
        }
        break;
      case STATE_ASK_CNT:
        if(SetRampRateSuccess==true){
          // send SDO request to ask for number of samples (data size)
          printf("    Geting data size.\n");
          RequestDataSize();
          ReceivedDataSize = false;
          _state = STATE_GET_CNT;
        }
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

        DataRxComplete = false;
        _state = STATE_GET_DATA;
        printf("    Collecting data ...\n");
        break;
      case STATE_GET_DATA :
        // collect data from controller
        if(DataRxComplete==true){
          _state = STATE_ANALYZE;
        }
        break;
      case STATE_ANALYZE :
        // analyze data
        break;
      case STATE_COMPLETE :
        // terminate
        printf("    Current loop sweepsine complete.\n");
        OutputFile.close();
        terminate = true;
        break;
      default :
        break;
    }

    ros::spinOnce();
  }
}

void SetStartFrequency(void){
  mcs_interface::CiA_SdoMessage SdoMsg;
  ObdAccessHandle handle;
  handle.Data.DataFloat32 = StartFrequency;

  SdoMsg.Idx = 0x2106;
  SdoMsg.SubIdx = 0x01;
  SdoMsg.AccessType = SDO_CSS_WRITE;
  SdoMsg.AccessResult = 0;
  SdoMsg.Data[0] = handle.Data.DataUint16[0];
  SdoMsg.Data[1] = handle.Data.DataUint16[1];
  SdoMsg.Length = 10;
  sdo_pub.publish(SdoMsg);
}

void SetEndFrequency(void){
  mcs_interface::CiA_SdoMessage SdoMsg;
  ObdAccessHandle handle;
  handle.Data.DataFloat32 = EndFrequency;

  SdoMsg.Idx = 0x2106;
  SdoMsg.SubIdx = 0x02;
  SdoMsg.AccessType = SDO_CSS_WRITE;
  SdoMsg.AccessResult = 0;
  SdoMsg.Data[0] = handle.Data.DataUint16[0];
  SdoMsg.Data[1] = handle.Data.DataUint16[1];
  SdoMsg.Length = 10;
  sdo_pub.publish(SdoMsg);
}

void SetRampRate(void){
  mcs_interface::CiA_SdoMessage SdoMsg;
  ObdAccessHandle handle;
  handle.Data.DataFloat32 = RampRate;

  SdoMsg.Idx = 0x2106;
  SdoMsg.SubIdx = 0x03;
  SdoMsg.AccessType = SDO_CSS_WRITE;
  SdoMsg.AccessResult = 0;
  SdoMsg.Data[0] = handle.Data.DataUint16[0];
  SdoMsg.Data[1] = handle.Data.DataUint16[1];
  SdoMsg.Length = 10;
  sdo_pub.publish(SdoMsg);
}

void RequestDataSize(void){
  mcs_interface::CiA_SdoMessage SdoMsg;
  SdoMsg.Idx = 0x2106;
  SdoMsg.SubIdx = 0x05;
  SdoMsg.AccessType = SDO_CSS_READ;
  SdoMsg.AccessResult = 0;
  SdoMsg.Data[0] = 0;
  SdoMsg.Data[1] = 0;
  SdoMsg.Length = 10;
  sdo_pub.publish(SdoMsg);
}


void PdoCallback(const mcs_interface::CiA_PdoMessage::ConstPtr& msg){
  PdoData data;
  if(PkgCounter<NumberOfPkg){
    if(msg->PDO_ID==PDO_ID_CLSW){
      PkgCounter += 1;
      data.data[0] = msg->Data[0];
      data.data[1] = msg->Data[1];
      data.data[2] = msg->Data[2];
      data.data[3] = msg->Data[3];
      data.data[4] = msg->Data[4];

      OutputFile << data.clsw.CurrentActual[0] << std::endl;
      OutputFile << data.clsw.CurrentActual[1] << std::endl;
      OutputFile << data.clsw.CurrentActual[2] << std::endl;
      OutputFile << data.clsw.CurrentActual[3] << std::endl;
    }
  } else {
    DataRxComplete = true;
  }
}


void SdoReplyCallback(const mcs_interface::CiA_SdoMessage::ConstPtr& msg){
  ObdAccessHandle handle;
  handle.Data.DataInt16[0] = msg->Data[0];
  handle.Data.DataInt16[1] = msg->Data[1];
  handle.AccessResult = msg->AccessResult;

  switch (_state) {
    case STATE_SET_FEND:
      if(handle.AccessResult==0){
        SetStartFrequencySuccess = true;
      } else {
        printf("    Failed to set start frequency\n");
        terminate = true;
      }
      break;
    case STATE_SET_RATE:
      if(handle.AccessResult==0){
        SetEndFrequencySuccess = true;
      } else {
        printf("    Failed to set end frequency\n");
        terminate = true;
      }
      break;
    case STATE_ASK_CNT:
      if(handle.AccessResult==0){
        SetRampRateSuccess = true;
      } else {
        printf("    Failed to set frequency ramp rate\n");
        terminate = true;
      }
      break;
    case STATE_GET_CNT:
      NumberOfPkg = handle.Data.DataUint32;
      ReceivedDataSize = true;
      if(NumberOfPkg == 0xFFFFFFFF){
        printf("    Wrong Parameters: is starting frequency higher than end frequency?\n");
        printf("Terminating..\n");
        terminate = true;
      }
      break;
    default:
      break;
  }




}
