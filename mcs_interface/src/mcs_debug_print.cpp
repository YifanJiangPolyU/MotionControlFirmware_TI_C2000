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


const float StartFrequency = 1000; // Hz
const float EndFrequency = 5000;
const float RampRate = 4000;
const float Amplitude = 1000;
const uint8_t Phase = 'C';

const float PI = 3.141592653589793116f;

enum CLSW_STATE {
  STATE_START,
  STATE_CALIBRATE,
  STATE_COMPLETE
};

enum CLSW_STATE _state;

volatile bool terminate;
volatile bool ReadyFlag;
volatile bool DataRxComplete;
volatile uint32_t PkgCounter;
volatile uint32_t NumberOfPkg;

ros::Publisher nmt_pub;
ros::Subscriber pdo_sub;

void PdoCallback(const mcs_interface::CiA_PdoMessage::ConstPtr& msg);

void StartCalibration(void);

int main(int argc, char **argv){

  ros::init(argc, argv, "mcs_calibrate");
  ros::NodeHandle node;


  // initialize publishers and subscribers
  nmt_pub = node.advertise<mcs_interface::CiA_NmtMessage>("Nmt", 20);
  pdo_sub = node.subscribe("Pdo", 100, PdoCallback);
  usleep(1000000);

  _state = STATE_START;

  printf("Current Loop Sweepsine Test\n");

  while(ros::ok() && (!terminate)){

    switch (_state) {
      case STATE_START:
        _state = STATE_COMPLETE;
        StartCalibration();
        break;
      case STATE_COMPLETE :
        // terminate
        printf("    Calibration Command Sent.\n");
        terminate = true;
        break;
      default :
        break;
    }

    ros::spinOnce();
  }
}



void StartCalibration(void){
  mcs_interface::CiA_NmtMessage msg1;
  msg1.NodeID = 0x03;
  msg1.State = NMT_TEST_CALIBRATION;
  nmt_pub.publish(msg1);
}


void PdoCallback(const mcs_interface::CiA_PdoMessage::ConstPtr& msg){
  PdoData data;
  SystemStatusReg status;

  data.data[0] = msg->Data[0];
  data.data[1] = msg->Data[1];
  data.data[2] = msg->Data[2];
  data.data[3] = msg->Data[3];
  data.data[4] = msg->Data[4];

  status.all = data.clsw.StatusReg;
  printf("status: %d\n", status.bit.State);

}
