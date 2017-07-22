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

#include "std_msgs/String.h"
#include "mcs_interface/CiA_SdoMessage.h"
#include "mcs_interface/CiA_NmtMessage.h"

volatile bool terminate;

void SdoReplyCallback(const mcs_interface::CiA_SdoMessage::ConstPtr& msg){
  printf("got reply! \n");
  terminate = true;

  ObdAccessHandle handle;
  handle.Data.DataInt16[0] = msg->Data[0];
  handle.Data.DataInt16[1] = msg->Data[1];
  printf("sdo access successful, value: %f\n", handle.Data.DataFloat32 );
}

int main(int argc, char **argv){

  ros::init(argc, argv, "mcs_example");
  ros::NodeHandle node;

  terminate = false;

  // initialize publisher
  ros::Publisher sdo_pub = node.advertise<mcs_interface::CiA_SdoMessage>("SdoRequest", 20);
  ros::Publisher nmt_pub = node.advertise<mcs_interface::CiA_NmtMessage>("Nmt", 20);
  ros::Subscriber sdo_sub = node.subscribe("SdoReply", 5, SdoReplyCallback);

  ObdAccessHandle handle;
  handle.Data.DataFloat32 = 31.2;

  mcs_interface::CiA_SdoMessage msg;
  msg.Idx = 0x00A;
  msg.AccessType = SDO_CSS_READ;
  msg.AccessResult = 0;
  msg.Data[0] = 34; //handle.Data.DataInt16[0];
  msg.Data[1] = 43; //handle.Data.DataInt16[1];
  msg.Length = 10;

  usleep (1000000);
  //sdo_pub.publish(msg);

  mcs_interface::CiA_NmtMessage msg1;
  msg1.NodeID = 0x03;
  msg1.State = NMT_TEST_CLSW;
  nmt_pub.publish(msg1);

  while(ros::ok() && (!terminate)){

    ros::spinOnce();
  }
}
