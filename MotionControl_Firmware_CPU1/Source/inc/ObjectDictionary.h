/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* ObjectDictionary class
*
* implements object dictionary, which is compatible with CANOpen standard (CiA301)
* but is also extendable to make use of higher-BW communication methods
*/

#ifndef OBJECTDICTIONARY_H
#define OBJECTDICTIONARY_H

#include "stdint.h"
#include "F28x_Project.h"

#define OBJDICTIONARY_SIZE 200

typedef struct MessageType{

  uint32_t Control       : 8;
  uint32_t ObjIndex      : 16;
  uint32_t ObjSubIndex   : 8;
  uint16_t Data0[2];
  uint16_t Data1[2];

} msg;


class ObjectDictionary{

public:
  ObjectDictionary();
  ~ObjectDictionary();

private:



};


#endif
