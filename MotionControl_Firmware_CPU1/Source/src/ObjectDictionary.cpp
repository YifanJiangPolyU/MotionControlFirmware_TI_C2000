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
* implements object dictionary, which is compatible with CANOpen standard (CiA304)
* but is also extendable to make use of higher-BW communication methods
*/

#include "ObjectDictionary.h"

/**
 *  search for the position of obj in the array, using binary search
 *  @param Idx       index of target object
 *  @param SubIdx    sub-index of the target object
 *  @retval          position of the object in the array
 *                   return 0xFFFF if not found.
 */
#pragma CODE_SECTION(".TI.ramfunc")
int16_t ObjectDictionary::SearchEntry(uint16_t Idx, uint16_t SubIdx){
  uint32_t target = (Idx<<8) | SubIdx;
  int16_t retval = -1;
  int16_t head = 0;
  int16_t tail = MAX_NO_OF_ENTRY-1;
  int16_t mid = 0;
  bool terminate = false;

  while(!terminate){
    mid = (head + tail)/2;
    if(target==_IdxArray[mid]){
      retval = mid;
      terminate = true;
    }else if(target>_IdxArray[mid]){
      head = mid+1;
    }else if(target<_IdxArray[mid]){
      tail = mid-1;
    }

    // found nothing
    if(head>tail){
      terminate = true;
    }
  }

  return retval;
}
