/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* system warehouse
*
* initialize statically and hold all objects
*/

#ifndef SYSTEMWAREHOUSE_h
#define SYSTEMWAREHOUSE_h

#include "ControlProcessMaster.h"
#include "CommutationMaster.h"
#include "ObjectDictionary.h"

class SystemWarehouse{
  public:
    SystemWarehouse():
      _ControlProcessMaster(&_CommutationMaster, &_ObjectDictionary),
      _CommutationMaster(),
      _ObjectDictionary()
      {}

    ~SystemWarehouse(){}

  private:
    ControlProcessMaster _ControlProcessMaster;
    CommutationMaster _CommutationMaster;
    ObjectDictionary  _ObjectDictionary;
};

/**
 *  create globally unique object of CreateSystemWarehouseInstance
 *    call this ONLY ONCE
 */
void CreateSystemWarehouseInstance(void){

  #pragma DATA_SECTION("CPU1DataRAM")
  static SystemWarehouse SystemWarehouseInstance;
}

#endif
