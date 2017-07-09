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

#include "SystemWarehouse.h"

static SystemWarehouse * SystemWarehousePtr;

/**
 *  create globally unique object of CreateSystemWarehouseInstance
 *    call this ONLY ONCE
 */
void CreateSystemWarehouseInstance(void){

  #pragma DATA_SECTION("CPU1DataRAM")
  static SystemWarehouse SystemWarehouseInstance;
  SystemWarehousePtr = &SystemWarehouseInstance;
}


CurrentLoopController * SystemWarehouse::_CurrentLoopController_GetInstance(void){
  return &(SystemWarehousePtr->_CurrentLoopController);
}
