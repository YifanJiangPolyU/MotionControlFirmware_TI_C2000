/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* Control process base class
* control processes to be executed by the CPU should be derived from this one
*/

#ifndef _OBJ_ENTRY_BASE_H
#define _OBJ_ENTRY_BASE_H

#include "stdint.h"
#include "CiATypeDef.h"

#define OBD_ACCESS_SUCCESS                 0x00  // access successful
#define OBD_ACCESS_ERR_READ                0x01  // access failed, trying to read a non-readable object
#define OBD_ACCESS_ERR_WRITE               0x02  // access failed, trying to write to a read-only object
#define OBD_ACCESS_ERR_IDX_NONEXIST        0x03  // access failed, index does not exist
#define OBD_ACCESS_ERR_SUBIDX_NONEXIST     0x04  // access failed, subindex does not exist

class ObjectDictionaryEntryBase{

  public:

    ObjectDictionaryEntryBase(){};
    ~ObjectDictionaryEntryBase(){};

    virtual void AccessObject(){};

  protected:
    bool _ProcessCompleted;

  private:

};

#endif
