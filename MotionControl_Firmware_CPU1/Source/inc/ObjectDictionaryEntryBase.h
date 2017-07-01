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
