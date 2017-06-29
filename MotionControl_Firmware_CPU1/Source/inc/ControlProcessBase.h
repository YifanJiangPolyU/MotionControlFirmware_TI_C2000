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

#ifndef _CONTROL_PROCESS_BASE_H
#define _CONTROL_PROCESS_BASE_H

#include "stdint.h"

class ControlProcessBase{

  public:

    ControlProcessBase(){};
    ~ControlProcessBase(){};

    virtual void Execute(void){};
    virtual void Reset(void){};

  protected:
    bool _ProcessCompleted;

  private:

};

#endif
