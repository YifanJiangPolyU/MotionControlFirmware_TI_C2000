/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* commutation master class
*
* Contains functionalities related to commutation, including: a) initial
* commutation angle detection, b) initial polarity detection, and c) real-time
* commutation with encoder and/or hall sensors
*/

#ifndef COMMUTATION_MASTER_H
#define COMMUTATION_MASTER_H

#include "stdint.h"
#include "F28x_Project.h"

class CommutationMaster{

  public:
    CommutationMaster():
      NPolePair(1),
      CountPerRev(1024),
      HallSensorPattern(0b001),
      CommAnglePerCount(0.0f),
      CommAngle(0.0f),
      CommAngle_Cos(1.0f),
      CommAngle_Sin(0.0f)
    {};

    ~CommutationMaster(){};

    void Update(int32_t PosCounter);
    void UpdateCommResolution(void);

  private:
    int32_t PosCounter_new;           // position counter value
    int32_t PosCounter_old;

    uint16_t  NPolePair;              // number of pole pairs (rotary)
    uint16_t  CountPerRev;            // resolution: count per revolution (rotaty)

    uint16_t  HallSensorPattern;      // hall sensor pattern (used only when hall sensor available)
    float32_t CommAnglePerCount;      // electrical angle per count (rad)
    float32_t CommAngle;              // electrical angle (rad)
    float32_t CommAngle_Cos;          // cosine of electrical angle
    float32_t CommAngle_Sin;          // sine of electrical angle

};

#endif
