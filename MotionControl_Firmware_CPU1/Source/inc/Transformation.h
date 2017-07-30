/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement functions for clark and park transformations
*/

#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include "ControlTypeDef.h"

void ClarkTransformation(ABCVec * in, AlBeVec * out);
void InvClarkTransformation(AlBeVec * in, ABCVec * out);

void ParkTransformation(AlBeVec * in, DQVec * out, RotationFrame * frame);
void InvParkTransformation(DQVec * in, AlBeVec * out, RotationFrame * frame);

#endif
