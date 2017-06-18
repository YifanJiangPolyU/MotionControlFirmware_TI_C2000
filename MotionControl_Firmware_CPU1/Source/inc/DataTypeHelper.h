/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* helper functions for 8-bit data types
* C28x only support 16-bit and 32-bit data types
* 8-bit types are stored in the lower 8-bit of a 16-bit type
* this header provides functions that process data types
* of different lengths, for example:
*       merge 4 8-bit type into 1 32-bit type
*       merge 2 8-bit type into 1 16-bit type
*       merge 2 16-bit type into 1 32-bit type
*       access 8-bit types stored in 32-bit and 16-bit types
*
*/

#ifndef DATA_TYPE_HELPER_H
#define DATA_TYPE_HELPER_H

/**
 *  accessing 8-bit bytes in 16-bit type,
 *  a warper for uint16_t arrount __byte()
 */
#define __byte_uint16_t(x, y) (__byte((int *)&(x), y))

#endif
