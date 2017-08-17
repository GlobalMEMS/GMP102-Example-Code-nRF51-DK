/*
 *
 ****************************************************************************
 * Copyright (C) 2017 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : iir_filter.h
 *
 * Usage: IIR filter
 *
 ****************************************************************************
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

#ifndef __IIR_FILTER_H__
#define __IIR_FILTER_H__

#include <stdint.h>

//typedef float iirFlt_t;
typedef double iirFlt_t;

typedef struct{

  int32_t isFirst;  //filter startup
  int32_t dof;      //dof of the data vector
  int32_t order;    //filter order
  iirFlt_t *histX;  //size of dof*order, {x_n-1, x_n-2, ..., x_n-order}
  iirFlt_t *histY;  //size of dof*order, {y_n-1, y_n-2, ..., y_n-order)}
  iirFlt_t *coeffA; //size of order+1, {a0, a1, a2, a3, ..., a(order)}
  iirFlt_t *coeffB; //size of order+1, {b0, b1, b2, b3, ..., b(order)}

} iir_filter_param_t;

/*!
 * @brief Intialize IIR filter
 *
 * @param pParam Pointer to the IIR filter struct
 *
 * @return None
 */
void iirFilterInit(iir_filter_param_t *pParam);

/*!
 * @brief Filtering data
 *
 * @param pData_in data input to the filter
 * @param pData_out data output form the filter
 * @param pParam IIR filter parameter
 *
 * @return None
 */
void filterData(float *pData_in, float *pData_out, iir_filter_param_t *pParam);

/*!
 * @brief Filtering data
 *
 * @param pData_in data input to the filter
 * @param pData_out data output form the filter
 * @param pParam IIR filter parameter
 *
 * @return None
 */
void filterData_int16(int16_t *pData_in, int16_t *pData_out, iir_filter_param_t *pParam);

#endif //__IIR_FILTER_H__
