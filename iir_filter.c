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

#include "iir_filter.h"

/*!
 * @brief Intialize IIR filter
 *
 * @param pParam Pointer to the IIR filter struct
 *
 * @return None
 */
void iirFilterInit(iir_filter_param_t *pParam)
{

  int32_t i;

  pParam->isFirst = 1;

  //Initialize the history
  for(i = 0; i < (pParam->dof)*(pParam->order); ++i){
    pParam->histX[i] = 0.0;
		pParam->histY[i] = 0.0;
	}
}

/*!
 * @brief Filtering data
 *
 * @param pData_in data input to the filter
 * @param pData_out data output form the filter
 * @param pParam IIR filter parameter
 *
 * @return None
 */
void filterData(float *X_n, float *Y_n, iir_filter_param_t *pParam)
{

  int32_t i, j;
  int32_t i_histXY;
  int32_t lenAB = pParam->order + 1;
  int32_t colhXY = pParam->order;
	iirFlt_t iirX, iirY;
	
	//Startup, init history
	if(pParam->isFirst){
		
		for(i = 0; i < pParam->dof; ++i){
			
			i_histXY = i * colhXY;
			iirX = (iirFlt_t)X_n[i];
			
			for(j = i_histXY; j < i_histXY + colhXY; ++j){
				pParam->histX[j] = iirX;
				pParam->histY[j] = iirX;
			}			
		}
		
		pParam->isFirst = 0;
		
	}

  // Data filtering
  for(i = 0; i < pParam->dof; ++i){

    i_histXY = i * colhXY;
		iirX = (iirFlt_t)X_n[i];

    //
    // a0*y_n + a_1 * y_n-1 + a_2 * y_n-2 + ....
    //    = b_0 * x_n + b_1 * x_n-1 + b_2 * x_n-2 + .... +
    //       
    iirY = pParam->coeffB[0] * iirX;

    for(j = 1; j < lenAB; ++j){
      iirY += (pParam->coeffB[j] * pParam->histX[i_histXY + j - 1] - pParam->coeffA[j] * pParam->histY[i_histXY + j - 1]);
		}

    //iirY /= pParam->coeffA[0];  //a0 = 1 is assumed

    //Update the history value
    for(j = colhXY - 1; j > 0; --j){
      pParam->histX[i_histXY + j] = pParam->histX[i_histXY + j - 1];
      pParam->histY[i_histXY + j] = pParam->histY[i_histXY + j - 1];
		}
		pParam->histX[i_histXY] = iirX;
		pParam->histY[i_histXY] = iirY;
		
		//filtered output
		Y_n[i] = iirY;
  }
}

/*!
 * @brief Filtering data
 *
 * @param pData_in data input to the filter
 * @param pData_out data output form the filter
 * @param pParam IIR filter parameter
 *
 * @return None
 */
void filterData_int16(int16_t *pData_in, int16_t *pData_out, iir_filter_param_t *pParam)
{

  float X_n[3];
	float Y_n[3];
  int i;

  for(i = 0; i < pParam->dof; ++i){
    X_n[i] = pData_in[i];
  }
  filterData(X_n, Y_n, pParam);

  for(i = 0; i < pParam->dof; ++i){
    pData_out[i] = (int16_t)(Y_n[i] + 0.5);
  }

}
