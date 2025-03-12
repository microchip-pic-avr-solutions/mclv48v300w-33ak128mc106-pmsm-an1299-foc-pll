// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file measure.c
 *
 * @brief This module has functions for signal conditioning of measured
 *        analog feedback signals.
 *
 * Component: MEASURE
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "measure.h"

// </editor-fold>

/**
* <B> Function: MCAPP_MeasureCurrentInit(MCAPP_MEASURE_CURRENT_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*        
*
* @param Pointer to the data structure containing measured currents.
* @return none.
*
* @example
* <CODE> MCAPP_MeasureCurrentInit(&current); </CODE>
*
*/
void MCAPP_MeasureCurrentInit(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->measureCurrent;
    
    pCurrent->counter = 0;
    pCurrent->sumIa = 0;
    pCurrent->sumIb = 0;
    pCurrent->sumIbus = 0;
    pCurrent->status = 0;
}

/**
* <B> Function: MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_CURRENT_T *)  </B>
*
* @brief Function to compute current offset after measuring specified number of
*        current samples and averaging them.
*        
* @param Pointer to the data structure containing measured current.
* @return none.
* 
* @example
* <CODE> MCAPP_MeasureCurrentOffset(&current); </CODE>
*
*/
void MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->measureCurrent;
    
    pCurrent->sumIa += pCurrent->Ia;
    pCurrent->sumIb += pCurrent->Ib;
    pCurrent->sumIbus += pCurrent->Ibus;
    pCurrent->counter++;

    if (pCurrent->counter >= OFFSET_COUNT_MAX)
    {
        pCurrent->offsetIa = (int16_t)(pCurrent->sumIa >> OFFSET_COUNT_BITS);
        pCurrent->offsetIb = (int16_t)(pCurrent->sumIb >> OFFSET_COUNT_BITS);
        pCurrent->offsetIbus =
            (int16_t)(pCurrent->sumIbus >> OFFSET_COUNT_BITS);

        pCurrent->counter = 0;
        pCurrent->sumIa = 0;
        pCurrent->sumIb = 0;
        pCurrent->sumIbus = 0;
        pCurrent->status = 1;
    }

}
/**
* <B> Function: MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *,SINGLE_SHUNT_PARM_T *)  </B>
*
* @brief Function to compensate offset from measured current samples and convert 
* to actual values from ADc counts.
*        
* @param Pointer to the data structure containing measured current.
* @return none.
* 
* @example
* <CODE> MCAPP_MeasureCurrentCalibrate(pMotorInputs,pSingleShunt); </CODE>
*
*/
void MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *pMotorInputs, SINGLE_SHUNT_PARM_T *pSingleShunt)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->measureCurrent;
    
    /* offset correction*/
    pCurrent->Ia = pCurrent->Ia - pCurrent->offsetIa;
    pCurrent->Ib = pCurrent->Ib  - pCurrent->offsetIb;
    pCurrent->Ibus = pCurrent->Ibus - pCurrent->offsetIbus;
    
    /*Convert ADC Counts to real value*/
    pCurrent->Ia_actual = (float)(pCurrent->Ia * ADC_CURRENT_SCALE);
    pCurrent->Ib_actual = (float)(pCurrent->Ib * ADC_CURRENT_SCALE);
    /* Ia + Ib + Ic  = 0; in a balanced 3-phase system*/
    pCurrent->Ic_actual = -pCurrent->Ia_actual - pCurrent->Ib_actual; 
    
#ifdef SINGLE_SHUNT
    /* offset correction*/
    pCurrent->Ibus1 = pCurrent->Ibus1 - pCurrent->offsetIbus;
    pCurrent->Ibus2 = pCurrent->Ibus2 - pCurrent->offsetIbus;
    /*Convert ADC Counts to real value*/
    pSingleShunt->Ibus1 = (float)(pCurrent->Ibus1* ADC_CURRENT_SCALE);
    pSingleShunt->Ibus2 = (float)(pCurrent->Ibus2* ADC_CURRENT_SCALE);
    /* Reconstruct Phase currents from Bus Current*/ 
    SingleShunt_PhaseCurrentReconstruction(pSingleShunt);
#endif 
    
}
/**
* <B> Function: MCAPP_MeasureCurrentOffsetStatus(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to compensate offset from measured current samples.
*        
* @param Pointer to the data structure containing measured current.
* @return status bit after computing offset.
* 
* @example
* <CODE> MCAPP_MeasureCurrentOffsetStatus(&current); </CODE>
*
*/
int16_t MCAPP_MeasureCurrentOffsetStatus (MCAPP_MEASURE_T *pMotorInputs)
{
    return pMotorInputs->measureCurrent.status;
}


/**
* <B> Function: MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *, uint16_t )          </B>
*
* @brief Function to initialize the moving average filter
*
* @param Pointer to the data structure containing filter data. 
* @param moving average filter scaler 
* @return none.
* 
* @example
* <CODE> MCAPP_MeasureAvgInit(&filterData, scaler); </CODE>
*
*/
void MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *pFilterData,uint16_t scaler)
{
    pFilterData->scaler = scaler;
    pFilterData->maxIndex = (uint16_t)((1 << scaler));
    pFilterData->index = 0;
    pFilterData->sum = 0;
    pFilterData->status = 0;
}

/**
* <B> Function: int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *)         </B>
*
* @brief Function implementing moving average filter 
*
* @param Pointer to the data structure containing filter data.
* @return none.
* 
* @example
* <CODE> MC_MovingAvgFilter(&filterData);                          </CODE>
*
*/
int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *pFilterData)
{    
    pFilterData->sum += pFilterData->input;

    if (pFilterData->index < pFilterData->maxIndex)
    {
        pFilterData->index++;
    }
    else
    {
        pFilterData->avg = pFilterData->sum >> pFilterData->scaler; 
        pFilterData->sum = 0;
        pFilterData->index = 0;
        pFilterData->status = 1;
    }
    return pFilterData->avg;
}

// </editor-fold>