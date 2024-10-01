// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file svm.c
 *
 * @brief This module implements Space Vector Modulation
 *
 * Component: SVM
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
#include "svm.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void CalcTimes(float );

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES ">

float T1, T2, Ta, Tb, Tc;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: CalcTimes(float ,MC_DUTYCYCLEOUT_T *)  </B>
*
* @brief Function calculates T1,T2,Tc,Tb and Ta required by SVM.
*        
* @param period value.
* @return none.
* 
* @example
* <CODE> CalcTimes(period); </CODE>
*
*/
static void CalcTimes(float period)
{
    T1 = period * T1;
    T2 = period * T2;
    Tc = (period-T1-T2)/2;
    Tb = Tc + T1;
    Ta = Tb + T2;
}

/**
* <B> Function: MC_CalculateSpaceVectorPhaseShifted(MC_ABC_T *,float ,
*                                                    MC_DUTYCYCLEOUT_T *)  </B>
*
* @brief Function implements space vector modulation.
*        
* @param Pointer to the data structure containing ABC voltages.
* @param period value.
* @param Pointer to the data structure containing Duty Cycle Outputs.
* @return none.
* 
* @example
* <CODE> MC_CalculateSpaceVectorPhaseShifted(&vabc,period,&dutycycle); </CODE>
*
*/
void MC_CalculateSpaceVectorPhaseShifted( const MC_ABC_T *pABC, float period,
                                              MC_DUTYCYCLEOUT_T *pDutyCycleOut)
{
    if(pABC->a < 0.0)
    {
        if(pABC->b < 0.0)
        {
            /** Must be Sector 4 since Sector 0 not allowed;
             * Sector 4: (1,0,0)  180-240 degrees */
            T2 = -pABC->a;
            T1 = -pABC->b;

            CalcTimes(period);

            pDutyCycleOut->dutycycle1 = Tc;
            pDutyCycleOut->dutycycle2  = Tb;
            pDutyCycleOut->dutycycle3  = Ta;
        }
        else
        {
            if(pABC->c < 0.0)
            {

                /** Sector 2: (0,1,0)  300-360 degrees */
                T2 = -pABC->c;
                T1 = -pABC->a;

                CalcTimes(period);

                pDutyCycleOut->dutycycle1  = Ta;
                pDutyCycleOut->dutycycle2  = Tc;
                pDutyCycleOut->dutycycle3  = Tb;
            }
            else
            {

                /** Sector 6: (1,1,0)  240-300 degrees */
                T2 = pABC->c;
                T1 = pABC->b;

                CalcTimes(period);

                pDutyCycleOut->dutycycle1  = Tb;
                pDutyCycleOut->dutycycle2  = Tc;
                pDutyCycleOut->dutycycle3  = Ta;
            }
        }
    }
    else
    {
        if(pABC->b < 0.0)
        {
            if(pABC->c < 0.0)
            {

                /** Sector 1: (0,0,1)  60-120 degrees */
                T2 = -pABC->b;
                T1 = -pABC->c;

                CalcTimes(period);

                pDutyCycleOut->dutycycle1  = Tb;
                pDutyCycleOut->dutycycle2  = Ta;
                pDutyCycleOut->dutycycle3  = Tc;
            }
            else
            {
                /** Sector 5: (1,0,1)  120-180 degrees */
                T2 = pABC->a;
                T1 = pABC->c;

                CalcTimes(period);

                pDutyCycleOut->dutycycle1  = Tc;
                pDutyCycleOut->dutycycle2  = Ta;
                pDutyCycleOut->dutycycle3  = Tb;
            }
        }
        else
        {
            /** Sector 3: (0,1,1)  0-60 degrees */
            T2 = pABC->b;
            T1 = pABC->a;

            CalcTimes(period);

            pDutyCycleOut->dutycycle1  = Ta;
            pDutyCycleOut->dutycycle2  = Tb;
            pDutyCycleOut->dutycycle3  = Tc;
        }
    }
}

// </editor-fold>