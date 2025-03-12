// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_pll.c
 *
 * @brief This module implements PLL Estimator.
 * This is a sensor-less speed observer based on motor back EMF.
 *
 * Component: ESTIMATOR
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

/* _Q15abs and _Q15sqrt function use */
#include <math.h>
#include "estim_pll.h"
#include "clarke_park.h"
#include "util.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">

// </editor-fold>


/**
* <B> Function: void MCAPP_EstimatorPLLInit(MCAPP_PLL_ESTIMATOR_T *)  </B>
*
* @brief Function to reset PLL Estimator Data Structure variables.
*
* @param    pointer to the data structure containing PLL Estimator parameters.
* @return   none.
* @example
* <CODE> MMCAPP_EstimatorPLLInit(&estimator); </CODE>
*
*/
void MCAPP_EstimatorPLLInit(MCAPP_ESTIMATOR_PLL_T *pEstim)
{
    pEstim->qDiCounter = 0;
          
    pEstim->qThetaStateVar = 0;
    pEstim->qTheta = 0;
    
    pEstim->omegaFilt = 0;
    pEstim->qOmegaStateVar = 0;
}

/**
* <B> Function: void MCAPP_EstimatorPLL(MCAPP_PLL_ESTIMATOR_T *)  </B>
*
* @brief Observer to determine rotor speed and position based on
* motor parameters and feedbacks.
*
* @param    pointer to the data structure containing PLL Estimator parameters.
* @return   none.
* @example
* <CODE> MCAPP_EstimatorPLL(&estimator); </CODE>
*
*/
void MCAPP_EstimatorPLL(MCAPP_ESTIMATOR_PLL_T *pEstim)
{
    const MCAPP_MOTOR_T *pMotor = pEstim->pMotor;
    const MC_ALPHABETA_T *pIAlphaBeta = pEstim->pIAlphaBeta;
 
    MC_SINCOS_T     estimSinCos;        /* Sine-cosine for estimator */  
     
    pEstim->Valpha = pEstim->pVAlphaBeta->alpha;
    pEstim->Vbeta  = pEstim->pVAlphaBeta->beta;
    
    uint16_t index = (pEstim->qDiCounter - 3)&0x0003;
    
    if(fabsf(pEstim->omegaFilt) < pEstim->thresholdSpeedDerivative)
    {
        pEstim->dIalpha = (pIAlphaBeta->alpha - pEstim->LastIalphaHS[index]);
        pEstim->dIbeta = (pIAlphaBeta->beta - pEstim->LastIbetaHS[index]);
        /* The current difference can exceed the maximum value per 4 ADC ISR
           cycle .The following limitation assures a limitation per low speed -
           up to the nominal speed */
        if (pEstim->dIalpha > pEstim->DIlimitLS) 
        {
            pEstim->dIalpha = pEstim->DIlimitLS;
        }
        if (pEstim->dIalpha < -pEstim->DIlimitLS) 
        {
            pEstim->dIalpha = -pEstim->DIlimitLS;
        }

        if (pEstim->dIbeta > pEstim->DIlimitLS) 
        {
            pEstim->dIbeta = pEstim->DIlimitLS;
        }
        if (pEstim->dIbeta < -pEstim->DIlimitLS) 
        {
            pEstim->dIbeta = -pEstim->DIlimitLS;
        }
        pEstim->vIndalpha = (pMotor->Ls * pEstim->dIalpha * pEstim->inverseDt/4);
        pEstim->vIndbeta = (pMotor->Ls * pEstim->dIbeta * pEstim->inverseDt/4);
    }
    else
    {
        pEstim->dIalpha = (pIAlphaBeta->alpha - pEstim->LastIalphaHS[pEstim->qDiCounter]);
        pEstim->dIbeta = (pIAlphaBeta->beta - pEstim->LastIbetaHS[pEstim->qDiCounter]);
        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
        the following limitation assures a limitation per high speed - up to
        the maximum speed */
        if (pEstim->dIalpha > pEstim->DIlimitHS) 
        {
            pEstim->dIalpha = pEstim->DIlimitHS;
        }
        if (pEstim->dIalpha < -pEstim->DIlimitHS) 
        {
            pEstim->dIalpha = -pEstim->DIlimitHS;
        }
        
        if (pEstim->dIbeta > pEstim->DIlimitHS) 
        {
            pEstim->dIbeta = pEstim->DIlimitHS;
        }
        if (pEstim->dIbeta < -pEstim->DIlimitHS) 
        {
            pEstim->dIbeta = -pEstim->DIlimitHS;
        }
        pEstim->vIndalpha = (pMotor->Ls * pEstim->dIalpha * pEstim->inverseDt);
        pEstim->vIndbeta = (pMotor->Ls * pEstim->dIbeta * pEstim->inverseDt);
    }
    /* Update the sample history of Ialpha and Ibeta */
    pEstim->qDiCounter = (pEstim->qDiCounter + 1) & 0x0003;
    pEstim->LastIalphaHS[pEstim->qDiCounter] = pIAlphaBeta->alpha;
    pEstim->LastIbetaHS[pEstim->qDiCounter] = pIAlphaBeta->beta;

   /* Calculate the BEMF voltage:
    *  Ealphabeta = Valphabeta - Rs*Ialphabeta - Ls*(dIalphabeta/dt)  */
    
    pEstim->BEMFAlphaBeta.alpha =  (pEstim->Valpha -(pMotor->Rs * pIAlphaBeta->alpha) 
                                    - pEstim->vIndalpha );
    
    pEstim->BEMFAlphaBeta.beta =   (pEstim->Vbeta -(pMotor->Rs * pIAlphaBeta->beta)
                                     - pEstim->vIndbeta );
    
    /* Calculate sine and cosine components of the rotor flux angle */
    estimSinCos.sin = sin( pEstim->qTheta * Q15_TO_RADIAN); 
    estimSinCos.cos = cos( pEstim->qTheta * Q15_TO_RADIAN);

    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark(&pEstim->BEMFAlphaBeta, &estimSinCos, &pEstim->BEMFdq);

    /** First Order Filter for Esd and Esq
     *  EsdFilter = 1/TFilterd * Intergal{( Esd - EsdFilter).dt } */
    LowPassFilter(pEstim->BEMFdq.d, pEstim->qKfilterEsdq, &pEstim->Esdf);
    
    LowPassFilter(pEstim->BEMFdq.q, pEstim->qKfilterEsdq, &pEstim->Esqf);


     /*  For stability the condition for low speed */
    if (fabsf(pEstim->omegaFilt) > pEstim->thresholdSpeedBEMF)
    {
        /* At speed greater than decimation speed, calculate the estimated
         * velocity based on:
         * OmegaMr = Invpsi * (Eqfiltered - sign(Eqfiltered) * Edfiltered)
         */
        if (pEstim->Esqf > 0) 
        {
            pEstim->omega = (pEstim->invKfiConst * 
                                               (pEstim->Esqf - pEstim->Esdf));
        } 
        else 
        {
            pEstim->omega = (pEstim->invKfiConst *
                                                (pEstim->Esqf + pEstim->Esdf));
        }
    }        
    else 
    {
        /* At speed lower than or equal to decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = (1/ke) * (Eqfiltered - sign(omega) * Edfiltered)
         * to improve stability.
         */
        if (pEstim->omegaFilt > 0) 
        {
            pEstim->omega = (pEstim->invKfiConst * 
                                               (pEstim->Esqf - pEstim->Esdf));
        } 
        else 
        {
            pEstim->omega = (pEstim->invKfiConst *
                                                (pEstim->Esqf + pEstim->Esdf));
        }
    }
  
    /* Integrate the estimated rotor velocity to get estimated rotor angle */ 
    pEstim->qThetaStateVar += (int32_t)( pEstim->omega * pEstim->q30DeltaTs);
    pEstim->qTheta = (int16_t)(pEstim->qThetaStateVar >>15);
               
    /* Filter the estimated velocity using a first order low-pass filter */
    LowPassFilter(pEstim->omega, pEstim->qOmegaFiltConst, &pEstim->omegaFilt);
}



