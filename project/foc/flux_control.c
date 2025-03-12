// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file flux_control.c
 *
 * @brief This module implements equation based flux weakening of PMSM.
 *
 * Component: FLUX WEAKENING
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
#include <math.h>

#include "util.h"
#include "flux_control.h"
#include "mc1_calc_params.h"
#include "foc_control_types.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_FeedForwardFluxWeakeningInit(MCAPP_FLUX_WEAKENING_FF_T *);
static void MCAPP_FeedBackFluxWeakeningInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T *);
static float MCAPP_FeedForwardFluxWeakening(MCAPP_FLUX_WEAKENING_FF_T *);
static float MCAPP_FeedBackFluxWeakening(MCAPP_FLUX_WEAKENING_VOLT_FB_T *);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: MCAPP_FluxWeakeningControlInit(MCAPP_ID_REFERENCE_T *) </B>
*
* @brief Function initializes the Flux Weakening Parameters.
*        
* @param Pointer to the data structure containing flux weakening parameters
* @return none.
 * 
* @example
* <CODE> MCAPP_FluxWeakeningControlInit(&pFOC->idRefGen); </CODE>
*
*/
void MCAPP_FluxWeakeningControlInit(MCAPP_ID_REFERENCE_T *pIdRefGen)
{
    MCAPP_FeedBackFluxWeakeningInit(&pIdRefGen->feedBackFW);
    MCAPP_FeedForwardFluxWeakeningInit(&pIdRefGen->feedForwardFW);
}

/**
* <B> Function: MCAPP_FluxWeakeningControl(MCAPP_ID_REFERENCE_T *) </B>
*
* @brief Function to select the type of flux weakening
*        
* @param Pointer to the data structure containing flux weakening parameters
* @return none.
 * 
* @example
* <CODE> MCAPP_FluxWeakeningControl(&pFOC->idRefGen); </CODE>
*
*/
void MCAPP_FluxWeakeningControl(MCAPP_ID_REFERENCE_T *pIdRefGen)
{
    float idRefSquared, iqSquaredLimit;
    
    if(pIdRefGen->variant == 1)
    {
        pIdRefGen->idRef = MCAPP_FeedBackFluxWeakening(&pIdRefGen->feedBackFW);
    }
    else if(pIdRefGen->variant == 2)
    {
        pIdRefGen->idRef = MCAPP_FeedForwardFluxWeakening(&pIdRefGen->feedForwardFW);
    }
    else
    {
        pIdRefGen->idRef = 0.0;
    }
    
    /* Dynamic limiting for Q axis current */ 
    idRefSquared  = pIdRefGen->idRef * pIdRefGen->idRef ;
    iqSquaredLimit = pIdRefGen->ImaxSquare - idRefSquared;  

    pIdRefGen->iqMax  = sqrt(iqSquaredLimit);

}

/**
* <B> Function: MCAPP_FeedForwardFluxWeakeningInit(MCAPP_FLUX_WEAKENING_FF_T *) </B>
*
* @brief Function initializes the Feed Forward flux Weakening Parameters.
*        
* @param Pointer to the data structure containing flux weakening parameters
* @return none.
 * 
* @example
* <CODE> MCAPP_FeedForwardFluxWeakeningInit(&pIdRefGen->feedForwardFW); </CODE>
*
*/
static void MCAPP_FeedForwardFluxWeakeningInit(MCAPP_FLUX_WEAKENING_FF_T *pFdweak)
{
    pFdweak->IdRef = 0;
}

/**
* <B> Function: MCAPP_FeedBackFluxWeakeningInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T *) </B>
*
* @brief Function initializes the Voltage Feedback based flux Weakening Parameters.
*        
* @param Pointer to the data structure containing flux weakening parameters
* @return none.
 * 
* @example
* <CODE> MCAPP_FeedBackFluxWeakeningInit(&pIdRefGen->feedBackFW); </CODE>
*
*/
static void MCAPP_FeedBackFluxWeakeningInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFdWeak)
{
    MC_ControllerPIReset(&pFdWeak->FWeakPI, 0);
}

/**
* <B> Function: MCAPP_FeedForwardFluxWeakening(MCAPP_FLUX_WEAKENING_FF_T * ) </B>
*
* @brief Function implements feed-forward flux weakening algorithm.
*        
* @param Pointer to the data structure containing flux weakening parameters
* @return d-axis current reference corresponding to the motor speed.
* 
* @example
* <CODE> pIdRefGen->idRef = MCAPP_FeedForwardFluxWeakening(&pIdRefGen->feedForwardFW); </CODE>
*
*/
static float  MCAPP_FeedForwardFluxWeakening(MCAPP_FLUX_WEAKENING_FF_T *pFdWeak)
{
    float IdRefOut; 
    
    const MCAPP_CONTROL_T *pCtrlParam = pFdWeak->pCtrlParam;
    const MCAPP_MOTOR_T *pMotor = pFdWeak->pMotor;
    
    float omega_ref = pCtrlParam->speedRef * MECHANICAL_RPM_TO_ELEC_RAD_PER_S;
    
    if(fabs(pCtrlParam->speedRef)  > FLUX_WEAKENING_ENABLE_SPEED)
    {
        pFdWeak->fwNum = sqrt(SquareFloat(pFdWeak->voltageLimitFW/omega_ref) - 
                                SquareFloat(pMotor->Ls * pCtrlParam->iqRef));

        IdRefOut = ((-pMotor->Ke + pFdWeak->fwNum)/pMotor->Ls);

        SaturateFloat(&IdRefOut, pFdWeak->IdRefMin, 0.0f);
               
    }
    else
    {
        IdRefOut = 0.0;
    }
    
        /*Filter for the FW Id reference current*/
#ifdef ID_REFERNCE_FILTER_ENABLE
    LowPassFilter(IdRefOut, pFdWeak->IdRefFiltConst, &pFdWeak->IdRefFilt);
    pFdWeak->IdRef = pFdWeak->IdRefFilt;
#else
    pFdWeak->IdRef = IdRefOut;
#endif
    
    return pFdWeak->IdRef;
}

/**
* <B> Function: MCAPP_FeedBackFluxWeakening(MCAPP_FLUX_WEAKENING_FF_T * ) </B>
*
* @brief Function implements voltage feedback based flux weakening algorithm.
*        
* @param Pointer to the data structure containing flux weakening parameters
* @return d-axis current reference corresponding to the motor speed.
* 
* @example
* <CODE> pIdRefGen->idRef = MCAPP_FeedBackFluxWeakening(&pIdRefGen->feedBackFW);</CODE>
*
*/
static float MCAPP_FeedBackFluxWeakening(MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFdWeak)
{    
    const MC_DQ_T *pVdq         = pFdWeak->pVdq;
    MCAPP_CONTROL_T *pCtrlParam = pFdWeak->pCtrlParam;

    float vdSqr, vqSqr, IdRefOut; 

    /* Compute voltage vector magnitude */
    vdSqr  = (pVdq->d * pVdq->d);
    vqSqr  = (pVdq->q * pVdq->q);
    pFdWeak->voltageMag = sqrt(vdSqr+vqSqr);
    
    if(fabs(pCtrlParam->speedRef) > FLUX_WEAKENING_ENABLE_SPEED)
    { 
        /* Compute PI output: pFdWeak->IdRef */
        pFdWeak->FWeakPI.inReference = pFdWeak->voltageMagRef;
        pFdWeak->FWeakPI.inMeasure = pFdWeak->voltageMag;
        MC_ControllerPIUpdate(&pFdWeak->FWeakPI);
        IdRefOut = pFdWeak->FWeakPI.output;
        
    }
    else
    {
        IdRefOut = 0 ;
        
        /* Reset PI integrator to Nominal Id reference value for smooth transition
         * when switching to PI output computation. */
        MC_ControllerPIReset(&pFdWeak->FWeakPI, 0);
    }

    /*Filter for the FW Id reference current*/
#ifdef ID_REFERNCE_FILTER_ENABLE
    LowPassFilter(IdRefOut, pFdWeak->IdRefFiltConst, &pFdWeak->IdRefFilt);
    pFdWeak->IdRef = pFdWeak->IdRefFilt;
#else
    pFdWeak->IdRef = IdRefOut;
#endif
    
    return pFdWeak->IdRef;
}



// </editor-fold>