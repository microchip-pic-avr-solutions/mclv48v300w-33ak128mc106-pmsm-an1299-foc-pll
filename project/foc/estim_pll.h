// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_pll.h
 *
 * @brief This module implements Back EMF based PLL Estimator.
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

#ifndef __ESTIM_PLL_H
#define __ESTIM_PLL_H

#ifdef __cplusplus
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
        
#include "foc_control_types.h"
#include "motor_types.h"
#include "clarke_park.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS">

 /* Description:
    This structure will host parameters related to angle/speed estimator
    parameters. */
        
typedef struct
{    
    /* Integration constant */
    float q30DeltaTs;
    /* angle of estimation */
    int16_t qTheta;
    /* internal variable for angle */
    int32_t qThetaStateVar;
    
    float lastIalpha;
    float lastIbeta;
    /* difference Ialpha */
    float  dIalpha;
    /* difference Ibeta */
    float dIbeta;
    /* Phase voltage Valpha*/
    float Valpha;
    /* Phase voltage Vbeta*/
    float Vbeta;
    
    /* counter in Last DI tables */
    int16_t qDiCounter;
    
    float inverseDt;
    /* dI*Ls/dt alpha */
    float vIndalpha;
    /* dI*Ls/dt beta */
    float vIndbeta;
    /* BEMF d filtered */
    float Esdf;

    /* BEMF q filtered */
    float Esqf;

    /* filter constant for d-q BEMF */
    float qKfilterEsdq;
    /* Filter constant for Estimated speed */
    float qOmegaFiltConst;
    /* State Variable for Estimated speed */
    float qOmegaStateVar;
    /* dIalphabeta/dt */
    float DIlimitLS;
    /* dIalphabeta/dt */
    float DIlimitHS;
    /*  last  value for Ialpha */
    float LastIalphaHS[8];
    /* last  value for Ibeta */
    float LastIbetaHS[8];

    /* BEMF Filter Constant */
    float qEmagFiltConst;   
    
    float invKfiConst,
          omega,
          omegaFilt;
    
    /* Threshold speed for derivative calculation */
    float thresholdSpeedDerivative;
    /* Threshold speed for omega computation */
    float thresholdSpeedBEMF;
    
    /* Back EMF voltage in alpha-beta */ 
    MC_ALPHABETA_T  BEMFAlphaBeta;    
    /* Back EMF voltage in DQ */
    MC_DQ_T         BEMFdq;            
     
    const MCAPP_CONTROL_T *pCtrlParam;
    const MC_ALPHABETA_T *pIAlphaBeta;
    const MC_ALPHABETA_T *pVAlphaBeta;
    const MCAPP_MOTOR_T *pMotor;
    
} MCAPP_ESTIMATOR_PLL_T;        
        
        

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_EstimatorPLLInit (MCAPP_ESTIMATOR_PLL_T *);
void MCAPP_EstimatorPLL (MCAPP_ESTIMATOR_PLL_T *);

// </editor-fold>

#ifdef __cplusplus
    }
#endif

#endif /* end of __ESTIM_PLL_H */
