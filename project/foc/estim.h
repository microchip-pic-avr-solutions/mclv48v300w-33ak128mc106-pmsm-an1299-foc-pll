// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim.h
 *
 * @brief This header file lists data type definitions and interface functions 
 * of the PLL estimator
 *
 * Component: ESTIMATOR - PLL
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

#ifndef __ESTIM_H
#define __ESTIM_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="TYPE DEFINITIONS ">

/**
 * State variables for PLL estimator
 */
typedef struct
{
    /** Integration constant */
    float   DeltaT;
    /** angle of estimation */
    float   Rho;
    /** internal variable for angle */
    float  	RhoStateVar;
    /** primary speed estimation */
    float   OmegaMr;
    /** last value for Ialpha */
    float   LastIalpha;
    /** last value for Ibeta */
    float   LastIbeta;
    /** difference Ialpha */
    float   dIalpha;
    /** difference Ibeta */
    float   dIbeta;
    /** dI*Ls/dt alpha */
	float	VIndalpha;
    /** dI*Ls/dt beta */
	float	VIndbeta;
    /** BEMF d filtered */
	float	Esdf;
    /** BEMF q filtered */
	float	Esqf;
    /** filter constant for d-q BEMF */
	float	KfilterEsdq;
    /** Estimated speed */
	float   VelEstim;
    /** Filter constant for Estimated speed */
	float   VelEstimFilterK;
    /** Value from last control step Ialpha */
    float   LastValpha;
    /** Value from last control step Ibeta */
    float   LastVbeta;
    /** dIalphabeta/dt */
	float	DIlimitLS;
    /** estimated angle initial offset */
	float   RhoOffset;

} tEstimParm;

/**
 * Motor Parameters used by the PLL estimator
 */
typedef struct
{
    /** Rs value - stator resistance */
	float	Rs;
    /** Ls/dt value - stator inductance / dt - variable with speed */
	float	LsDt;
    /** InvKfi constant value ( InvKfi = Omega/BEMF ) */
	float	InvKFi;

} tMotorEstimParm;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void	Estim(void);
void	InitEstimParm(void);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc=" VARIABLES ">

extern tEstimParm 	EstimParm;

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __ESTIM_H
