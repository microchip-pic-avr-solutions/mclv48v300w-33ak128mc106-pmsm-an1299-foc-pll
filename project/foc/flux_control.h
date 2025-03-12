// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file fdweak.h
 *
 * @brief This header file lists data type definitions and interface functions 
 * of the flux weakening module
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

#ifndef __FDWEAK_H
#define __FDWEAK_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "foc_control_types.h"
#include "motor_types.h"
#include "clarke_park.h"
#include "pi.h"
// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="TYPE DEFINITIONS ">
        
/*
 * Flux weakening control data type for feed-forward
 */
typedef struct
{
    float 
        IdRef,          /* Id Current reference */       
        IdRefFilt,      /* Filtered Id Current reference */ 
        IdRefFiltConst, /* Filter constant for Id */
        IdRefMin,       /* Lower limit on IdRef */        
        voltageLimitFW, /* Effective voltage limit for flux weakening */
        fwNum,          /* numerator part of the flux weakening equation */
        fwDen;          /* denominator part of the flux weakening equation */

    const MCAPP_CONTROL_T *pCtrlParam;
    const MCAPP_MOTOR_T *pMotor;
    
} MCAPP_FLUX_WEAKENING_FF_T;

typedef struct
{
    float
        IdRef,              /* Id Current reference */
        IdRefFilt,          /* Filtered Id Current reference */
        IdRefFiltConst,     /* Filter constant for Id */
        IdRefMin,           /* Lower limit on IdRef */
        voltageMag,         /* Voltage vector magnitude */
        voltageMagRef;      /* Voltage vector magnitude reference */    
    
    MC_PI_T FWeakPI;

    MCAPP_CONTROL_T *pCtrlParam;
    const MC_DQ_T *pVdq;
    const MCAPP_MOTOR_T *pMotor;

} MCAPP_FLUX_WEAKENING_VOLT_FB_T;
 

/*
 * Flux weakening control data type for feedback
 */
typedef struct
{
    /* Id Current reference */
    float idRef,
          ImaxSquare,
          iqMax;
 
    /* Feed-forward or Feedback based flux weakening */
    uint8_t variant;  
    
    MCAPP_FLUX_WEAKENING_VOLT_FB_T 
        feedBackFW;
    
    MCAPP_FLUX_WEAKENING_FF_T 
        feedForwardFW;

} MCAPP_ID_REFERENCE_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
/* Define the macro ID_REFERNCE_FILTER_ENABLE to filter the Id reference generated 
 * from the flux weakening control algorithm */
#undef ID_REFERNCE_FILTER_ENABLE

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_FluxWeakeningControlInit(MCAPP_ID_REFERENCE_T *);
void MCAPP_FluxWeakeningControl(MCAPP_ID_REFERENCE_T *);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc=" VARIABLES ">


// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __FDWEAK_H
