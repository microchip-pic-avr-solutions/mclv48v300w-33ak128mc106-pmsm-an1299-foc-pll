// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file clarke_park.h
 *
 * @brief This header file lists variable types and interface functions for 
 * clarke and park transformations.
 *
 *
 * Component: CLARKE PARK TRANSFORMS
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

#ifndef __CLARKE_PARK_H
#define __CLARKE_PARK_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "motor_control_types.h"

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="TYPE DEFINITIONS ">

/**
 * D-Q reference frame data type
*/
typedef struct
{
    /** D-axis component */
    float d;
    /** Q-axis component */
    float q;

} MC_DQ_T;

/**
 * Alpha-Beta reference frame data type
*/
typedef struct
{
    /** Alpha component */
    float alpha;
    /** Beta component */
    float beta;

} MC_ALPHABETA_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MC_TransformClarkeInverseSwappedInput( const MC_ALPHABETA_T *,MC_ABC_T *);
void MC_TransformParkInverse( const MC_DQ_T *,const MC_SINCOS_T *,
                                                            MC_ALPHABETA_T *);
void MC_TransformPark(const MC_ALPHABETA_T *,const MC_SINCOS_T *,MC_DQ_T *);
void MC_TransformClarke(const MC_ABC_T *,MC_ALPHABETA_T *);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc=" VARIABLES ">



// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __CLARKE_PARK_H
