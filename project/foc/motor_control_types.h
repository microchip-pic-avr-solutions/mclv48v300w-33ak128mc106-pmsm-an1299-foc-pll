// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file motor_control_types.h
 *
 * @brief This module holds variable type definitions of data structure holding
 * Motor control parameters.
 *
 * Component: FOC
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


#ifndef __MOTOR_CONTROL_TYPES_H
#define __MOTOR_CONTROL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

/**
 * ABC reference frame data type
 */
typedef struct
{
    /* Phase A component */
    float   a;
    /* Phase B component */
    float   b;
    /* Phase C component */
    float   c;

} MC_ABC_T;

/**
 * Sine-Cosine data type
*/
typedef struct
{
    /* Cosine component */
    float cos;
    /* Sine component */
    float sin;

} MC_SINCOS_T;

/**
 * Angle data type
*/
typedef struct
{
    /* Angle in counts */
    int16_t counts;
    /* Angle in radians */
    float radian;

} MC_ANGLE_T;

/**
 * Speed data type
*/
typedef struct
{
    /* Rotor speed in RPM */
    float RPM;
    
    /* speed in radians per sec */
    float radpersec;

} MC_SPEED_T;

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif /* end of __MOTOR_CONTROL_TYPES_H */
