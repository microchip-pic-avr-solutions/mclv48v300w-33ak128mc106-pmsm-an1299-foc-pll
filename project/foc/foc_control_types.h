// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file foc_control_types.h
 *
 * @brief This module holds variable type definitions of data structure holding
 * FOC control parameters.
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


#ifndef __FOC_CONTROL_TYPES_H
#define __FOC_CONTROL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "motor_control_types.h"
    
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    float
        speedRef,                   /* Reference Velocity */
        targetSpeed,                /* Speed Reference command*/
        CLSpeedRampRate,            /* Ramp Reference for Speed */
        idRef,                      /* D axis Current Reference Value */                 
        iqRef;                      /* Q axis Current Reference Value */
    
    float
        MaxVoltageSquare;           /* Maximum voltage limit square*/
        
    float 
        idRefOffset,
        iqRefOffset,
        currentRamp;
	
    float 
        lockVoltage;
	
    uint16_t
        openLoop,                   /* Open loop flag */
        fluxWeakEnable,             /* Flux weakening enable flag */
        lockTime,                   /* Lock variable for initial ramp */
        lockTimeLimit;              /* Total lock time */

} MCAPP_CONTROL_T;


typedef struct
{
    float   OLSpeedRampRate;        /* Ramp rate for Open loop */
    
    MC_ANGLE_T OLtheta;             /* Open Loop Angle */
    
    int16_t qDeltaT;                /* Scaled sampling time */
    
    int32_t OLThetaSum;             /* Open loop Theta Accumulation \
                                     * state variable in Q30*/
            
    float
        lockCurrent,                /* Locking current */
        OLCurrent,                  /* Open Loop Current */
        OLCurrentMax,               /* Maximum Open Loop Current */
        OLCurrentRampRate;          /* Current Ramp rate in open loop */
    
} MCAPP_OPENLOOPSTARTUP_T;



// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif /* end of __FOC_CONTROL_TYPES_H */
