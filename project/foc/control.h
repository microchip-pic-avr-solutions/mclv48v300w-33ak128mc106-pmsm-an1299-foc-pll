// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file control.h
 *
 * @brief This header file lists data type definitions and variables required for
 * open loop and closed loop control of the motor.
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

#ifndef __CONTROL_H
#define __CONTROL_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="TYPE DEFINITIONS ">

/**
 * Control Parameter data type
 */
typedef struct
{
    /** Reference velocity */
    float   VelRef;
    /** Vd flux reference value */
    float   vdRef;
    /** Vq torque reference value */
    float   vqRef;
    /** Ramp for speed reference value */
	float	refRamp;
    /** Speed of the ramp */
	float   diffSpeed;
    /** Target Speed*/
    float   targetSpeed;
    /** The Speed Control Loop will be executed only every speedRampCount */
    float   speedRampCount;

} CTRL_PARM_T;

/**
 * General system flag data type
 */
typedef union
{
    struct
    {
        /** Run motor indication flag */
        unsigned RunMotor:1;
        /** Open loop/closed loop indication flag */
        unsigned OpenLoop:1;
        /** Mode changed indication flag - from open to closed loop */
        unsigned ChangeMode:1;
        /** Field weakening indication */
        unsigned ChangeSpeed:1;
        /** Unused bits */
        unsigned    :28;
    }bits;
    uint32_t Word;
} UGF_T;

/**
 * Motor Startup Parameter data type
*/
typedef struct
{
    /** Start up ramp in open loop. */
    float startupRamp;
    /** counter that is incremented in CalculateParkAngle() up to LOCK_TIME,*/
    int16_t startupLock;
    /* Start up ramp increment */
    float tuningAddRampup;
    float tuningDelayRampup;
} MOTOR_STARTUP_DATA_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc=" VARIABLES ">

extern UGF_T uGF;
extern CTRL_PARM_T CtrlParm;
extern MOTOR_STARTUP_DATA_T motorStartUpData;

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __CONTROL_H
