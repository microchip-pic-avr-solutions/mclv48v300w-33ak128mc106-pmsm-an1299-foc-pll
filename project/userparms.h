// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file userparms.h
 *
 * @brief This file has definitions to be configured by the user for spinning
 * motor using field oriented control.
 *
 * Board : MCLV-48V-300W + dsPIC33AK128MC106 MC DIM
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

#ifndef __USERPARMS_H
#define __USERPARMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">   
#undef OPEN_LOOP_FUNCTIONING
#undef TORQUE_MODE
#define INTERNAL_OPAMP_CONFIG
#define SINGLE_SHUNT
/*Motor Selection : 1 = Hurst DMA0204024B101(AC300022: Hurst300 or Long Hurst)
                    2 = Hurst DMB0224C10002(AC300020: Hurst075 or Short Hurst)*/
#define MOTOR  1
// </editor-fold>    
    
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <math.h>
#include <stdint.h>
    
#if MOTOR == 1
    #include "hurst300.h"
#elif MOTOR == 2
    #include "hurst075.h"
#else
    #include "hurst300.h" 
#endif
// </editor-fold>
  
// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">    
/** Constants */
    
#define SINGLE_ELEC_ROT_RADS_PER_SEC                  (float)(M_PI)
#define SQRT_3                                        1.732050808f
#define MAX_COUNT                                     32767.0f
    
/** Board Parameters */
    
#define PEAK_CURRENT                                  22.0f
#define DC_BUS_VOLTAGE                                24.0f

#define LOOPTIME_SEC                                  0.00005f
#define DEADTIME_MICROSEC                             1.0f
    
/** Estimator Parameters - PLL */
 
#define KFILTER_ESDQ                                  1200.0f/32767.0f
#define KFILTER_VELESTIM                              900.0f/32767.0f

/** Control Loop Constants and Coefficients */ 
    
/** Lock time is the time needed for aligning the motor to a known vector before
   starting the motor in open loop (unit: seconds) */
#define LOCK_TIME_IN_SEC                              0.2f
/** Open Loop Speed Reference Ramp Rate */    
#define OPENLOOP_RAMPSPEED_INCREASERATE               0.000006541f   
/** Closed Loop Speed Reference Ramp */
#define SPEEDREFRAMP                                  1.0f
/** The Speed Control Ramp Executes every  SPEEDREFRAMP_COUNT */
#define SPEEDREFRAMP_COUNT                            10  
// </editor-fold>
    
// <editor-fold defaultstate="expanded" desc="DEFINITIONS/COMPUTATIONS ">    
/*Computations*/ 
  
#define LOOPTIME_MICROSEC                             (LOOPTIME_SEC * 1000000.0f)
#define EFFECTIVE_MAX_DUTY                            (float)((LOOPTIME_MICROSEC - DEADTIME_MICROSEC)/LOOPTIME_MICROSEC)
#define LOCK_COUNT_FOR_LOCK_TIME                      (uint32_t)(LOCK_TIME_IN_SEC/LOOPTIME_SEC)
#define ADC_CURRENT_SCALE                             (float)(PEAK_CURRENT/32768.0f)

#define ADC_CURRENT_SCALE_INVERSE                     (float)((32768.0f/ PEAK_CURRENT)

#define END_SPEED_RADS_SEC_ELEC                       (float)(((END_SPEED_RPM/60.0f)*2.0f*M_PI)*POLE_PAIRS)
#define END_SPEED_RADS_SEC_ELEC_IN_LOOPTIME           (float)(END_SPEED_RADS_SEC_ELEC * LOOPTIME_SEC)
 
#define NOMINAL_SPEED_RAD_PER_SEC_ELEC                (float)(((NOMINAL_SPEED_RPM/60.0f)*2.0f*M_PI)*POLE_PAIRS)

/** Fraction of dc link voltage(expressed as a squared amplitude) to set 
 * the limit for current controllers PI Output - 96% after Dead Time */
#define MAX_VOLTAGE_VECTOR                             DCBUS_UTILIZATION_FACTOR*DCBUS_UTILIZATION_FACTOR            

#define DC_BUS_VOLTAGE_DUE_TO_SVM                     (float)((DC_BUS_VOLTAGE * EFFECTIVE_MAX_DUTY)/SQRT_3)
#define EFFECTIVE_VOLATGE_FW                          (float)((DC_BUS_VOLTAGE * DCBUS_UTILIZATION_FACTOR)/SQRT_3)
#define MOTOR_BEMF_CONSTANT_ELEC                      (float)(((((MOTOR_BEMF_CONSTANT_MECH/SQRT_3)*60.0f)/1000.0f)/(2.0f*M_PI))/POLE_PAIRS)

#define FW_SPEED_RAD_PER_SEC_ELEC                     (float)(((FW_SPEED_RPM/60.0f)*2.0f*M_PI)*POLE_PAIRS)
 
/* In Torque Mode,the Q axis current reference is set by the Potentiometer.
 * in this mode, the speed controller is bypassed. */    
#ifdef TORQUE_MODE
    
    /* Minimum Q axis current in percentage of maximum motor current
     * This is the initial current to start the motor when POT is at zero */
    #define MIN_IQ_REF_LIMIT                        0.15f
    /* Maximum Q axis current in percentage of maximum motor current*/
    #define MAX_IQ_REF_LIMIT                        0.95f
                      
    /* Minimum Q axis current reference(in per unit) when POT is at zero position*/
    #define MIN_IQ_REF_PU                             (float)((MIN_IQ_REF_LIMIT * MAX_MOTOR_CURRENT) / PEAK_CURRENT)
    /* Maximum Q axis current reference in per unit*/
    #define MAX_IQ_REF_PU                             (float)((MAX_IQ_REF_LIMIT * MAX_MOTOR_CURRENT) / PEAK_CURRENT)

#endif

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __USERPARMS_H */
