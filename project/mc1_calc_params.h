// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_calc_params.h
 *
 * @brief This file has definitions used in the application to run motor 1,
 *        calculated based on associated user parameter header file
 *        mc1_user_params.h.
 *
 * Component: BOARD
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

#ifndef __MC1_CALC_PARAMS_H
#define __MC1_CALC_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <math.h>
    
#include "mc1_user_params.h"
#include "adc.h"
#include "pwm.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
      
#define MC1_LOOPTIME_SEC            (float)(1.0/PWMFREQUENCY_HZ)
  
#define ADC_CURRENT_SCALE           (float)(MC1_PEAK_CURRENT/32768.0f)
      
#define MOTOR_BEMF_CONSTANT_ELEC    (float)(((((MOTOR_BEMF_CONSTANT_MECH/SQRT_3)*60.0f)/1000.0f)/(2.0f*M_PI))/POLE_PAIRS)  

#define MECHANICAL_RPM_TO_ELEC_RAD_PER_S    (float)(POLE_PAIRS*M_PI/30.0f)
    
/* Delta Ts factor */
#define	DELTA_T_Q30                 (float)(MC1_LOOPTIME_SEC *(2.0/60.0)*POLE_PAIRS * Q30_MAX)

/* Back EMF PLL Estimator Parameters */
#define	D_ILIMIT_HS                 (float)(NOMINAL_CURRENT_PEAK*MC1_LOOPTIME_SEC*MAXIMUM_SPEED_RPM*(2.0f*M_PI/60.0f)*POLE_PAIRS)
#define	D_ILIMIT_LS                 (float)(4*D_ILIMIT_HS) 
#define ESTIM_INVERSE_BEMF_CONSTANT (float)( 1.0f/( MOTOR_BEMF_CONSTANT_MECH/(SQRT_3 *1000.0f ) ) )

/* BEMF Estimator filter parameters  */
#define TAU_EDQ                     (float)(1.0f/(2.0f*M_PI*BEMF_FILTER_CUTOFF_FREQUENCY))
#define KFILTER_ESDQ                (float)(MC1_LOOPTIME_SEC/(MC1_LOOPTIME_SEC+TAU_EDQ))   
/* Velocity filter parameters  */    
#define TAU_VELESTIM                (float)(1.0f/(2.0f*M_PI*VELOCITY_FILTER_CUTOFF_FREQUENCY))
#define KFILTER_VELESTIM            (float)(MC1_LOOPTIME_SEC/(MC1_LOOPTIME_SEC+TAU_VELESTIM))
    
/* Rotor Lock parameters*/
/* Locking Time in counts of PWM switching interval*/      
#define LOCK_TIME_COUNTS            LOCK_TIME_SEC/MC1_LOOPTIME_SEC
/* Locking Voltage (unit : volts)
 * considered 200% to compensate dead time and circuit resistance*/
#define LOCKING_VOLTAGE             (float)((LOCK_CURRENT * MOTOR_PER_PHASE_RESISTANCE) * 2.0)
       
/* Flux Weakening Parameters */
/* Effective voltage considered for Id reference calculation */
#define EFFECTIVE_VOLATGE_FW        (float)(VMAX_CLOSEDLOOP_CONTROL * FW_VOLATGE_MARGIN_FACTOR )
/*Id reference calculation starts at this speed*/
#define FLUX_WEAKENING_ENABLE_SPEED (float)(NOMINAL_SPEED_RPM/2.0)
/* Id reference filter parameters*/
#define TAU_FW_IDREF                (float)(1.0f/(2.0f*M_PI*FW_ID_FILTER_CUTOFF_FREQUENCY)) 
#define KFILTER_FW_IDREF            (float)(MC1_LOOPTIME_SEC/(MC1_LOOPTIME_SEC+TAU_FW_IDREF))
/* Square of peak current, for current limit constraint */    
#define IMAX_SQUARE_FW        (float)(SPEEDCNTR_OUTMAX * SPEEDCNTR_OUTMAX )

/* Ramp Rates */    
/* Open Loop Speed Reference Ramp rate (unit : RPM per PWM switching interval)  */
#define OL_SPEED_REF_RAMP_VALUE      OPEN_LOOP_SPEED_REF_RAMP_RATE*MC1_LOOPTIME_SEC
/* Closed Loop Speed Reference Ramp rate (unit : RPM per PWM switching interval)  */
#define CL_SPEED_REF_RAMP_VALUE      CLOSED_LOOP_SPEED_REF_RAMP_RATE*MC1_LOOPTIME_SEC 
    
/* Square of Maximum Voltage */
#define MAX_VOLTAGE_SQUARE          ((float)VMAX_CLOSEDLOOP_CONTROL*VMAX_CLOSEDLOOP_CONTROL)
    
/* Comparator reference for PWM Fault PCI from DC Bus current*/
#define CMP_REF_DCBUS_FAULT         (uint16_t)(((OC_FAULT_LIMIT_DCBUS*HALF_ADC_COUNT)/MC1_PEAK_CURRENT)+HALF_ADC_COUNT)

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __MC1_CALC_PARAMS_H */
