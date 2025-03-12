// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file hurst300.h
 *
 * @brief This file has definitions to be configured by the user for spinning
 * motor using field oriented control.
 *
 * Motor : Hurst300 (Hurst DMA0204024B101 or AC300022 or Long Hurst)
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

#ifndef __HURST300_H
#define __HURST300_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <math.h>
#include <stdint.h>

#include "../mc1_user_params.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
    
/*Parameters of Hurst300 (Hurst DMA0204024B101 : AC300022 : Long Hurst)*/
/** Motor Name Plate Parameters */
/* No.of pole pairs*/
#define POLE_PAIRS                                      5
/* per phase resistance (unit : ohm) */
#define MOTOR_PER_PHASE_RESISTANCE                      0.3715f
/* per phase inductance (unit : henry) */
#define MOTOR_PER_PHASE_INDUCTANCE                      0.000359f
/* Motor Back EMF Constant (unit : line voltage peak / kRPM) */
#define MOTOR_BEMF_CONSTANT_MECH                        6.7316f
/* Speed for Open Loop to Closed Loop Transition (unit : RPM)*/
#define MINIMUM_SPEED_RPM                               500.0f
/* Nominal Speed without Flux Weakening (unit : RPM)*/
#define NOMINAL_SPEED_RPM                               2500.0f
/* Maximum Speed with Flux Weakening (unit : RPM)*/
#define MAXIMUM_SPEED_RPM                               3500.0f
/* Motor Rated Phase Current in RMS (unit : amps) */
#define NOMINAL_CURRENT_PHASE_RMS                       3.4f
    
/* Motor Peak Current per phase (unit : amps) */
#define NOMINAL_CURRENT_PEAK      (float) (NOMINAL_CURRENT_PHASE_RMS * M_SQRT2)

/*PI Controller Parameters*/    
/** D-axis Current Control Loop - PI Coefficients */
#define D_CURRCNTR_PTERM                                1.19667f
#define D_CURRCNTR_ITERM                                0.061917f
#define D_CURRCNTR_OUTMAX                               VMAX_CLOSEDLOOP_CONTROL

/** Q-axis Current Control Loop - PI Coefficients */
#define	Q_CURRCNTR_PTERM                                1.19667f
#define	Q_CURRCNTR_ITERM                                0.061917f
#define Q_CURRCNTR_OUTMAX                               VMAX_CLOSEDLOOP_CONTROL

/** Speed Control Loop - PI Coefficients */
#define SPEEDCNTR_PTERM                                 0.00573f
#define SPEEDCNTR_ITERM                                 0.000012929f
#define SPEEDCNTR_OUTMAX                                NOMINAL_CURRENT_PEAK

/* Filter constant definitions  */
/* BEMF filter cut off frequency (unit : Hz)*/
#define BEMF_FILTER_CUTOFF_FREQUENCY                    250.0f
/* Velocity filter cut off frequency (unit : Hz)*/
#define VELOCITY_FILTER_CUTOFF_FREQUENCY                75.0f    
    
/* Control parameters */
/* Open loop startup peak current per phase (unit : amps) */
#define OPEN_LOOP_CURRENT                               1.0f
/* Open Loop Speed Reference Ramp rate (unit : RPM per second)  */
#define OPEN_LOOP_SPEED_REF_RAMP_RATE                   2000.0f 
/* Open Loop to Closed Loop Transition speed (unit : RPM)*/
#define MAX_OPENLOOP_SPEED_RPM                          MINIMUM_SPEED_RPM
/* Closed Loop Speed Reference Ramp rate (unit : RPM per second) */
#define CLOSED_LOOP_SPEED_REF_RAMP_RATE                 2000.0f
/*Current ramp rate for open loop to closed loop (unit : amps per MC1_LOOPTIME_SEC*/
#define CURRENT_RAMP_VALUE                              0.05f
    
/*Fault parameters*/
/* Overcurrent fault limit(software) - phase current (unit : amps)*/
#define OC_FAULT_LIMIT_PHASE                            7.0f  
/* Overcurrent fault limit(comparator and Fault PCI) - bus current (unit : amps)*/
#define OC_FAULT_LIMIT_DCBUS                            7.0f  
    
/* Rotor locking parameters */
/* Lock time for Motor's poles alignment (unit : seconds)*/
#define LOCK_TIME_SEC                                   0.5f
/* Locking Current (unit : amps)*/
#define LOCK_CURRENT                                    1.0f
 
/** Flux Weakening Control Parameters */  
/* Voltage circle limit for Flux Weakening*/
#define FW_VOLATGE_MARGIN_FACTOR                        0.90f
/* Maximum D-axis current Reference for Flux Weakening (unit : amps)*/
#define MAX_FW_NEGATIVE_ID_REF                          -(NOMINAL_CURRENT_PEAK)
/* Voltage feedback Flux Weakening controller parameters */
#define FW_PTERM                                        SPEEDCNTR_PTERM*30 
#define FW_ITERM                                        SPEEDCNTR_ITERM*30
/* Flux Weakening Id reference filter cut off frequency (unit : Hz)*/
#define FW_ID_FILTER_CUTOFF_FREQUENCY                   100.0f
    
// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __HURST300_H */
