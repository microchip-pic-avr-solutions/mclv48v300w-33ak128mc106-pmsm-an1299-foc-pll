// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file hurst075.h
 *
 * @brief This file has definitions to be configured by the user for spinning
 * motor using field oriented control.
 *
 * Motor : Hurst075 (Hurst DMB0224C10002 or AC300020 or Short Hurst)
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

#ifndef __HURST075_H
#define __HURST075_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <math.h>
#include <stdint.h>

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
    
/*Parameters of Hurst075 (Hurst DMB0224C10002 or AC300020 or Short Hurst)*/    

/** Motor Parameters */
/* per phase resistance (unit : Ohm) */
#define MOTOR_PER_PHASE_RESISTANCE                    2.833f
/* per phase inductance (unit : Henry) */
#define MOTOR_PER_PHASE_INDUCTANCE                    0.00243f
/* Motor Back EMF Constant (unit : line voltage peak / kRPM */
#define MOTOR_BEMF_CONSTANT_MECH                      7.56f
/* No.of pole pairs*/
#define POLE_PAIRS                                    5.0f
/* Speed for Open Loop to Closed Loop Transition (unit : RPM)*/
#define END_SPEED_RPM                                 250.0f
/* Nominal Speed without Field Weakening (unit : RPM)*/
#define NOMINAL_SPEED_RPM                             2000.0f
/* Maximum Speed with Field Weakening (unit : RPM)*/
#define FW_SPEED_RPM                                  3500.0f
/* Maximum motor current : peak current per phase(unit : amps)*/
#define MAX_MOTOR_CURRENT                              1.44f
/* Maximum ID Reference for Field Weakening (unit : amps)*/
#define MAX_FW_NEGATIVE_ID_REF                        -1.44f
    
/**DC Bus Utilization Factor or Modulation Index Limit*/    
#define DCBUS_UTILIZATION_FACTOR                      0.85f
/** Open Loop Current Reference (unit: Amps )*/
#define IQ_CURRENT_REF_OPENLOOP                        0.30f

/*PI Controller Parameters*/    
/** D-axis Current Control - PI Coefficients */
#define D_CURRCNTR_PTERM                              1.58
#define D_CURRCNTR_ITERM                              0.734
#define D_CURRCNTR_CTERM                              0.999
#define D_CURRCNTR_OUTMAX                             DCBUS_UTILIZATION_FACTOR

/** Q-axis Current Control - PI Coefficients */
#define Q_CURRCNTR_PTERM                              1.58
#define Q_CURRCNTR_ITERM                              0.734
#define Q_CURRCNTR_CTERM                              0.999
#define Q_CURRCNTR_OUTMAX                             DCBUS_UTILIZATION_FACTOR

/** Velocity Control - PI Coefficients */
#define SPEEDCNTR_PTERM                               0.00183
#define SPEEDCNTR_ITERM                               0.0543f/20000.0f
#define SPEEDCNTR_CTERM                               1.0
#define SPEEDCNTR_OUTMAX                              MAX_MOTOR_CURRENT

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __HURST075_H */
