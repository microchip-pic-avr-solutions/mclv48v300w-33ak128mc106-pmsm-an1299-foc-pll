// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file singleshunt.h
 *
 * @brief This header file lists the functions and definitions to configure 
 * Single Shunt Current Reconstruction Algorithm
 * 
 * Component: SINGLE SHUNT
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

#ifndef __SINGLESHUNT_H
#define	__SINGLESHUNT_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h> 
    
#include "clock.h"
#include "svm.h"
#include "pwm.h"
    
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">

/*  Critical Minimum window in seconds to measure current through single shunt*/
#define SSTCRIT_MICROSEC	2.5
/*  Critical Minimum window in counts to measure current through single shunt*/
#define SSTCRIT             (float)(SSTCRIT_MICROSEC*16*PWM_CLOCK_MHZ)
/* Additional delay for ADC Trigger in counts of PWM Period */
#define SS_SAMPLE_DELAY     0

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES ">
 /* Description:
    This structure will host parameters related to single shunt
 */
typedef struct
{
    float T1;
    float T2;
    float T7;
    float Ta1;
    float Ta2;
    float Tb1;
    float Tb2;
    float Tc1;
    float Tc2;
    /* Variable indicating the active sector in which SVM is in. The Sector value 
     * is used to identify which duty cycle registers to be modified in order to 
     * measure current through single shunt */
    uint8_t sectorSVM;			
    /* variable used to create minimum window to measure the current through single 
     * shunt resistor when enabled */
    float tcrit;
    /* variable used to create a delay for single shunt current measurement. 
     * Delay is defined in counts of PWM Period */
    float tDelaySample;		
    /* Reconstructed value for Ia */
    float Ia;			
    /* Reconstructed value for Ib */
    float Ib;				
    /* Reconstructed value for Ic */
    float Ic;				
    float Ibus1;
    float Ibus2;	
    /* This variable holds the first trigger value to be stored in TRIG1 register 
     * for A/D conversion. The converted value will be used for current reconstruction later on*/
    float trigger1;       
    /* Variable holding the second trigger value to be stored in TRIG1 register 
     * to trigger A/D conversion  */
    float trigger2;       
    MC_DUTYCYCLEOUT_T phase;
    MC_DUTYCYCLEOUT_T pdc;
    
} SINGLE_SHUNT_PARM_T;

typedef enum tagSSADCSAMPLE_STATE
{ 
    SS_SAMPLE_BUS1 = 0,     /* Bus Current Sample1 */
    SS_SAMPLE_BUS2 = 1,     /* Bus Current Sample2 */   
     
}SSADCSAMPLE_STATE;
   

// </editor-fold>
   
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

uint8_t SingleShunt_CalculateSpaceVectorPhaseShifted(MC_ABC_T *pABC,
                                                     float iPwmPeriod,
                                                     SINGLE_SHUNT_PARM_T *);
void SingleShunt_PhaseCurrentReconstruction(SINGLE_SHUNT_PARM_T *);
void SingleShunt_InitializeParameters(SINGLE_SHUNT_PARM_T *);

    
// </editor-fold>
    
#ifdef	__cplusplus
}
#endif

#endif	/* end of __SINGLESHUNT_H */
