// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file adc.h
 *
 * @brief This header file lists the functions and definitions - to configure 
 * and enable ADC Module and its features 
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: ADC
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

#ifndef _ADC_H
#define _ADC_H
        
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
        
#include <xc.h>
#include <stdint.h>
        
#include "userparms.h"
        
// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
        
#define ADCBUF_IA     (int16_t)(2048 - AD1CH0DATA)<<4
#define ADCBUF_IB     (int16_t)(2048 - AD2CH0DATA)<<4
        
#define ADCBUF_IBUS1   (int16_t)((AD1CH2DATA- 2048)<<4)
#define ADCBUF_IBUS2   (int16_t)((AD1CH3DATA- 2048)<<4)      
        
#define ADCBUF_POT    (int16_t)AD1CH1DATA
/*Maximum count in 12-bit ADC*/
#define MAX_POT_COUNT  4096.0f

#ifdef SINGLE_SHUNT        
    /* IBUS2 (AD1CH3) is the ADC Interrupt source in Single Shunt*/
    #define EnableADCInterrupt()            _AD1CH3IE = 1
    #define DisableADCInterrupt()           _AD1CH3IE = 0
    #define ADCInterrupt                    _AD1CH3Interrupt  
    #define ADCInterruptFlagClear           _AD1CH3IF = 0 
#else
    /* POT (AD1CH1) is the ADC Interrupt source in Dual Shunt*/
    #define EnableADCInterrupt()            _AD1CH1IE = 1
    #define DisableADCInterrupt()           _AD1CH1IE = 0
    #define ADCInterrupt                    _AD1CH1Interrupt  
    #define ADCInterruptFlagClear           _AD1CH1IF = 0      
#endif              

// </editor-fold>
        
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitializeADCs(void);

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif      // end of __ADC_H

