// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file generic_load.c
 *
 * @brief This module implements generic load state machine.
 *
 * Component: GENERIC LOAD
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "generic_load.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_GenericLoadStateMachine (MCAPP_GENERIC_LOAD_T *);
void MCAPP_GenericLoadInit (MCAPP_GENERIC_LOAD_T *);

// </editor-fold>

/**
* <B> Function: void MCAPP_GenericLoadInit (MCAPP_GENERIC_LOAD_T *)  </B>
*
* @brief Generic load initialization.
*
* @param Pointer to the data structure containing load parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_GenericLoadInit(&genrLoad); </CODE>
*
*/
void MCAPP_GenericLoadInit (MCAPP_GENERIC_LOAD_T *pGenericLoad)
{
    pGenericLoad->state = GENERIC_LOAD_WAIT;
}

/**
* <B> Function: void MCAPP_GenericLoadStateMachine (MCAPP_GENERIC_LOAD_T *)  </B>
*
* @brief Generic Load state machine.
*
* @param Pointer to the data structure containing load parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_GenericLoadStateMachine(&genrLoad); </CODE>
*
*/
void MCAPP_GenericLoadStateMachine (MCAPP_GENERIC_LOAD_T *pGenericLoad)
{
    switch(pGenericLoad->state)
    {
        case GENERIC_LOAD_WAIT:
            break;
            
        case GENERIC_LOAD_RUN:
            break;
            
        case GENERIC_LOAD_STOP:
            break;
            
        case GENERIC_LOAD_FAULT:            
            break;
            
        default:
            pGenericLoad->state = GENERIC_LOAD_FAULT;
        break;
    }
}

/**
* <B> Function: void MCAPP_IsGenericLoadReadyToStart (MCAPP_GENERIC_LOAD_T *)  
* </B>
*
* @brief Load Ready to Start Interface Function.
*
* @param Pointer to the data structure containing load parameters.
* @return Ready to start Flag.
* 
* @example
* <CODE> MCAPP_IsGenericLoadReadyToStart(&genrLoad); </CODE>
*
*/
uint8_t MCAPP_IsGenericLoadReadyToStart (MCAPP_GENERIC_LOAD_T *pGenericLoad)
{
    return 1;
}

/**
* <B> Function: void MCAPP_IsGenericLoadReadyToStop (MCAPP_GENERIC_LOAD_T *)  
* </B>
*
* @brief Load Ready to Stop Interface Function.
*
* @param Pointer to the data structure containing load parameters.
* @return Ready to stop Flag.
* 
* @example
* <CODE> MCAPP_IsLoadReadyToStop(&genrLoad); </CODE>
*
*/
uint8_t MCAPP_IsGenericLoadReadyToStop (MCAPP_GENERIC_LOAD_T *pGenericLoad)
{
    uint8_t status;
    
    if(fabsf(*pGenericLoad->mechSpeedRPM) < *pGenericLoad->minMechSpeedRPM)
    {
        status = 1;
    }
    else
    {
        status = 0;
    }

    return status;
}