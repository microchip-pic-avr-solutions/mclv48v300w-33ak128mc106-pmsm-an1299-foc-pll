// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file fdweak.c
 *
 * @brief This module implements equation based field weakening of PMSM.
 *
 * Component: FIELD WEAKENING
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
#include <math.h>

#include "fdweak.h"
#include "board_service.h"
#include "userparms.h"
#include "general.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">

FDWEAK_PARM_T fdWeakParm;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: InitFWParams() </B>
*
* @brief Function initializes the Field Weakening Parameters.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitFWParams(); </CODE>
*
*/
void InitFWParams(void)
{
    fdWeakParm.IdRef = 0.0;
    fdWeakParm.fwNum = 0.0;
    fdWeakParm.fwDen = 1.0;
}
/**
* <B> Function: FieldWeakening(float, float ) </B>
*
* @brief Function implements equation based field weakening algorithm.
*        
* @param motor speed.
* @return d-axis current reference corresponding to the motor speed.
* 
* @example
* <CODE> id = FieldWeakening(omega); </CODE>
*
*/
float FieldWeakening(float omega, float vqRef)
{
    if(omega > END_SPEED_RADS_SEC_ELEC)
    {
        fdWeakParm.fwNum = sqrt(SquareFloat(EFFECTIVE_VOLATGE_FW/omega) - 
                                SquareFloat(MOTOR_PER_PHASE_INDUCTANCE * 
                                                vqRef * PEAK_CURRENT));

        fdWeakParm.IdRef = ((-MOTOR_BEMF_CONSTANT_ELEC + fdWeakParm.fwNum) / 
                                                  MOTOR_PER_PHASE_INDUCTANCE);

        SaturateFloat(&fdWeakParm.IdRef, MAX_FW_NEGATIVE_ID_REF, 0.0f);

        fdWeakParm.IdRef = (fdWeakParm.IdRef / PEAK_CURRENT);
    }
    else
    {
        fdWeakParm.IdRef = 0.0;
    }
    
    return fdWeakParm.IdRef;
}

// </editor-fold>