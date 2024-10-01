// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file clarke_park.c
 *
 * @brief This module implements Clarke and Park Transformations.
 *
 * Component: CLARKE PARK TRANSFORMS
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

#include "clarke_park.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/CONSTANTS">

#define SQRT3_BY2               (float)0.866025403788
#define ONE_BY_SQRT3            (float)0.5773502691

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: MC_TransformClarkeInverseSwappedInput( const MC_ALPHABETA_T *,
                                                                MC_ABC_T *) </B>
*
* @brief Function implementing Inverse Clarke Transform.
*        
* @param Pointer to the data structure containing alpha beta components.
* @param Pointer to the data structure containing outputs of the Clarke Transform.
* @return none.
* 
* @example
* <CODE> MC_TransformClarkeInverseSwappedInput(&alphabeta,&abc); </CODE>
*
*/
void MC_TransformClarkeInverseSwappedInput( const MC_ALPHABETA_T *pAlphaBeta,
                                                                MC_ABC_T *pABC)
{
    pABC->a = pAlphaBeta->beta;
    pABC->b = (-pAlphaBeta->beta/2.0 + SQRT3_BY2 * pAlphaBeta->alpha);
    pABC->c = (-pAlphaBeta->beta/2.0 - SQRT3_BY2 * pAlphaBeta->alpha);
}

/**
* <B> Function: MC_TransformParkInverse(const MC_DQ_T *pDQ, const MC_SINCOS_T *,
                                                    MC_ALPHABETA_T *)       </B>
*
* @brief Function implementing Inverse Park Transform.
*        .
* @param Pointer to the data structure containing d,q components.
* @param Pointer to the data structure containing sine and cosine of angle.
* @param Pointer to the data structure containing outputs of the Inverse Park Transform.
* @return none.
* 
* @example
* <CODE> MC_TransformParkInverse(&dq,&sincosTheta,&alphabeta);  </CODE>
*
*/
void MC_TransformParkInverse( const MC_DQ_T *pDQ, const MC_SINCOS_T *pSinCos,
                                                    MC_ALPHABETA_T *pAlphaBeta)
{
    pAlphaBeta->alpha =  (float)(pDQ->d * pSinCos->cos - pDQ->q * pSinCos->sin);
    pAlphaBeta->beta  =  (float)(pDQ->d * pSinCos->sin + pDQ->q * pSinCos->cos);
}

/**
* <B> Function: MC_TransformPark( const MC_ALPHABETA_T *,const MC_SINCOS_T *,
                                                               MC_DQ_T *))  </B>
*
* @brief Function implementing Park Transform.
*        .
* @param Pointer to the data structure containing alpha,beta components.
* @param Pointer to the data structure containing sine and cosine of angle.
* @param Pointer to the data structure containing outputs of the Park Transform.
* @return none.
* 
* @example
* <CODE> MC_TransformPark(&alphabeta,&sincosTheta,&dq); </CODE>
*
*/
void MC_TransformPark( const MC_ALPHABETA_T *pAlphaBeta,
                                       const MC_SINCOS_T *pSinCos, MC_DQ_T *pDQ)
{
    pDQ->d = (pAlphaBeta->alpha * pSinCos->cos) + (pAlphaBeta->beta * pSinCos->sin);
    pDQ->q = (-pAlphaBeta->alpha * pSinCos->sin) + (pAlphaBeta->beta * pSinCos->cos);
}

/**
* <B> Function: MC_TransformClarke(const MC_ABC_T *,MC_ALPHABETA_T *)  </B>
*
* @brief Function implementing Clarke Transform.
*        .
* @param Pointer to the data structure containing abc components which are inputs.
* @param Pointer to the data structure containing outputs of the Clarke Transform.
* @return none.
* 
* @example
* <CODE> MC_TransformClarke(&abc,&alphabeta); </CODE>
*
*/
void MC_TransformClarke(const MC_ABC_T *pABC,MC_ALPHABETA_T *pAlphaBeta)
{
    pAlphaBeta->alpha = pABC->a;
    pAlphaBeta->beta = (pABC->a * ONE_BY_SQRT3) + (pABC->b * 2.0 * ONE_BY_SQRT3);
}

// </editor-fold>

