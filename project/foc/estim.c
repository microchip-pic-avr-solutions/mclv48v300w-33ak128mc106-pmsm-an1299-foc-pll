// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim.c
 *
 * @brief This module implements PLL estimator
 *
 * Component: ESTIMATOR - PLL
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

#include "estim.h"
#include "clarke_park.h"
#include "userparms.h"
#include "general.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/CONSTANTS">

#define DECIMATE_NOMINAL_SPEED  ((NOMINAL_SPEED_RPM *(M_PI/30))*POLE_PAIRS/10)

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">

tEstimParm 			EstimParm;
tMotorEstimParm 	MotorEstimParm;
MC_ALPHABETA_T bemfAlphaBeta;
MC_DQ_T bemfdq;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: Estim() </B>
*
* @brief Function implements PLL estimator.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> Estim(); </CODE>
*
*/
void Estim(void)
{
    /** dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta */
    EstimParm.dIalpha	= (ialphabeta.alpha - EstimParm.LastIalpha);
    EstimParm.VIndalpha = (MotorEstimParm.LsDt * EstimParm.dIalpha);

    EstimParm.dIbeta = (ialphabeta.beta - EstimParm.LastIbeta);
    EstimParm.VIndbeta = (MotorEstimParm.LsDt * EstimParm.dIbeta);

    /** update  LastIalpha and LastIbeta */
    EstimParm.LastIalpha =	ialphabeta.alpha;
    EstimParm.LastIbeta = ialphabeta.beta;

    /** Stator voltage eqation
     * if Valpha = Rs * Ialpha + Ls dIalpha/dt + BEMF
     * then, BEMF = Valpha - Rs Ialpha - Ls dIalpha/dt
    */
	bemfAlphaBeta.alpha = (valphabeta.alpha * DC_BUS_VOLTAGE_DUE_TO_SVM) -
						((MotorEstimParm.Rs  * ialphabeta.alpha)) -
                                                EstimParm.VIndalpha;

    /** if Vbeta = Rs * Ibeta + Ls dIbeta/dt + BEMF
     *  then, BEMF = Vbeta - Rs Ibeta - Ls dIbeta/dt
    */
	bemfAlphaBeta.beta = (valphabeta.beta * DC_BUS_VOLTAGE_DUE_TO_SVM) -
							((MotorEstimParm.Rs  * ialphabeta.beta ))-
                                                   EstimParm.VIndbeta;

    /** Calculate Sin(Rho) and Cos(Rho) */
    sincosTheta.sin = sin(EstimParm.Rho);
    sincosTheta.cos = cos(EstimParm.Rho);

    /* Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark(&bemfAlphaBeta, &sincosTheta, &bemfdq);

    /** First Order Filter for Esd and Esq
     *  EsdFilter = 1/TFilterd * Intergal{( Esd - EsdFilter).dt } */
    
    LowPassFilter(bemfdq.d, EstimParm.KfilterEsdq, &EstimParm.Esdf);
    LowPassFilter(bemfdq.q, EstimParm.KfilterEsdq, &EstimParm.Esqf);

    if(EstimParm.Esqf > 0)
    {
    	EstimParm.OmegaMr = ((MotorEstimParm.InvKFi *
                                        (EstimParm.Esqf - EstimParm.Esdf)));
    }
    else
    {
    	EstimParm.OmegaMr = ((MotorEstimParm.InvKFi*
                                          (EstimParm.Esqf + EstimParm.Esdf)));
    }
    
    /** The integral of the angle is the estimated angle */
    EstimParm.Rho = EstimParm.Rho + (EstimParm.OmegaMr)*(EstimParm.DeltaT);

    if(EstimParm.Rho >= M_PI)
    {
        EstimParm.Rho = -M_PI + (EstimParm.Rho - M_PI);
    }
    /** The estimated speed is filtered value of the calculated OmegaMr.
     *  The filter implementation is the same that of \d-q Back-EMF filters. */
    LowPassFilter(EstimParm.OmegaMr, EstimParm.VelEstimFilterK, &EstimParm.VelEstim);

}

/**
* <B> Function: InitEstimParm() </B>
*
* @brief Function initializes PLL estimator.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitEstimParm(); </CODE>
*
*/
void InitEstimParm(void)
{
	MotorEstimParm.LsDt = (float)(MOTOR_PER_PHASE_INDUCTANCE/LOOPTIME_SEC);
	MotorEstimParm.Rs = MOTOR_PER_PHASE_RESISTANCE;
	MotorEstimParm.InvKFi = (float)(1.0/MOTOR_BEMF_CONSTANT_ELEC);
   	EstimParm.RhoStateVar = 0.0;
	EstimParm.OmegaMr = 0.0;
    EstimParm.KfilterEsdq = KFILTER_ESDQ;
    EstimParm.VelEstimFilterK = KFILTER_VELESTIM;
    EstimParm.DeltaT = LOOPTIME_SEC;
    EstimParm.RhoOffset = 0.0;
}

// </editor-fold>