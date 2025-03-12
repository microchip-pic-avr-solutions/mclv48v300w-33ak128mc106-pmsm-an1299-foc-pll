// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_init.c
 *
 * @brief This module initializes data structure holding motor control
 * parameters required to run motor 1 using field oriented control.
 * In this application to initialize variable required to run the application.
 *
 * Component: APPLICATION (Motor Control 1 - mc1)
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
#include <string.h>

#include "mc1_init.h"
#include "measure.h"
#include "foc.h"
#include "board_service.h"
#include "generic_load.h"
#include "util.h"
#include "mc1_user_params.h"
#include "mc1_calc_params.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: MCAPP_MC1ParamsInit (MC1APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the Application data structure required for 
* controlling motor 1.
* @return none.
* 
* @example
* <CODE> MCAPP_MC1ParamsInit(&mc1); </CODE>
*
*/
void MCAPP_MC1ParamsInit(MC1APP_DATA_T *pMCData)
{    
    /* Reset all variables in the data structure to '0' */
    memset(pMCData,0,sizeof(MC1APP_DATA_T));

    pMCData->pControlScheme = &pMCData->controlScheme;
    pMCData->pMotorInputs = &pMCData->motorInputs;
    pMCData->pLoad = &pMCData->load;
    pMCData->pMotor = &pMCData->motor;
    pMCData->pPWMDuty = &pMCData->PWMDuty;
    pMCData->pSingleShunt = &pMCData->singleShunt;
    
    /* Configure Control Scheme */
    MCAPP_MC1ControlSchemeConfig(pMCData);
    
    /* Set motor control state as 'MTR_INIT' */
    pMCData->appState = MCAPP_INIT;
}

/**
* <B> Function: MCAPP_MC1ControlSchemeConfig (MC1APP_DATA_T *)  </B>
*
* @brief Function to configure the control scheme and parameters
*
* @param Pointer to the Application data structure required for 
* controlling motor 1.
* @return none.
* 
* @example
* <CODE> MCAPP_MC1ControlSchemeConfig(&mc1); </CODE>
*
*/
void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MCAPP_MEASURE_T *pMotorInputs;
    MCAPP_MOTOR_T *pMotor;
    MCAPP_LOAD_T *pLoad;

    pControlScheme = pMCData->pControlScheme;
    pMotorInputs = pMCData->pMotorInputs;
    pMotor = pMCData->pMotor;
    pLoad = pMCData->pLoad;
 
    /* Configure Inputs */  
#ifdef SINGLE_SHUNT
    SINGLE_SHUNT_PARM_T *pSingleShunt = pMCData->pSingleShunt;
    pControlScheme->pSingleShunt = pMCData->pSingleShunt;
    pControlScheme->singleShuntEnable = 1;
        
    pControlScheme->pIa = &pSingleShunt->Ia;
    pControlScheme->pIb = &pSingleShunt->Ib;
    pControlScheme->pIc = &pSingleShunt->Ic;
#else
    pControlScheme->singleShuntEnable = 0;
    
    pControlScheme->pIa = &pMotorInputs->measureCurrent.Ia_actual;
    pControlScheme->pIb = &pMotorInputs->measureCurrent.Ib_actual;
    pControlScheme->pIc = &pMotorInputs->measureCurrent.Ic_actual;
#endif
    pControlScheme->pVdc = &pMotorInputs->measureVdc.value;  
    pControlScheme->pMotor = pMCData->pMotor;
    pControlScheme->directionCmd = &pMCData->directionCmd;
    
    /* Initialize Motor parameters */
    pMotor->polePairs       = POLE_PAIRS;
    pMotor->Rs              = MOTOR_PER_PHASE_RESISTANCE;
    pMotor->Ls              = MOTOR_PER_PHASE_INDUCTANCE;
    pMotor->Ke              = MOTOR_BEMF_CONSTANT_ELEC;
    pMotor->NominalSpeed    = NOMINAL_SPEED_RPM;
    pMotor->MaxSpeed        = MAXIMUM_SPEED_RPM;
    pMotor->MaxOLSpeed      = MAX_OPENLOOP_SPEED_RPM;
    pMotor->MinSpeed        = MINIMUM_SPEED_RPM;
    pMotor->RatedCurrent    = NOMINAL_CURRENT_PEAK; 

    /* Initialize FOC control parameters */
#ifdef  OPEN_LOOP_FUNCTIONING
    pControlScheme->ctrlParam.openLoop = 1;
#else
    pControlScheme->ctrlParam.openLoop = 0;
#endif       
    pControlScheme->ctrlParam.lockTimeLimit = LOCK_TIME_COUNTS;
    pControlScheme->ctrlParam.lockVoltage = LOCKING_VOLTAGE;
    pControlScheme->ctrlParam.CLSpeedRampRate = CL_SPEED_REF_RAMP_VALUE;   
    pControlScheme->ctrlParam.currentRamp   = CURRENT_RAMP_VALUE;
    pControlScheme->ctrlParam.MaxVoltageSquare = MAX_VOLTAGE_SQUARE;
    
    /* Initialize fault parameters */
    pMCData->fault.overCurrentFaultLimit = OC_FAULT_LIMIT_PHASE;
    
    /* Initialize startup parameters */
    pControlScheme->startup.lockCurrent = LOCK_CURRENT;
    pControlScheme->startup.OLCurrent = OPEN_LOOP_CURRENT;
    pControlScheme->startup.OLCurrentMax = OPEN_LOOP_CURRENT;
    pControlScheme->startup.OLSpeedRampRate = OL_SPEED_REF_RAMP_VALUE;
    pControlScheme->startup.qDeltaT = DELTA_T_Q30;
    
    /* Initialize PI controller used for D axis current control */      
    pControlScheme->piId.param.kp = D_CURRCNTR_PTERM;       
    pControlScheme->piId.param.ki = D_CURRCNTR_ITERM;                 
    pControlScheme->piId.param.outMax = D_CURRCNTR_OUTMAX;
    pControlScheme->piId.param.outMin = -D_CURRCNTR_OUTMAX;
    pControlScheme->piId.stateVar.integrator = 0;

    /* Initialize PI controller used for Q axis current control */   
    pControlScheme->piIq.param.kp = Q_CURRCNTR_PTERM;       
    pControlScheme->piIq.param.ki = Q_CURRCNTR_ITERM;                  
    pControlScheme->piIq.param.outMax = Q_CURRCNTR_OUTMAX;
    pControlScheme->piIq.param.outMin = -Q_CURRCNTR_OUTMAX;
    pControlScheme->piIq.stateVar.integrator = 0;

    /* Initialize speed PI controller  */   
    pControlScheme->piSpeed.param.kp = SPEEDCNTR_PTERM;       
    pControlScheme->piSpeed.param.ki = SPEEDCNTR_ITERM;           
    pControlScheme->piSpeed.param.outMax = SPEEDCNTR_OUTMAX;   
    pControlScheme->piSpeed.param.outMin = -SPEEDCNTR_OUTMAX;
    pControlScheme->piSpeed.stateVar.integrator = 0;

    /* Initialize PLL Estimator */
    pControlScheme->estimator.pCtrlParam  = &pControlScheme->ctrlParam;
    pControlScheme->estimator.pIAlphaBeta = &pControlScheme->ialphabeta;
    pControlScheme->estimator.pVAlphaBeta = &pControlScheme->valphabeta;
    pControlScheme->estimator.pMotor      = pMCData->pMotor;
    pControlScheme->estimator.invKfiConst = ESTIM_INVERSE_BEMF_CONSTANT;  
    pControlScheme->estimator.qKfilterEsdq = KFILTER_ESDQ;
    pControlScheme->estimator.q30DeltaTs    = DELTA_T_Q30;
    pControlScheme->estimator.qOmegaFiltConst = KFILTER_VELESTIM;
    pControlScheme->estimator.thresholdSpeedBEMF = 100.0f;
    pControlScheme->estimator.thresholdSpeedDerivative = pMotor->NominalSpeed;
    pControlScheme->estimator.inverseDt = (float)(1.0f/MC1_LOOPTIME_SEC); 
    pControlScheme->estimator.DIlimitHS = D_ILIMIT_HS;
    pControlScheme->estimator.DIlimitLS = D_ILIMIT_LS;
    
    /* Initialize Estimator Interface */
    pControlScheme->estimatorInterface.pEstimPLL  = &pControlScheme->estimator;
    
    /* Initialize flux weakening parameters */
    pControlScheme->idRefGen.variant = FLUX_WEAKENING_VARIANT;
    pControlScheme->idRefGen.feedBackFW.pCtrlParam  = &pControlScheme->ctrlParam;
    pControlScheme->idRefGen.feedBackFW.pVdq       =  &pControlScheme->vdq;
    pControlScheme->idRefGen.feedBackFW.pMotor      = pMCData->pMotor;
    pControlScheme->idRefGen.feedBackFW.voltageMagRef = EFFECTIVE_VOLATGE_FW;
    pControlScheme->idRefGen.feedBackFW.FWeakPI.param.outMax = 0;
    pControlScheme->idRefGen.feedBackFW.FWeakPI.param.outMin = MAX_FW_NEGATIVE_ID_REF;
    pControlScheme->idRefGen.feedBackFW.FWeakPI.param.kp = FW_PTERM;
    pControlScheme->idRefGen.feedBackFW.FWeakPI.param.ki = FW_ITERM;    
    pControlScheme->idRefGen.feedBackFW.IdRefFiltConst = KFILTER_FW_IDREF;
    
    pControlScheme->idRefGen.feedForwardFW.pCtrlParam  = &pControlScheme->ctrlParam;
    pControlScheme->idRefGen.feedForwardFW.pMotor      = pMCData->pMotor;
    pControlScheme->idRefGen.feedForwardFW.IdRefMin    = MAX_FW_NEGATIVE_ID_REF;
    pControlScheme->idRefGen.feedForwardFW.voltageLimitFW = EFFECTIVE_VOLATGE_FW;
    pControlScheme->idRefGen.feedForwardFW.IdRefFiltConst = KFILTER_FW_IDREF;
    
    pControlScheme->idRefGen.ImaxSquare  = IMAX_SQUARE_FW;
    
    /* Configure load interface*/
    pLoad->minMechSpeedRPM = &pControlScheme->pMotor->MinSpeed;
    pLoad->mechSpeedRPM = &pControlScheme->estimatorInterface.speedMech.RPM; 
    
    /* Output Initializations */
    pControlScheme->pwmPeriod = (float)LOOPTIME_TCY; 
    pControlScheme->pPWMDuty = pMCData->pPWMDuty;
    
}

/**
* <B> Function: MCAPP_MC1LoadStartTransition (MCAPP_CONTROL_SCHEME_T *,
*                                                        MCAPP_LOAD_T *)  </B>
*
* @brief Function to handle the load transition (idle/windmill to start)
*
* @param Pointer to the FOC control scheme data structure
* @param Pointer to the FOC Load parameter data structure
* #return none.
* 
* @example
* <CODE> MCAPP_MC1LoadStartTransition(pControlScheme, pLoad);  </CODE>
*
*/
void MCAPP_MC1LoadStartTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pControlScheme->focState = FOC_INIT; 
    pLoad->state = GENERIC_LOAD_RUN;
}

/**
* <B> Function: MCAPP_MC1LoadStopTransition (MCAPP_CONTROL_SCHEME_T *,
*                                                        MCAPP_LOAD_T *)  </B>
*
* @brief Function to handle the load transition (running to stop)
*
* @param Pointer to the FOC control scheme data structure
* @param Pointer to the FOC Load parameter data structure
* #return none.
* 
* @example
* <CODE> MCAPP_MC1LoadStopTransition(pControlScheme, pLoad);  </CODE>
*
*/
void MCAPP_MC1LoadStopTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pLoad->state = GENERIC_LOAD_STOP;
}

