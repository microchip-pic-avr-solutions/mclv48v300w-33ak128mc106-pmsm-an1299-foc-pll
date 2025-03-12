// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file foc.c
 *
 * @brief This module implements Field Oriented Control(FOC).
 *
 * Component: FOC
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

#include "foc.h"
#include "foc_types.h"

#include "clarke_park.h"
#include "svm.h"

#include "estim_interface.h"
#include "estim_pll.h"

#include "mc1_calc_params.h"
#include "mc1_user_params.h"  

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *);
static void MCAPP_FOCForwardPath(MCAPP_FOC_T *);
static void MCAPP_ComputeOpenloopReference(MCAPP_OPENLOOPSTARTUP_T *, float );
static void MCAPP_CalculateModulationSiganl(MC_ABC_T *, MC_ABC_T *, float );
static void MCAPP_TransformRRFvariables(MCAPP_FOC_T *, int16_t , int16_t );
static void MCAPP_FOCRotorLock(MCAPP_FOC_T *);
inline static uint8_t ReadyforClosedLoopTransition(MCAPP_FOC_T *);

// </editor-fold>


/**
* <B> Function: void MCAPP_FOCInit(MCAPP_FOC_T *)  </B>
*
* @brief Executes FOC Parameters Initialization.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_FOCInit(pControlScheme); </CODE>
*
*/
void MCAPP_FOCInit(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;
    
    /* Initialize PI state parameters */
    MC_ControllerPIReset(&pFOC->piId, 0);
    MC_ControllerPIReset(&pFOC->piIq, 0);
    MC_ControllerPIReset(&pFOC->piSpeed,0);
    
    /* Initialize Estimator parameters */
    MCAPP_EstimatorInit(&pFOC->estimatorInterface); 
    
    /* Initialize flux weakening parameters */
    MCAPP_FluxWeakeningControlInit(&pFOC->idRefGen);
    
    pCtrlParam->lockTime = 0;
    pCtrlParam->speedRef = 0;
    pCtrlParam->idRef = 0;
    pCtrlParam->iqRef = 0;
    
    pFOC->faultStatus = 0;
    
    pFOC->pPWMDuty->dutycycle3 = 0;
    pFOC->pPWMDuty->dutycycle2 = 0;
    pFOC->pPWMDuty->dutycycle1 = 0;
    
    if(pFOC->singleShuntEnable == 1)
    {
        SingleShunt_InitializeParameters(pFOC->pSingleShunt);
        pFOC->pSingleShunt->phase.dutycycle1 = 0;
        pFOC->pSingleShunt->phase.dutycycle2 = 0;
        pFOC->pSingleShunt->phase.dutycycle3 = 0;
        pFOC->pSingleShunt->pdc.dutycycle1 = 0;
        pFOC->pSingleShunt->pdc.dutycycle2 = 0;
        pFOC->pSingleShunt->pdc.dutycycle3 = 0;
        SINGLE_SHUNT_TRIGGER1 = LOOPTIME_TCY/4;
        SINGLE_SHUNT_TRIGGER2 = LOOPTIME_TCY/2;
    }
}

/**
* <B> Function: void MCAPP_FOCStateMachine(MCAPP_FOC_T *)  </B>
*
* @brief Executes Speed and Current Control Loops and performs actions
*        associated with control state transition required for FOC.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_FOCStateMachine(&mc); </CODE>
*
*/
void MCAPP_FOCStateMachine(MCAPP_FOC_T *pFOC)
{

    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;
    uint8_t directionCmd =  *pFOC->directionCmd;

    switch (pFOC->focState)
    {
        case FOC_INIT:
            MCAPP_FOCInit(pFOC); 
            pFOC->focState = FOC_RTR_LOCK;
            break;
            
        case FOC_RTR_LOCK:
            MCAPP_FOCFeedbackPath(pFOC); 
            
            MCAPP_FOCRotorLock(pFOC); 

            if (pCtrlParam->lockTime < pCtrlParam->lockTimeLimit)
            {
                pCtrlParam->lockTime++;
            }
            else
            {
                pCtrlParam->lockTime = 0;

                pFOC->focState = FOC_OPEN_LOOP;
                /* Reset open loop parameters */
                pFOC->startup.OLThetaSum = 0; 
                MC_ControllerPIReset(&pFOC->piId, pFOC->vdq.d);
                MC_ControllerPIReset(&pFOC->piIq, pFOC->vdq.q);
                if(directionCmd)
                { 
                    pFOC->startup.OLCurrent = -(pFOC->startup.OLCurrentMax);
                }
                else
                {
                    pFOC->startup.OLCurrent = pFOC->startup.OLCurrentMax;               
                }      
            }            
            break;
        
        case FOC_OPEN_LOOP:
            
            MCAPP_FOCFeedbackPath(pFOC); 
            
            if(fabsf(pCtrlParam->speedRef) < pFOC->pMotor->MaxOLSpeed )
            {
                if(directionCmd)
                {
                    pCtrlParam->speedRef -= pFOC->startup.OLSpeedRampRate;
                }
                else
                {
                    pCtrlParam->speedRef += pFOC->startup.OLSpeedRampRate;
                }
                pFOC->OLrampFinished = 0;
            }
            else
            {
                pFOC->OLrampFinished = 1;
            }
                      
            MCAPP_ComputeOpenloopReference(&pFOC->startup, pCtrlParam->speedRef);
            
            pFOC->commutationAngle = pFOC->startup.OLtheta.radian ;
            
            pCtrlParam->iqRef = pFOC->startup.OLCurrent;
            pCtrlParam->idRef = 0;
            
            MCAPP_EstimatorStep(&pFOC->estimatorInterface); 

            MCAPP_FOCForwardPath(pFOC);
            
            if((pCtrlParam->openLoop == 0))
            {
                if(ReadyforClosedLoopTransition(pFOC))
                {
                    pFOC->focState = FOC_CLOSE_LOOP;
                    
                    /* Transform previous auxiliary DQ reference frame variables
                     * to new estimated DQ reference frame for seamless 
                     * transition to closed loop*/
                    MCAPP_TransformRRFvariables(pFOC, pFOC->estimator.qTheta, pFOC->startup.OLtheta.counts);

                    /* Reset speed PI controller */
                    MC_ControllerPIReset(&pFOC->piSpeed, 
                            (pFOC->ctrlParam.iqRef + pFOC->ctrlParam.iqRefOffset));   
                }
            }
            break;  
          
        case FOC_CLOSE_LOOP:
            
            MCAPP_FOCFeedbackPath(pFOC);
            
            /* Estimate Speed and rotor position */
            MCAPP_EstimatorStep(&pFOC->estimatorInterface); 
                      
            pFOC->commutationAngle  = pFOC->estimatorInterface.thetaEle.radian;
       
            /* Closed Loop Speed Ramp */
            float deltaSpeed = pCtrlParam->targetSpeed - pCtrlParam->speedRef;   
            if(deltaSpeed > pCtrlParam->CLSpeedRampRate)
            {
                pCtrlParam->speedRef = pCtrlParam->speedRef + 
                                                    pCtrlParam->CLSpeedRampRate;
            }
            else if(deltaSpeed < -pCtrlParam->CLSpeedRampRate)
            {
                pCtrlParam->speedRef = pCtrlParam->speedRef - 
                                                    pCtrlParam->CLSpeedRampRate;
            }
            else
            {
                pCtrlParam->speedRef = pCtrlParam->targetSpeed;
            }
            /* Generate Id Reference */
            MCAPP_FluxWeakeningControl(&pFOC->idRefGen);
            
            /* Execute Outer Speed Loop - Generate Iq Reference Generation */            
            pFOC->piSpeed.inMeasure = pFOC->estimatorInterface.speedMech.RPM;          
            pFOC->piSpeed.inReference = pCtrlParam->speedRef;
            pFOC->piSpeed.param.outMax = pFOC->idRefGen.iqMax;
            pFOC->piSpeed.param.outMin = -(pFOC->idRefGen.iqMax);
            MC_ControllerPIUpdate(&pFOC->piSpeed);
            pCtrlParam->iqRef = pFOC->piSpeed.output;      
            
            if(pCtrlParam->idRefOffset > pCtrlParam->currentRamp)
            {
                pCtrlParam->idRefOffset -= pCtrlParam->currentRamp;
            }
            else if(pCtrlParam->idRefOffset <- pCtrlParam->currentRamp)
            {
                pCtrlParam->idRefOffset += pCtrlParam->currentRamp;
            }
            else{
                pCtrlParam->idRefOffset = 0;
            }
            pCtrlParam->idRef = pFOC->idRefGen.idRef + pCtrlParam->idRefOffset;
 
            MCAPP_FOCForwardPath(pFOC);
            
            break;

        case FOC_FAULT:
                    
            break;
        
        default:
            pFOC->focState = FOC_FAULT;
            break;

    } /* End Of switch - case */
}

/**
* <B> Function: void MCAPP_FOCFeedbackPath (MCAPP_FOC_T *)  </B>
*
* @brief Function executing Clarke and Parks transforms for the feedback parameters
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_FOCFeedbackPath(&mc); </CODE>
*
*/
static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *pFOC)
{
    pFOC->iabc.a = *(pFOC->pIa);
    pFOC->iabc.b = *(pFOC->pIb);
    pFOC->iabc.c = *(pFOC->pIc);
    pFOC->vdc    = *(pFOC->pVdc);
    
    /* Perform Clark & Park transforms to generate d axis and q axis currents */
    MC_TransformClarke(&pFOC->iabc, &pFOC->ialphabeta);
    MC_TransformPark(&pFOC->ialphabeta, &pFOC->sincosTheta, &pFOC->idq);
}

/**
* <B> Function: void MCAPP_FOCForwardPath(MCAPP_FOC_T *)  </B>
*
* @brief Executes Current Control loops, Inverse Transformations and SVM
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCForwardPath(&mc); </CODE>
*
*/
static void MCAPP_FOCForwardPath(MCAPP_FOC_T *pFOC)
{
    float vqSquaredLimit, vdSquared;
    
    /** Execute inner current control loops */
    /* Execute PI Control of D axis. */
    pFOC->piId.inMeasure = pFOC->idq.d;          
    pFOC->piId.inReference = pFOC->ctrlParam.idRef;
    MC_ControllerPIUpdate(&pFOC->piId);
    pFOC->vdq.d = pFOC->piId.output;
    
    /* Generate Q axis current reference based on available voltage and D axis
       voltage */
    vdSquared  = pFOC->vdq.d * pFOC->vdq.d;
    vqSquaredLimit = pFOC->ctrlParam.MaxVoltageSquare - vdSquared;  

    pFOC->piIq.param.outMax = sqrt(vqSquaredLimit);
    pFOC->piIq.param.outMin = -(pFOC->piIq.param.outMax); 
    
    /* Execute PI Control of Q axis current. */  
    pFOC->piIq.inMeasure = pFOC->idq.q;          
    pFOC->piIq.inReference  = pFOC->ctrlParam.iqRef;  
    MC_ControllerPIUpdate(&pFOC->piIq);
    pFOC->vdq.q = pFOC->piIq.output;
    
    /* Calculate sin and cos of theta (angle) */		
    pFOC->sincosTheta.sin = sin(pFOC->commutationAngle);
    pFOC->sincosTheta.cos = cos(pFOC->commutationAngle); 

    /* Perform inverse Clarke and Park transforms and generate phase voltages.*/
    MC_TransformParkInverse(&pFOC->vdq, &pFOC->sincosTheta, &pFOC->valphabeta);
    MC_TransformClarkeInverseSwappedInput(&pFOC->valphabeta, &pFOC->vabc);
  
    /* Calculate modulation signal input for SVM */
    MCAPP_CalculateModulationSiganl(&pFOC->vabc, &pFOC->vabcModulation, pFOC->vdc);

    /* Execute space vector modulation and generate PWM duty cycles */
    if(pFOC->singleShuntEnable == 1)
    {
        pFOC->sectorSVM = SingleShunt_CalculateSpaceVectorPhaseShifted(&pFOC->vabcModulation, pFOC->pwmPeriod, pFOC->pSingleShunt);
    }    
    else
    {
        pFOC->sectorSVM = MC_CalculateSpaceVectorPhaseShifted(&pFOC->vabcModulation, pFOC->pwmPeriod, pFOC->pPWMDuty);        
    }    
}

/**
* <B> Function: void MCAPP_CalculateModulationSiganl(MC_ABC_T *,MC_ABC_T *,float)  </B>
*
* @brief function for DC bus voltage compensation
*
* @param Pointer to the data structure containing phase voltage parameters input
* @param Pointer to the data structure containing phase voltage parameters output
* @param float variable for DC bus
* @return none.
* 
* @example
* <CODE>  MCAPP_CalculateModulationSiganl(&pFOC->vabc, &pFOC->vabcModulation, pFOC->vdc); </CODE>
*
*/
static void MCAPP_CalculateModulationSiganl(MC_ABC_T *pabcIn, MC_ABC_T *pvabcOut, float vdc)
{    
    pvabcOut->a = pabcIn->a*SQRT_3/vdc;
    pvabcOut->b = pabcIn->b*SQRT_3/vdc;
    pvabcOut->c = pabcIn->c*SQRT_3/vdc;
}

/**
* <B> Function: void MCAPP_ComputeOpenloopReference(MCAPP_OPENLOOPSTARTUP_T *, float)  </B>
*
* @brief Compute open-loop current and position reference
*
* @param Pointer to the data structure containing Open-loop startup parameters.
* @param speed reference.
* @return none.
* 
* @example
* <CODE> MCAPP_ComputeOpenloopReference(&startup, speedRef ) </CODE>
*
*/
static void MCAPP_ComputeOpenloopReference(MCAPP_OPENLOOPSTARTUP_T *pOL, float speedRef)
{
    
    /* Calculate open loop theta reference */     
    pOL->OLThetaSum += (int32_t)( speedRef * pOL->qDeltaT);
    pOL->OLtheta.counts = (int16_t)(pOL->OLThetaSum >>15);
    pOL->OLtheta.radian  = ((float)pOL->OLtheta.counts)* Q15_TO_RADIAN;
}

/**
* <B> Function: void MCAPP_TransformRRFvariables(MCAPP_FOC_T *, int16_t, int16_t)  </B>
*
* @brief Transform previous auxiliary DQ reference frame variables to new estimated 
* DQ reference frame for seamless transition to closed loop
*
* @param Pointer to the data structure containing FOC parameters
* @param angle from estimator.
* @param angle from open loop start-up
* @return none.
* 
* @example
* <CODE> MCAPP_TransformRRFvariables(pFOC, pFOC->estimator.qTheta, pFOC->startup.OLtheta.counts); </CODE>
*
*/
static void MCAPP_TransformRRFvariables(MCAPP_FOC_T *pFOC, 
                        int16_t newRRFangleCount, int16_t prevRRFangleCount)
{
    float newRRFangle, id_new, iq_new, vd_new, vq_new;
    
    newRRFangle = ((float)(newRRFangleCount ) *M_PI/32767.0f); 
    
    /* Update sin and cos of theta (angle) */		
    pFOC->sincosTheta.sin = sin(newRRFangle);
    pFOC->sincosTheta.cos = cos(newRRFangle);

    /* Calculate angle difference between previous auxiliary DQ frame 
     * and new actual DQ frame */
    int16_t deltaTheta = prevRRFangleCount - newRRFangleCount;
    float thetaOffset = ((float)deltaTheta)*M_PI/32767.0f;

    /* Transform previous auxiliary DQ frame variables to new actual DQ frame */
    id_new = pFOC->idq.d * cos(thetaOffset) - pFOC->idq.q * sin(thetaOffset);
    iq_new = pFOC->idq.d * sin(thetaOffset) + pFOC->idq.q * cos(thetaOffset);
    vd_new = pFOC->vdq.d * cos(thetaOffset) - pFOC->vdq.q * sin(thetaOffset);
    vq_new = pFOC->vdq.d * sin(thetaOffset) + pFOC->vdq.q * cos(thetaOffset);

    /* Reset current PI controllers to new DQ frame values*/
    MC_ControllerPIReset(&pFOC->piId, vd_new);
    MC_ControllerPIReset(&pFOC->piIq, vq_new);

    /*Calculate offset for Iq & Id reference values */
    pFOC->ctrlParam.iqRefOffset = iq_new - pFOC->ctrlParam.iqRef;
    pFOC->ctrlParam.idRefOffset = id_new - pFOC->ctrlParam.idRef;
}

/**
* <B> Function: void MCAPP_FOCRotorLock (MCAPP_FOC_T *)  </B>
*
* @brief Function to calculate duty cycles for rotor locking
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_FOCRotorLock(&mc); </CODE>
*
*/
static void MCAPP_FOCRotorLock(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;
    
    pFOC->commutationAngle = 0;           

    /* Calculate sin and cos of theta (angle) */		
    pFOC->sincosTheta.sin = sin(pFOC->commutationAngle);
    pFOC->sincosTheta.cos = cos(pFOC->commutationAngle); 
    pFOC->vdq.d = pCtrlParam->lockVoltage;
    pFOC->vdq.q = 0;
    /* Perform inverse Clarke and Park transforms and generate phase voltages.*/
    MC_TransformParkInverse(&pFOC->vdq, &pFOC->sincosTheta, &pFOC->valphabeta);
    MC_TransformClarkeInverseSwappedInput(&pFOC->valphabeta, &pFOC->vabc); 
    MCAPP_CalculateModulationSiganl(&pFOC->vabc, &pFOC->vabcModulation, pFOC->vdc);

    /* Execute space vector modulation and generate PWM duty cycles */
    if(pFOC->singleShuntEnable == 1)
    {
        pFOC->sectorSVM = SingleShunt_CalculateSpaceVectorPhaseShifted(&pFOC->vabcModulation, pFOC->pwmPeriod, pFOC->pSingleShunt);
    }    
    else
    {
        pFOC->sectorSVM = MC_CalculateSpaceVectorPhaseShifted(&pFOC->vabcModulation, pFOC->pwmPeriod, pFOC->pPWMDuty);        
    }  

}

/**
* <B> Function: inline static uint8_t ReadyforClosedLoopTransition(MCAPP_FOC_T *pFOC)  </B>
*
* @brief Function to check the open loop to closed loop transition
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* 
* @example
* <CODE> ReadyforClosedLoopTransition(&mc); </CODE>
*
*/
inline static uint8_t ReadyforClosedLoopTransition(MCAPP_FOC_T *pFOC)
{
    uint8_t readyStatus;
    
    if(pFOC->OLrampFinished == 1)
    {
        if(fabs(pFOC->estimator.omegaFilt)> pFOC->pMotor->MaxOLSpeed)
        {
            readyStatus = 1;
        }
        else
        {
            readyStatus = 0;
        }
    }
    else
    {
        readyStatus = 0;
    } 
    
    return readyStatus;
}