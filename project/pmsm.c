// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
* @file pmsm.c
*
* @brief This is the main entry to the application.
*
* Component: APPLICATION
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
 
#include <xc.h>
#include <stdio.h>
#include <math.h>

#include "userparms.h"
#include "board_service.h"
#include "control.h"
#include "estim.h"
#include "pi.h"
#include "svm.h"
#include "clarke_park.h"
#include "fdweak.h"
#include "diagnostics.h"
#include "singleshunt.h"
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc=" Global Variables ">
 
MCAPP_MEASURE_T measureInputs;
CTRL_PARM_T CtrlParm;
MOTOR_STARTUP_DATA_T motorStartUpData;
UGF_T uGF;

MC_PIPARMIN_T     piInputIq;
MC_PIPARMOUT_T    piOutputIq;
MC_PIPARMIN_T     piInputId;   
MC_PIPARMOUT_T    piOutputId;
MC_PIPARMIN_T     piInputOmega;    
MC_PIPARMOUT_T    piOutputOmega;
MC_SINCOS_T sincosTheta;
MC_ALPHABETA_T valphabeta,ialphabeta;
MC_DQ_T vdq,idq;
MC_ABC_T vabc,iabc;
MC_DUTYCYCLEOUT_T pwmDutycycle;
SINGLE_SHUNT_PARM_T singleShuntParam;

float pwmPeriod;
float thetaElectricalOpenLoop;
int32_t heartBeatCount = 0;
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc=" Function Declarations ">

void CalculateParkAngle(void);
void DoControl( void );
void InitControlParameters(void);
void ResetParameters(void);
void ResetPeripherals(void);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: int main (void)  </B>
*
* @brief main() function,entry point of the application.
*
*/
int main(void) 
{
    InitOscillator();       
    SetupGPIOPorts();
    
    /* LED1 is turned on here, and toggled in Timer1 Interrupt */
    LED1 = 1; 
    
    /* Reset the Control Parameters */
    ResetParameters();
    /* Initialize Diagnostics and Board Service */
    DiagnosticsInit();
    BoardServiceInit();
    
    /* Initialize Peripherals */
    InitPeripherals();
    
    ResetPeripherals();

    while(1)
    {
        DiagnosticsStepMain();
        BoardService();

        if (IsPressed_Button1())
        {
            if  ((uGF.bits.RunMotor == 1) || (PWM_FAULT_STATUS == 1))
            {
                ResetParameters();
                ResetPeripherals();
            }
            else
            {
                EnablePWMOutputs();
                uGF.bits.RunMotor = 1;
            }
        }
        if(IsPressed_Button2())
        {
            if ((uGF.bits.RunMotor == 1) && (uGF.bits.OpenLoop == 0))
            {
                uGF.bits.ChangeSpeed = !uGF.bits.ChangeSpeed;
            }
        }
        /* LED2 is used as motor run Status */
        LED2 = uGF.bits.RunMotor;
    } 
}
/**
* <B> Function: ADCInterrupt() </B>
*
* @brief Does speed calculation and executes the vector update loop.
* The ADC sample and conversion is triggered by the PWM period.
* The speed calculation assumes a fixed time interval between calculations.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ADCInterrupt(); </CODE>
*
*/
void __attribute__((__interrupt__, no_auto_psv))ADCInterrupt(void)
{   

#ifdef SINGLE_SHUNT
    volatile int16_t ibusTemp1 = 0,ibusTemp2 = 0;
    ibusTemp1 = (ADCBUF_IBUS1 - measureInputs.current.offsetIbus);
    ibusTemp2 = (ADCBUF_IBUS2 - measureInputs.current.offsetIbus);
#else
    measureInputs.current.Ia = ADCBUF_IA ;
    measureInputs.current.Ib = ADCBUF_IB ;
    MCAPP_MeasureCurrentCalibrate(&measureInputs);    
#endif
    
    /* if run motor command is activated */    
    if(uGF.bits.RunMotor == 1)  
    {
#ifdef SINGLE_SHUNT
        /*Convert ADC Counts to real value*/
        singleShuntParam.Ibus1 = (float)(ibusTemp1 * ADC_CURRENT_SCALE);
        singleShuntParam.Ibus2 = (float)(ibusTemp2 * ADC_CURRENT_SCALE);
        /* Reconstruct Phase currents from Bus Current*/                
        SingleShunt_PhaseCurrentReconstruction(&singleShuntParam);
        
        iabc.a = singleShuntParam.Ia;
        iabc.b = singleShuntParam.Ib;
#else        
        /*Convert ADC Counts to real value*/
        iabc.a = (float)(measureInputs.current.Ia * ADC_CURRENT_SCALE); 
        iabc.b = (float)(measureInputs.current.Ib * ADC_CURRENT_SCALE);
#endif        
        MC_TransformClarke(&iabc,&ialphabeta);
        MC_TransformPark(&ialphabeta,&sincosTheta,&idq);
        
        Estim();
        
        DoControl();

		CalculateParkAngle();
        
        /* if open loop */
		if(uGF.bits.OpenLoop == 1)
		{
		    /* the angle is generated by open loop ramp up */
            sincosTheta.sin = sin(thetaElectricalOpenLoop);
            sincosTheta.cos = cos(thetaElectricalOpenLoop);              
		} 
        else
		{
			/* if closed loop, angle is generated by estimator */
            sincosTheta.sin = sin(EstimParm.Rho + EstimParm.RhoOffset);
            sincosTheta.cos = cos(EstimParm.Rho + EstimParm.RhoOffset);                           
        }
          
        MC_TransformParkInverse(&vdq,&sincosTheta,&valphabeta); 
        MC_TransformClarkeInverseSwappedInput(&valphabeta,&vabc);
        
#ifdef SINGLE_SHUNT
        SingleShunt_CalculateSpaceVectorPhaseShifted(&vabc,pwmPeriod,
                                                            &singleShuntParam);
        PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,
                                               &singleShuntParam.pwmDutycycle2);
#else
        MC_CalculateSpaceVectorPhaseShifted(&vabc,pwmPeriod,
                                                        &pwmDutycycle);
        PWMDutyCycleSet(&pwmDutycycle);
#endif        
    }
    else
    {
        /* if run motor command is not activated */   
        
        PWM_TRIGA = ADC_SAMPLING_POINT;
#ifdef SINGLE_SHUNT
        SINGLE_SHUNT_TRIGGER1 = LOOPTIME_TCY>>2;
        SINGLE_SHUNT_TRIGGER2 = LOOPTIME_TCY>>1;
        PWM_PHASE3 = MIN_DUTY;
        PWM_PHASE2 = MIN_DUTY;
        PWM_PHASE1 = MIN_DUTY;
#endif
        PWM_PDC3  = MIN_DUTY;
        PWM_PDC2  = MIN_DUTY;
        PWM_PDC1  = MIN_DUTY;
    }
    
    /* if run motor command is not activated */ 
    if (uGF.bits.RunMotor == 0)
    {
        measureInputs.current.Ia = ADCBUF_IA;
        measureInputs.current.Ib = ADCBUF_IB;
        measureInputs.current.Ibus = ADCBUF_IBUS1;
    }
       
    if (MCAPP_MeasureCurrentOffsetStatus(&measureInputs) == 0)
    {
        MCAPP_MeasureCurrentOffset(&measureInputs);
    }
    else
    {
        BoardServiceStepIsr(); 
    }
    
    measureInputs.potValue = ADCBUF_POT;
    DiagnosticsStepIsr();
   
    ADCInterruptFlagClear;
}
/**
* <B> Function: DoControl()     </B>
*
* @brief This routine executes one PI iteration for each of the three loops
*        Id,Iq,Speed
*        
* @param none.
* @return none.
* 
* @example
* <CODE> DoControl();        </CODE>
*
*/
void DoControl(void)
{
    /*Temporary variables for sqrt calculation of q reference*/
    float squaredVd,squaredVq;
    
    if( uGF.bits.OpenLoop )
    {
        /* OPENLOOP:  force rotating angle,Vd,Vq */
        if( uGF.bits.ChangeMode )
        {
            /* Just changed to open loop */
            uGF.bits.ChangeMode = 0;
            
            /* Synchronize angles */
            /* VqRef & VdRef not used */
            CtrlParm.vqRef = 0.0;
            CtrlParm.vdRef = 0.0;

			/* Reinitialize variables for initial speed ramp */
			motorStartUpData.startupLock  = 0.0;
			motorStartUpData.startupRamp = 0.0;            
        }
        /*D axis PI control input*/
        CtrlParm.vdRef = 0.0;
        
        /* PI control for D axis current in Open Loop */
        piInputId.inMeasure = idq.d/PEAK_CURRENT;
        piInputId.inReference  = CtrlParm.vdRef;
        MC_ControllerPIUpdate(&piInputId,
                                       &piInputId.piState,
                                       &piOutputId);        
        vdq.d = piOutputId.out;
        
        /* dynamic d-q adjustment */
        /* with d component priority */
        /* vq=sqrt (vs^2 - vd^2) */
        /* limit vq maximum to the one resulting from the calculation above */
        squaredVd = piOutputId.out * piOutputId.out;
        squaredVq = (MAX_VOLTAGE_VECTOR - squaredVd);
        piInputIq.piState.outMax = sqrt(squaredVq);

        /* Q axis PI control input */
        CtrlParm.vqRef    = IQ_CURRENT_REF_OPENLOOP/PEAK_CURRENT;       
       	
        /* PI control for Q axis current in Open Loop */
        piInputIq.inMeasure = idq.q/PEAK_CURRENT;
        piInputIq.inReference  = CtrlParm.vqRef;
        MC_ControllerPIUpdate(&piInputIq,
                                       &piInputIq.piState,
                                       &piOutputIq);
        vdq.q = piOutputIq.out;

    } 
    else
    /* Closed Loop Vector Control */
	{
        if(uGF.bits.ChangeSpeed)
        {
            /* Potentiometer value is scaled between NOMINAL SPEED and 
             * FIELD WEAKENING SPEED to set the speed reference*/
            CtrlParm.targetSpeed = ((measureInputs.potValue*
              (FW_SPEED_RAD_PER_SEC_ELEC - NOMINAL_SPEED_RAD_PER_SEC_ELEC)/MAX_POT_COUNT)+
                                                NOMINAL_SPEED_RAD_PER_SEC_ELEC);                  
        }
        else
        {
            /* Potentiometer value is scaled between END SPEED and 
             * NOMINAL SPEED to set the speed reference*/
            CtrlParm.targetSpeed = ((measureInputs.potValue*
                (NOMINAL_SPEED_RAD_PER_SEC_ELEC - END_SPEED_RADS_SEC_ELEC)/
                    MAX_POT_COUNT)+ END_SPEED_RADS_SEC_ELEC);
             CtrlParm.vdRef = 0; 
        }
        
        if  (CtrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           CtrlParm.speedRampCount++; 
        }
        else
        {
            /* Ramp generator to limit the change of the speed reference
              the rate of change is defined by CtrlParm.qRefRamp */
            CtrlParm.diffSpeed = CtrlParm.VelRef - CtrlParm.targetSpeed;
            /* Speed Ref Ramp */
            if (CtrlParm.diffSpeed < 0.0)
            {
                /* Set this cycle reference as the sum of
                previously calculated one plus the reference ramp value */
                CtrlParm.VelRef = CtrlParm.VelRef+CtrlParm.refRamp;
            }
            else
            {
                /* Same as above for speed decrease */
                CtrlParm.VelRef = CtrlParm.VelRef-CtrlParm.refRamp;
            }
            /* If difference less than half of ref ramp, set reference
            directly from the pot */
            if (fabsf(CtrlParm.diffSpeed) < (CtrlParm.refRamp/2.0))
            {
                CtrlParm.VelRef = CtrlParm.targetSpeed;
            }
            CtrlParm.speedRampCount = 0.0;
        }        

        if( uGF.bits.ChangeMode )
        {
            /* Just changed from open loop */
            uGF.bits.ChangeMode = 0.0;
			piInputOmega.piState.integrator= CtrlParm.vqRef/PEAK_CURRENT;
            CtrlParm.VelRef = END_SPEED_RADS_SEC_ELEC;    
	    }             
        

/* if TORQUE MODE, skip the speed controller */                
#ifndef	TORQUE_MODE

        /* Execute the velocity control loop */   
		piInputOmega.inMeasure = EstimParm.VelEstim;
    	piInputOmega.inReference  = CtrlParm.VelRef;
        MC_ControllerPIUpdate(&piInputOmega,
                                       &piInputOmega.piState,
                                       &piOutputOmega);
    	CtrlParm.vqRef = piOutputOmega.out/PEAK_CURRENT;
       
#else
        /*Temporary variable to calculate Q axis current reference (actual amps) */
        float iqRefActual = 0;        
        /* D axis current reference set to zero*/
        CtrlParm.vdRef = 0;
        /* Q axis current reference set by the POT (in actual amps) */
        iqRefActual = (((float)measureInputs.potValue)*MAX_MOTOR_CURRENT)/4095.0;
        /*Q axis current reference in per unit ( base : PEAK_CURRENT) */
        CtrlParm.vqRef = iqRefActual/PEAK_CURRENT; 
        /* Min and Max limit for Q axis current reference in Torque Mode*/
        if( CtrlParm.vqRef < MIN_IQ_REF_PU)
            {
               CtrlParm.vqRef  = MIN_IQ_REF_PU;
            }
        if( CtrlParm.vqRef > MAX_IQ_REF_PU)
            {
               CtrlParm.vqRef  = MAX_IQ_REF_PU;
            }
#endif
        /* Flux weakening control - the actual speed is replaced 
        with the reference speed for stability 
        reference for d current component 
        adapt the estimator parameters in concordance with the speed */
        CtrlParm.vdRef = FieldWeakening(CtrlParm.VelRef, CtrlParm.vqRef );  
        piInputOmega.piState.outMax = (sqrt((SPEEDCNTR_OUTMAX)*(SPEEDCNTR_OUTMAX) - CtrlParm.vdRef*CtrlParm.vdRef*PEAK_CURRENT*PEAK_CURRENT));
        /* PI control for D axis current in Closed Loop */ 
        piInputId.inMeasure = idq.d/PEAK_CURRENT;          
        piInputId.inReference  = CtrlParm.vdRef;      
        MC_ControllerPIUpdate(&piInputId,
                                       &piInputId.piState,
                                       &piOutputId);
        vdq.d    = piOutputId.out   ; 

        /* dynamic d-q adjustment */
        /* with d component priority */
        /* vq=sqrt (vs^2 - vd^2) */
        /* limit vq maximum to the one resulting from the calculation above */
        squaredVd = piOutputId.out * piOutputId.out;
        squaredVq = (MAX_VOLTAGE_VECTOR - squaredVd);
        piInputIq.piState.outMax = sqrt(squaredVq);           
        piInputIq.piState.outMin = - piInputIq.piState.outMax;
        
        /* PI control for Q axis current in Closed Loop */ 
        piInputIq.inMeasure = idq.q/PEAK_CURRENT;          
        piInputIq.inReference  = CtrlParm.vqRef;  
        MC_ControllerPIUpdate(&piInputIq,
                                       &piInputIq.piState,
                                       &piOutputIq);
        vdq.q    = piOutputIq.out ;       
    }
}
/**
* <B> Function: CalculateParkAngle()     </B>
*
* @brief Function calculates the angle for open loop control.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> CalculateParkAngle();        </CODE>
*
*/
void CalculateParkAngle(void)
{
    /* if open loop */
	if(uGF.bits.OpenLoop)	
	{
		/* begin with the lock sequence, for field alignment */
		if (motorStartUpData.startupLock < LOCK_COUNT_FOR_LOCK_TIME)
        {
			motorStartUpData.startupLock += 1;
        }
	    /* then ramp up till the end speed */
		else if (motorStartUpData.startupRamp <= END_SPEED_RADS_SEC_ELEC_IN_LOOPTIME)
        {
            motorStartUpData.startupRamp += OPENLOOP_RAMPSPEED_INCREASERATE;
        }
		else /* switch to closed loop */
		{
#ifndef OPEN_LOOP_FUNCTIONING
            uGF.bits.ChangeMode = 1;
            uGF.bits.OpenLoop = 0;
#endif
        }
	
		/* the angle set depends on startup ramp */
		thetaElectricalOpenLoop += motorStartUpData.startupRamp;
        if(thetaElectricalOpenLoop >= M_PI)
          {
              thetaElectricalOpenLoop = -M_PI + (thetaElectricalOpenLoop - M_PI);
          }
    
	}
	else /* switched to closed loop */
	{
   	    /* in closed loop slowly decrease the offset add to */
   	    /* the estimated angle */
   	    if(EstimParm.RhoOffset > (M_PI/MAX_COUNT))
            EstimParm.RhoOffset = EstimParm.RhoOffset - ((M_PI/MAX_COUNT)) ;
	}
}
/**
* <B> Function: InitControlParameters()     </B>
*
* @brief Function initializes the control parameters :
* PI coefficients, scaling constants etc.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitControlParameters();        </CODE>
*
*/
void InitControlParameters(void)
{
    /*Closed Loop Speed Ramp Control*/
    CtrlParm.refRamp = SPEEDREFRAMP;
    CtrlParm.speedRampCount = SPEEDREFRAMP_COUNT;
    
    /* Set PWM period to Loop Time */
    pwmPeriod = (float)LOOPTIME_TCY;
    
    /* PI - Id Current Control */     
    piInputId.piState.kp = D_CURRCNTR_PTERM;       
    piInputId.piState.ki = D_CURRCNTR_ITERM;              
    piInputId.piState.kc = D_CURRCNTR_CTERM;       
    piInputId.piState.outMax = D_CURRCNTR_OUTMAX;
    piInputId.piState.outMin = -piInputId.piState.outMax;
    piInputId.piState.integrator = 0;
    piOutputId.out = 0;

    /* PI - Iq Current Control */
    piInputIq.piState.kp = Q_CURRCNTR_PTERM;    
    piInputIq.piState.ki = Q_CURRCNTR_ITERM;
    piInputIq.piState.kc = Q_CURRCNTR_CTERM;
    piInputIq.piState.outMax = Q_CURRCNTR_OUTMAX;
    piInputIq.piState.outMin = 0;
    piInputIq.piState.integrator = 0;
    piOutputIq.out = 0;

    /* PI - Speed Control */
    piInputOmega.piState.kp = SPEEDCNTR_PTERM;       
    piInputOmega.piState.ki = SPEEDCNTR_ITERM;       
    piInputOmega.piState.kc = SPEEDCNTR_CTERM;       
    piInputOmega.piState.outMax = SPEEDCNTR_OUTMAX;   
    piInputOmega.piState.outMin = 0;
    piInputOmega.piState.integrator = 0;
    piOutputOmega.out = 0;

}
/**
* <B> Function: ResetParameters()     </B>
*
* @brief Function resets the control parameters: ADC trigger points,Duty cycle,
* PI coefficients, scaling constants etc.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ResetParameters();        </CODE>
*
*/
void ResetParameters(void)
{
    /* Stop the motor   */
    uGF.bits.RunMotor = 0;        
    /* Set the reference speed value to 0 */
    CtrlParm.VelRef = 0.0;
    /* Restart in open loop */
    uGF.bits.OpenLoop = 1;
    //SincosParm.openLoop = 1;
    /* Change speed */
    uGF.bits.ChangeSpeed = 0;
    /* Change mode */
    uGF.bits.ChangeMode = 1;
    /* Initialize PI control parameters */
    InitControlParameters();        
    /* Initialize estimator parameters */
    InitEstimParm();
    /* Initialize flux weakening parameters */
    InitFWParams();
}
/**
* <B> Function: ResetPeripherals()     </B>
*
* @brief Function to resets the peripherals ; ADC trigger points,Duty cycle
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ResetPeripherals();        </CODE>
*
*/
void ResetPeripherals(void)
{
    /* Make sure ADC does not generate interrupt while initializing parameters*/
	DisableADCInterrupt();
#ifdef SINGLE_SHUNT
    /* Initialize Single Shunt Related parameters */
    SingleShunt_InitializeParameters(&singleShuntParam);
    SINGLE_SHUNT_TRIGGER1 = LOOPTIME_TCY>>2;
    SINGLE_SHUNT_TRIGGER2 = LOOPTIME_TCY>>1;
    PWM_PHASE3 = MIN_DUTY;
    PWM_PHASE2 = MIN_DUTY;
    PWM_PHASE1 = MIN_DUTY;
#endif
    PWM_TRIGA = ADC_SAMPLING_POINT;
    /* Re initialize the duty cycle to minimum value */
    PWM_PDC3 = MIN_DUTY;
    PWM_PDC2 = MIN_DUTY;
    PWM_PDC1 = MIN_DUTY;
    
    DisablePWMOutputs();
    
    /* Initialize measurement parameters */
    MCAPP_MeasureCurrentInit(&measureInputs);
    /* Enable ADC interrupt and begin main loop timing */
    ADCInterruptFlagClear;
    EnableADCInterrupt();
}
/**
* <B> Function: _PWMInterrupt()     </B>
*
* @brief Function to handle PWM Fault Interrupt from PCI
*        
* @param none.
* @return none.
* 
* @example
* <CODE> _PWMInterrupt();        </CODE>
*
*/
void __attribute__((__interrupt__,no_auto_psv)) _PWMInterrupt()
{
    ResetParameters();
    ResetPeripherals();
    ClearPWMPCIFault();
    ClearPWMIF(); 
}
/**
* <B> Function: _T1Interrupt()     </B>
*
* @brief Function to handle Timer1 Interrupt. Timer1 is configured for 1 ms.
* LED1 is toggled at a rate of 250 ms as a Heartbeat LED.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> _T1Interrupt();        </CODE>
*
*/
void __attribute__((__interrupt__, no_auto_psv))_T1Interrupt(void)
{
    
    if (heartBeatCount < HEART_BEAT_LED_COUNT)
    {
        heartBeatCount += 1;
    }
    else
    {
        heartBeatCount = 0;
        if(LED1 == 1)
        {
            LED1 = 0;
        }
        else
        {
            LED1 = 1;
        }
    }
    
    TIMER1_InterruptFlagClear();
}
// </editor-fold>

