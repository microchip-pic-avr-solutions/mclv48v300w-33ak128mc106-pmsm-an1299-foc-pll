// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file board_service.c
 *
 * @brief This module implements the board service routines 
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: BOARD SERVICE
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
#include <stdint.h>
#include <stdbool.h>

#include "board_service.h"
#include "userparms.h"
#include "delay.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">
BUTTON_T buttonStartStop;
BUTTON_T buttonSpeedHalfDouble;

uint16_t boardServiceISRCounter = 0;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T * ,bool);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: IsPressed_Button1() </B>
*
* @brief Function monitors the actuation of push button
*        
* @param none.
* @return boolean for button actuation.
* 
* @example
* <CODE> IsPressed_Button1(); </CODE>
*
*/
bool IsPressed_Button1(void)
{
    if (buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

/**
* <B> Function: IsPressed_Button2() </B>
*
* @brief Function monitors the actuation of push button
*        
* @param none.
* @return boolean for button actuation.
* 
* @example
* <CODE> IsPressed_Button2(); </CODE>
*
*/
bool IsPressed_Button2(void)
{
    if (buttonSpeedHalfDouble.status)
    {
        buttonSpeedHalfDouble.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

/**
* <B> Function: BoardServiceStepIsr() </B>
*
* @brief Function to increment the board service routine counter
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardServiceStepIsr(); </CODE>
*
*/
void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter <  BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}

/**
* <B> Function: BoardService() </B>
*
* @brief Function to reset the board service routine counter 
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardService(); </CODE>
*
*/
void BoardService(void)
{
    if (boardServiceISRCounter ==  BOARD_SERVICE_TICK_COUNT)
    {
        /* Button scanning loop for Button 1 to start Motor A */
        ButtonScan(&buttonStartStop,BUTTON_START_STOP);

        /* Button scanning loop for SW2 to enter into filed
            weakening mode */
        ButtonScan(&buttonSpeedHalfDouble,BUTTON_SPEED_HALF_DOUBLE);

        boardServiceISRCounter = 0;
    }
}

/**
* <B> Function: BoardServiceInit() </B>
*
* @brief Function to initialize the board service routine
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardServiceInit(); </CODE>
*
*/
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

/**
* <B> Function: ButtonScan() </B>
*
* @brief Function to monitor the push button actuation with de-bounce delay
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ButtonScan(); </CODE>
*
*/
void ButtonScan(BUTTON_T *pButton,bool button) 
{
    if (button == true) 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } 
    else 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->state = BUTTON_NOT_PRESSED;
        } 
        else 
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}

/**
* <B> Function: ButtonGroupInitialize() </B>
*
* @brief Function to initialize the push button states
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ButtonGroupInitialize(); </CODE>
*
*/
void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.state = false;

    buttonSpeedHalfDouble.state = BUTTON_NOT_PRESSED;
    buttonSpeedHalfDouble.debounceCount = 0;
    buttonSpeedHalfDouble.state = false;

}

/**
* <B> Function: InitPeripherals() </B>
*
* @brief Function to initialize the peripherals(Op-Amp, ADC and PMW)
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPeripherals(); </CODE>
*
*/
void InitPeripherals(void)
{
#ifdef INTERNAL_OPAMP_CONFIG    
    OpampConfig();
#endif
    
    InitializeADCs();
    /*400ms POR delay for IBUS_EXT signal coming from MCP651S in Dev Board*/
    __delay_ms(400);
    
    InitPWMGenerators(); 
    
    /*Clearing and enabling PWM Interrupt to handle PCI Fault*/
    ClearPWMIF();
    EnablePWMIF();
    
    TIMER1_Initialize();
    TIMER1_InputClockSet();
    TIMER1_PeriodSet(TIMER1_PERIOD_COUNT);
    TIMER1_InterruptPrioritySet(5);
    TIMER1_InterruptFlagClear();
    TIMER1_InterruptEnable(); 
    TIMER1_ModuleStart();
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    DisableADCInterrupt();  
}

/**
* <B> Function: DisablePWMOutputs() </B>
*
* @brief Function to disable the PWM outputs by overriding to zero
*        
* @param none.
* @return none.
* 
* @example
* <CODE> DisablePWMOutputs(); </CODE>
*
*/
void DisablePWMOutputs(void)
{
    /** Set Override Data on all PWM outputs */
    /* 0b00 = State for PWM3H,L, if Override is Enabled */
    PG3IOCONbits.OVRDAT = 0;
    /* 0b00 = State for PWM2H,L, if Override is Enabled */
    PG2IOCONbits.OVRDAT = 0; 
    /* 0b00 = State for PWM1H,L, if Override is Enabled */
    PG1IOCONbits.OVRDAT = 0;  

    /* 1 = OVRDAT<1> provides data for output on PWM3H */
    PG3IOCONbits.OVRENH = 1; 
    /* 1 = OVRDAT<0> provides data for output on PWM3L */
    PG3IOCONbits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM2H */
    PG2IOCONbits.OVRENH = 1;
    /* 1 = OVRDAT<0> provides data for output on PWM2L */
    PG2IOCONbits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM1H */
    PG1IOCONbits.OVRENH = 1;  
    /* 1 = OVRDAT<0> provides data for output on PWM1L */
    PG1IOCONbits.OVRENL = 1;     
}

/**
* <B> Function: EnablePWMOutputs() </B>
*
* @brief Function to enable the PWM outputs by disabling the override feature
*        
* @param none.
* @return none.
* 
* @example
* <CODE> EnablePWMOutputs(); </CODE>
*
*/
void EnablePWMOutputs(void)
{    
    /*  0 = PWM Generator provides data for the PWM3H pin */
    PG3IOCONbits.OVRENH = 0; 
    /*  0 = PWM Generator provides data for the PWM3L pin */
    PG3IOCONbits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM2H pin */
    PG2IOCONbits.OVRENH = 0;
    /*  0 = PWM Generator provides data for the PWM2L pin */
    PG2IOCONbits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM1H pin */
    PG1IOCONbits.OVRENH = 0;  
    /*  0 = PWM Generator provides data for the PWM1L pin */
    PG1IOCONbits.OVRENL = 0;     
}
/**
* <B> Function: ClearPWMPCIFault() </B>
*
* @brief Function to clear the PCI Fault by Software Termination
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ClearPWMPCIFault(); </CODE>
*
*/
void ClearPWMPCIFault(void)
{
    /*write of ?1? to SWTERM bit will produce a PCI Fault termination event*/
    PG1FPCIbits.SWTERM = 1;
    PG2FPCIbits.SWTERM = 1;
    PG3FPCIbits.SWTERM = 1;
    
}
/**
* <B> Function: PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *) </B>
*
* @brief Function to write the calculated duty to PDC register after limiting to 
* maximum and minimum value
*        
* @param Pointer to the data structure containing PWM duty cycles.
* @return none.
* 
* @example
* <CODE> PWMDutyCycleSet(&pwmDutycycle); </CODE>
*
*/
void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle,(DEADTIME>>1),(LOOPTIME_TCY - (DEADTIME>>1)));  
    PWM_PDC3 = pPwmDutycycle->dutycycle3;
    PWM_PDC2 = pPwmDutycycle->dutycycle2;
    PWM_PDC1 = pPwmDutycycle->dutycycle1;
}

/**
* <B> Function: PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *, 
*                                                     MC_DUTYCYCLEOUT_T *) </B>
*
* @brief Function to write the calculated duty to PDC and PHASE register after 
* limiting to maximum and minimum value
*        
* @param Pointer to the data structure containing PWM duty cycles for PDC register.
* @param Pointer to the data structure containing PWM duty cycles for PHASE register.
* @return none.
* 
* @example
* <CODE> PWMDutyCycleSetDualEdge(&pwmDutycycle1,&pwmDutycycle2); </CODE>
*
*/
void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *pPwmDutycycle1,MC_DUTYCYCLEOUT_T *pPwmDutycycle2)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle1,(DEADTIME>>1),(LOOPTIME_TCY - (DEADTIME>>1)));
    
    PWM_PHASE3 = pPwmDutycycle1->dutycycle3 + (DEADTIME>>1);
    PWM_PHASE2 = pPwmDutycycle1->dutycycle2 + (DEADTIME>>1);
    PWM_PHASE1 = pPwmDutycycle1->dutycycle1 + (DEADTIME>>1);
    
    pwmDutyCycleLimitCheck(pPwmDutycycle2,(DEADTIME>>1),(LOOPTIME_TCY - (DEADTIME>>1)));
    
    PWM_PDC3 = pPwmDutycycle2->dutycycle3 - (DEADTIME>>1);
    PWM_PDC2 = pPwmDutycycle2->dutycycle2 - (DEADTIME>>1);
    PWM_PDC1 = pPwmDutycycle2->dutycycle1 - (DEADTIME>>1);
}

/**
* <B> Function: pwmDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *,uint32_t, uint32_t) </B>
*
* @brief Function to write the calculated duty to PDC register after limiting to 
* maximum and minimum value
*        
* @param Pointer to the data structure containing PWM duty cycles.
* @param Minimum duty cycle.
* @param Maximum duty cycle.
* @return none.
* 
* @example
* <CODE> pwmDutyCycleLimitCheck(&pwmDutycycle, min, max); </CODE>
*
*/
void pwmDutyCycleLimitCheck (MC_DUTYCYCLEOUT_T *pPwmDutycycle,uint32_t min,uint32_t max)
{
    if (pPwmDutycycle->dutycycle1 < min)
    {
        pPwmDutycycle->dutycycle1 = min;
    }
    else if (pPwmDutycycle->dutycycle1 > max)
    {
        pPwmDutycycle->dutycycle1 = max;
    }
    
    if (pPwmDutycycle->dutycycle2 < min)
    {
        pPwmDutycycle->dutycycle2 = min;
    }
    else if (pPwmDutycycle->dutycycle2 > max)
    {
        pPwmDutycycle->dutycycle2 = max;
    }
    
    if (pPwmDutycycle->dutycycle3 < min)
    {
        pPwmDutycycle->dutycycle3 = min;
    }
    else if (pPwmDutycycle->dutycycle3 > max)
    {
        pPwmDutycycle->dutycycle3 = max;
    }
}

// </editor-fold>