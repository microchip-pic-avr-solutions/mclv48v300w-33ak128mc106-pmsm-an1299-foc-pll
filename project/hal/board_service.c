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

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">

BUTTON_T buttonStartStop;
BUTTON_T buttonDirectionChange;

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
    if(buttonStartStop.status)
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
    if (buttonDirectionChange.status)
    {
        buttonDirectionChange.status = false;
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
        ButtonScan(&buttonDirectionChange,BUTTON_DIRECTION_CHANGE);

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

    buttonDirectionChange.state = BUTTON_NOT_PRESSED;
    buttonDirectionChange.debounceCount = 0;
    buttonDirectionChange.state = false;
}

/**
* <B> Function: HAL_InitPeripherals() </B>
*
* @brief Function to initialize the peripherals(Op-Amp, ADC, CMP, DAC and PWM)
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_InitPeripherals(); </CODE>
*
*/
void HAL_InitPeripherals(void)
{                
#ifdef INTERNAL_OPAMP_CONFIG    
    OpampConfig();
#endif
    
    InitializeADCs();
    
    InitializeCMPs();  
    CMP3_ReferenceSet(CMP_REF_DCBUS_FAULT);
    CMP3_ModuleEnable(true);

    /*400ms POR delay for IBUS_EXT signal coming from MCP651S in Dev Board*/
    __delay_ms(400);
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    MC1_DisableADCInterrupt();  
    
    InitPWMGenerators(); 
    
    /*Clearing and enabling PWM Interrupt to handle PCI Fault*/
    ClearPWMIF();
    EnablePWMIF();
    
    /*Timer 1 initialization*/
    TIMER1_Initialize();
    TIMER1_InputClockSet();
    TIMER1_PeriodSet(TIMER1_PERIOD_COUNT);
    TIMER1_InterruptPrioritySet(5);
    TIMER1_InterruptFlagClear();
    TIMER1_InterruptEnable(); 
    TIMER1_ModuleStart();

}

/**
* <B> Function: HAL_ResetPeripherals() </B>
*
* @brief Function to reset the clear ADC interrupt and disable PWM outputs
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_ResetPeripherals(); </CODE>
*
*/
void HAL_ResetPeripherals(void)
{
    MC1_ClearADCIF();
    MC1_EnableADCInterrupt();
    HAL_MC1PWMDisableOutputs();
}

/**
* <B> Function: HAL_MC1PWMEnableOutputs() </B>
*
* @brief Function to enable the PWM outputs -  override is removed
* PWM generator controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMEnableOutputs(); </CODE>
*
*/
void HAL_MC1PWMEnableOutputs(void)
{
    /* Set PWM Duty Cycles */
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;
	
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
* <B> Function: HAL_MC1PWMDisableOutputs() </B>
*
* @brief Function to disable the PWM outputs -  override is activated
* OVRDAT<> register controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMDisableOutputs(); </CODE>
*
*/
void HAL_MC1PWMDisableOutputs(void)
{
    /* Set PWM Duty Cycles */
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0; 
    
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
* <B> Function: HAL_MC1PWMDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *) </B>
*
* @brief Function to limit the PWM duty cycles between Min and Max duty
*        
* @param Pointer to the data structure containing duty cycles
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMDutyCycleLimitCheck(pPdc); </CODE>
*
*/
void HAL_MC1PWMDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *pdc)
{
    if(pdc->dutycycle3 < MIN_DUTY)
    {
        pdc->dutycycle3 = MIN_DUTY;
    }
    if(pdc->dutycycle2 < MIN_DUTY)
    {
        pdc->dutycycle2 = MIN_DUTY;
    }
    if(pdc->dutycycle1 < MIN_DUTY)
    {
        pdc->dutycycle1 = MIN_DUTY;
    }

    if(pdc->dutycycle3 > MAX_DUTY)
    {
        pdc->dutycycle3 = MAX_DUTY;
    }
    if(pdc->dutycycle2 > MAX_DUTY)
    {
        pdc->dutycycle2 = MAX_DUTY;
    }
    if(pdc->dutycycle1 > MAX_DUTY)
    {
        pdc->dutycycle1 = MAX_DUTY;
    }
}

/**
* <B> Function: HAL_PWM_DutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *,uint32_t) </B>
*
* @brief Function to set the duty cycle values to the PDC registers
*        
* @param Pointer to the data structure containing duty cycles
* @param offset value
* @return none.
* 
* @example
* <CODE> HAL_PWM_DutyCycleRegister_Set(pMC1Data->pPWMDuty,0); </CODE>
*
*/
void HAL_PWM_DutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *pPdc,uint32_t offset)
{
    
    HAL_MC1PWMDutyCycleLimitCheck(pPdc);
    PWM_PDC3 = (uint32_t)(pPdc->dutycycle3 + offset);
    PWM_PDC2 = (uint32_t)(pPdc->dutycycle2 + offset);
    PWM_PDC1 = (uint32_t)(pPdc->dutycycle1 + offset);
}

/**
* <B> Function: HAL_PWM_PhaseRegister_Set(MC_DUTYCYCLEOUT_T *,uint32_t) </B>
*
* @brief Function to set the duty cycle values to the PHASE registers
*        
* @param Pointer to the data structure containing duty cycles
* @param offset value
* @return none.
* 
* @example
* <CODE> HAL_PWM_PhaseRegister_Set(&pMC1Data->pSingleShunt->phase,(DEADTIME>>1)); </CODE>
*
*/
void HAL_PWM_PhaseRegister_Set(MC_DUTYCYCLEOUT_T *pPhase,uint32_t offset)
{
    
    HAL_MC1PWMDutyCycleLimitCheck(pPhase);
    PWM_PHASE3 = (uint32_t)(pPhase->dutycycle3 + offset);
    PWM_PHASE2 = (uint32_t)(pPhase->dutycycle2 + offset);
    PWM_PHASE1 = (uint32_t)(pPhase->dutycycle1 + offset);
}

/**
* <B> Function: HAL_TrapHandler(void) </B>
*
* @brief Function handle the traps
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_TrapHandler(); </CODE>
*
*/
void HAL_TrapHandler(void)
{
    HAL_MC1PWMDisableOutputs();
    while (1)
    {
        Nop();
        Nop();
        Nop();
    }
}

/**
* <B> Function: HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to assign the variables with respective ADC buffers
*        
* @param Pointer to the data structure containing measured currents.
* @return none.
* 
* @example
* <CODE> HAL_MC1MotorInputsRead(pMotorInputs); </CODE>
*
*/
void HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs)
{
    pMotorInputs->measureCurrent.Ia = MC1_ADCBUF_IA;
    pMotorInputs->measureCurrent.Ib = MC1_ADCBUF_IB;
    pMotorInputs->measureCurrent.Ibus1 = ADCBUF_IBUS1;
    pMotorInputs->measureCurrent.Ibus2 = ADCBUF_IBUS2;
    pMotorInputs->measureCurrent.Ibus = ADCBUF_IBUS1;
    pMotorInputs->measurePot = MC1_ADCBUF_POT;
    pMotorInputs->measureVdc.count = MC_ADCBUF_VDC;
}

/**
* <B> Function: HAL_MC1ClearPWMPCIFault(void)  </B>
*
* @brief Function to terminate the PWM Fault PWM event through software
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1ClearPWMPCIFault(); </CODE>
*
*/
void HAL_MC1ClearPWMPCIFault(void)
{
    /* write of '1' to SWTERM bit will produce a PCI Fault termination event */
    PG1FPCIbits.SWTERM = 1;
    PG2FPCIbits.SWTERM = 1;
    PG3FPCIbits.SWTERM = 1;  
}