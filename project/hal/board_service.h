// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file board_service.h
 *
 * @brief This header file lists variable types and interface functions for 
 * board services
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

#ifndef __BOARD_SERVICE_H
#define __BOARD_SERVICE_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include <xc.h>

#include "clock.h"
#include "cmp.h"
#include "pwm.h"
#include "adc.h"
#include "port_config.h"
#include "timer1.h"
#include "measure.h"
#include "svm.h"
#include "singleshunt.h"
#include "delay.h"
#include "mc1_user_params.h"
#include "mc1_calc_params.h"

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="collapsed" desc="TYPE DEFINITIONS ">
        
/* Button Scanning State

  Description:
    This structure will host parameters required by Button scan routine.
 */
typedef enum tagBUTTON_STATE
{
    BUTTON_NOT_PRESSED = 0, /* wait for button to be pressed */
    BUTTON_PRESSED = 1,     /* button was pressed, check if short press or long press */
    BUTTON_DEBOUNCE = 2     /* button de-bounced after pressing */
} BUTTON_STATE;
    
// *****************************************************************************
/* Button data type

  Description:
    This structure will host parameters required by Button scan routine.
 */
typedef struct
{
   BUTTON_STATE state;
   uint16_t debounceCount;
   bool logicState;
   bool status;
} BUTTON_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
/* Heart beat LED - specify in no.of milli second */
#define HEART_BEAT_LED_mSec     250
/* Rate at which board service i executed**/
/* Specify the board service tick in milli second */
#define BOARD_SERVICE_TICK_mSec 1
/* Button De-bounce in milli Seconds */
#define BUTTON_DEBOUNCE_mSec    30

/* Heart beat LED count is incremented in every Timer 1 interrupt */
#define HEART_BEAT_LED_COUNT        (HEART_BEAT_LED_mSec*1000/TIMER1_PERIOD_uSec)
/* Board service tick count is incremented in every Timer 1 interrupt */
#define BOARD_SERVICE_TICK_COUNT    (BOARD_SERVICE_TICK_mSec*1000/TIMER1_PERIOD_uSec)
/* Button De-bounce count is incremented at every board service function call */
#define BUTTON_DEBOUNCE_COUNT       (BUTTON_DEBOUNCE_mSec/BOARD_SERVICE_TICK_mSec)

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void BoardServiceInit(void);
void BoardServiceStepIsr(void);
void BoardService(void);
bool IsPressed_Button1(void);
bool IsPressed_Button2(void);
void HAL_InitPeripherals(void);
void HAL_ResetPeripherals(void);

void HAL_MC1PWMDisableOutputs(void);
void HAL_MC1PWMEnableOutputs(void);
void HAL_MC1PWMDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *);
void HAL_PWM_DutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *,uint32_t);
void HAL_PWM_PhaseRegister_Set(MC_DUTYCYCLEOUT_T *,uint32_t);
void HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *);

void HAL_MC1ClearPWMPCIFault(void);
void HAL_TrapHandler(void);
// </editor-fold

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_SERVICE_H */
