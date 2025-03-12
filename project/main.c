// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file main.c
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

#include <stdint.h>
#include <stdbool.h>

#include <xc.h>

#include "board_service.h"
#include "diagnostics.h"

#include "mc1_service.h" 
#include "mc1_init.h"


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc=" Global Variables ">

uint8_t runCmdMC1,directionCmdMC1;
int16_t heartBeatCount;

// </editor-fold>

/**
* <B> Function: int main (void)  </B>
*
* @brief main() function,entry point of the application.
*
*/
int main (void)
{
    InitOscillator();
    SetupGPIOPorts();
    
    /* Initialize Peripherals */
    HAL_InitPeripherals();
    
#ifdef ENABLE_DIAGNOSTICS
    /* Diagnostics using X2CScope Plugin */
    DiagnosticsInit();
#endif

    MCAPP_MC1ServiceInit(); 
    
    runCmdMC1  = 0;
    
    while(1)
    {
        
#ifdef ENABLE_DIAGNOSTICS
        DiagnosticsStepMain();
#endif
        
    }
    
    return 0;
}


/**
* <B> Function: _T1Interrupt()     </B>
*
* @brief Function to handle Timer1 Interrupt. 
* Timer1 is configured for 100 micro second.
* Board service routine is executed here.
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
    BoardService();

    if(IsPressed_Button1())
    {
        if(runCmdMC1 == 1)
        {
            runCmdMC1 = 0;
        }
        else
        {
            runCmdMC1 = 1;
        }
    }
    
    if(IsPressed_Button2())
    {
        if(directionCmdMC1 == 1)
        {
            directionCmdMC1 = 0;
        }
        else
        {
            directionCmdMC1 = 1;
        }
    }
    
    /*Heart Beat LED : LED1*/
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
    
    /* LED2 status indicates the run command */
    LED2 = runCmdMC1;  
    
    MCAPP_MC1InputBufferSet(runCmdMC1,directionCmdMC1);
    
    BoardServiceStepIsr();
    TIMER1_InterruptFlagClear();
}
// </editor-fold>

