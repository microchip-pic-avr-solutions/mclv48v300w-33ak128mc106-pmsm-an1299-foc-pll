// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file port_config.h
 *
 * @brief This header file lists the functions and definitions for initializing 
 * GPIO pins as analog/digital,input or output etc. Also to PPS functionality to 
 * Re-mappable input or output pins.
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PORTS
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

#ifndef _PORTCONFIG_H
#define _PORTCONFIG_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif
       
// <editor-fold defaultstate="expanded" desc="DEFINITIONS ">

/* Digital I/O definitions */
        
/*Push buttons*/
/* SW1 : DIM:034 -  Pin #49 : RP58/IOMF7/RD9 */
#define SW1                   PORTDbits.RD9
/* SW2 : DIM:036 - PIN #50 : RP59/RD10  */
#define SW2                   PORTDbits.RD10
        
/* Used as START/STOP button of Motor */
#define BUTTON_START_STOP     SW1
/* Used as Speed HALF/DOUBLE button of Motor */
#define BUTTON_SPEED_HALF_DOUBLE        SW2	

/* Debug LEDs */
/* LED2(LD3) : DIM:032 - Pin #34 : RP42/IOMD10/SDO2/IOMF10/PCI19/RC9 */
#define LED2                    LATCbits.LATC9
/* LED1(LD2) : DIM:030 - Pin #55 : RP54/ASCL1/RD5 */
#define LED1                    LATDbits.LATD5

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void SetupGPIOPorts(void);
void OpampConfig (void);
void CN_Configure (void);
void MapGPIOHWFunction (void);

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif      // end of __PORT_CONFIG_H
