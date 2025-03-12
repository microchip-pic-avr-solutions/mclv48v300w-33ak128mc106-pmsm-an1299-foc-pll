// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file port_config.c
 *
 * @brief This module initializes the GPIO pins as analog/digital,input or 
 * output etc. Also to PPS functionality to Re-mappable input or output pins.
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>

#include "port_config.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: SetupGPIOPorts() </B>
*
* @brief Function initialize all ports as input and digital pins
*        
* @param none.
* @return none.
* 
* @example
* <CODE> SetupGPIOPorts(); </CODE>
*
*/
void SetupGPIOPorts(void)
{
    /* Reset all PORTx register (all inputs) */
    
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    MapGPIOHWFunction();

}
/**
* <B> Function: MapGPIOHWFunction() </B>
*
* @brief Function maps port pins as input or output, analog or digital
*        
* @param none.
* @return none.
* 
* @example
* <CODE> MapGPIOHWFunction(); </CODE>
*
*/
void MapGPIOHWFunction(void)
{  
    /* Amplifier input and output pins */
    /* IA : OA1
     * Positive Input   : DIM:013 - PIN14: OA1IN+/AD1AN1/CMP1B/RP5/RA4
     * Negative Input   : DIM:015 - PIN13: OA1IN-/AD1ANN1/AD2AN0/RP4/RA3
     * Amplifier Output : DIM:017 - PIN12: OA1OUT/AD1AN0/CMP1A/RP3/RA2 */
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   
    ANSELAbits.ANSELA3 = 1;
    TRISAbits.TRISA3 = 1;   
    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 0;
    
    /* IB : OA2
     * Positive Input   : DIM:021 - PIN22: OA2IN+/AD2AN4/CMP2B/RP19/RB2
     * Negative Input   : DIM:023 - PIN21: TMS/OA2IN-/AD1AN4/AD2ANN1/RP18/RB1
     * Amplifier Output : DIM:025 - PIN20: OA2OUT/AD2AN1/CMP2A/RP17/INT0/RB0 */
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   
    ANSELBbits.ANSELB1 = 1;
    TRISBbits.TRISB1 = 1;     
    ANSELBbits.ANSELB0 = 1;
    TRISBbits.TRISB0 = 0;
    
    /* IBUS : OA3
     * Positive Input   : DIM:029 - PIN17: OA3IN+/AD2AN2/CMP3B/RP22/RB5
     * Negative Input   : DIM:031 - PIN16: OA3IN-/AD1AN2/RP7/RA6
     * Amplifier Output : DIM:033 - PIN15: OA3OUT/AD1AN3/CMP3A/RP6/RA5 */
    ANSELBbits.ANSELB5 = 1;
    TRISBbits.TRISB5 = 1;   
    ANSELAbits.ANSELA6 = 1;
    TRISAbits.TRISA6 = 1;   
    ANSELAbits.ANSELA5 = 1;
    TRISAbits.TRISA5 = 0;
    
#ifndef INTERNAL_OPAMP_CONFIG
    /* External Amplifier mode - Amplifier output pins are configured as
     analog input channels
     * IA   : DIM:019 - PIN #12: OA1OUT/AD1AN0/CMP1A/RP3/RA2   
     * IB   : DIM:027 - PIN #20: OA2OUT/AD2AN1/CMP2A/RP17/INT0/RB0
     * IBUS : DIM:035 - PIN #15: OA3OUT/AD1AN3/CMP3A/RP6/RA5 */
    TRISAbits.TRISA2 = 1;   
    TRISBbits.TRISB0 = 1;  
    TRISAbits.TRISA5 = 1;  
#endif
    
    /* Potentiometer  input (POT1) - used as Speed Reference 
     * DIM:028 - PIN #06: AD1AN10/RP12/RA11 */
    ANSELAbits.ANSELA11 = 1;
    TRISAbits.TRISA11 = 1;   
    
    /* DC Bus Voltage (VBUS) 
     * DIM:039 -  PIN #02: AD1AN6/RP8/IOMF1/RA7 */
    ANSELAbits.ANSELA7 = 1;
    TRISAbits.TRISA7 = 1;   
    
    /* Digital SIGNALS */   
    /* Inverter Control - PWM Outputs
     * PWM1L : DIM:003 - PIN #54  TDI/RP52/PWM1L/IOMD5/RD3
     * PWM1H : DIM:001 - PIN #53  TDO/RP51/PWM1H/IOMD4/RD2
     * PWM2L : DIM:007 - PIN #52  TCK/RP50/PWM2L/IOMD3/RD1
     * PWM2H : DIM:005 - PIN #51  RP49/PWM2H/IOMD2/RD0
     * PWM3L : DIM:004 - PIN #44  RP37/PWM3L/IOMD1/RC4
     * PWM3H : DIM:002 - PIN #43  PGD3/RP36/PWM3H/IOMD0/RC3      */
    TRISDbits.TRISD3 = 0 ;          
    TRISDbits.TRISD2 = 0 ;         
    TRISDbits.TRISD1 = 0 ;          
    TRISDbits.TRISD0 = 0 ;           
    TRISCbits.TRISC4 = 0 ;          
    TRISCbits.TRISC3 = 0 ;         
     
    /* Debug LEDs */
    /* LED1 : DIM:030 - PIN #55 : RP54/ASCL1/RD5   */
    TRISDbits.TRISD5 = 0;
    /* LED2 : DIM:032 - PIN #34 : RP42/IOMD10/SDO2/IOMF10/PCI19/RC9  */
    TRISCbits.TRISC9 = 0;

    /* Push button Switches */
    /* SW1 : DIM:034 - PIN #49 : RP58/IOMF7/RD9   */
    TRISDbits.TRISD9 = 1;            
    /* SW2 : DIM:036 - PIN #50 : RP59/RD10  */
    TRISDbits.TRISD10 = 1;            
	
    /* Configuring FLTLAT_OC_OV (DIM:040) - Pin #32 : RP28/SDI2/RB11 as PCI8, 
     Please note PWM fault PCI is configured with Comparator output, not PCI pin*/
	_PCI8R = 28;
    
	/** Diagnostic Interface
        Re-map UART Channels to the device pins connected to the following 
        pins on the Motor Control Development Board.
        UART_RX : DIM:054 - PIN #46 : RP44/IOMD8/IOMF8/RC11 (Input)
        UART_TX : DIM:052 - PIN #45 : RP43/IOMD9/IOMF9/RC10(Output)   */
    _U1RXR = 44;
    _RP43R = 9;
    
}

/**
* <B> Function: OpampConfig() </B>
*
* @brief Function to configure and enable the Op-Amp Module
*        
* @param none.
* @return none.
* 
* @example
* <CODE> OpampConfig(); </CODE>
*
*/
void OpampConfig (void)
{
    /** AMP1CON1 :AMP1 Control Register 1*/
    AMP1CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP1CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP1CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP1CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP1CON1bits.DIFFCON = 0;
    /** Bit 8 = OMONEN Enable Output Monitor bit
        1 Enables output to ADC; 
        0 Disables output to ADC */     
    AMP1CON1bits.OMONEN = 1;
    
    /** AMP2CON1 :AMP2 Control Register 1*/
    AMP2CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP2CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP2CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP2CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP2CON1bits.DIFFCON = 0;
    /** Bit 8 = OMONEN Enable Output Monitor bit
        1 Enables output to ADC; 
        0 Disables output to ADC */     
    AMP2CON1bits.OMONEN = 1;
    
    /** AMP3CON1 :AMP3 Control Register 1*/
    AMP3CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP3CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP3CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP3CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP3CON1bits.DIFFCON = 0;
    /** Bit 8 = OMONEN Enable Output Monitor bit
        1 Enables output to ADC; 
        0 Disables output to ADC */     
    AMP3CON1bits.OMONEN = 1;
    
    /* Enabling the amplifiers */
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP1CON1bits.AMPEN = 1;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP2CON1bits.AMPEN = 1;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP3CON1bits.AMPEN = 1;
}

// </editor-fold>