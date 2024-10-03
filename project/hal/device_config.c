// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file device_config.c
 *
 * @brief This module implements the device configuration
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: DEVICE CONFIGURATION
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* � [2024] Microchip Technology Inc. and its subsidiaries
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

// <editor-fold defaultstate="expanded" desc="CONFIGURATION BITS ">

/* DSPIC33AK128MC106 Configuration Bit Settings */

// 'C' source line config statements

// FCP
#pragma config FCP_CP = OFF             // Memory protection enable bit (Memory protection is disabled)
#pragma config FCP_CRC = OFF            // CRC enable bit (CRC is disabled)
#pragma config FCP_WPUCA = OFF          // User configuration areas write protection enable bits (Protection is disabled)

// FICD
#pragma config FICD_JTAGEN = OFF        // JTAG enable bit (JTAG is disabled)

// FDEVOPT
#pragma config FDEVOPT_ALTI2C1 = OFF    // Alternate I2C1 pins selection bit (Primary I2C1 pins are used)
#pragma config FDEVOPT_ALTI2C2 = OFF    // Alternate I2C2 pins selection bit (Primary I2C2 pins are used)
#pragma config FDEVOPT_BISTDIS = OFF    // Hardware RAM test disable bit (RAM test after reset is enabled)
#pragma config FDEVOPT_SPI2PIN = OFF    // SPI2 peripheral pin selection disable bit (SPI2 pins are selected by peripheral pin selection feature)

// FWDT
#pragma config FWDT_WINDIS = OFF        // Watchdog Timer Window Enable bit (Watchdog Timer operates in Window mode)
#pragma config FWDT_SWDTMPS = PS2147483648// Sleep Mode Watchdog Timer Post Scaler select bits (1:2147483648)
#pragma config FWDT_RCLKSEL = BPRC256   // Watchdog Timer Clock select bits (WDT Run Mode uses BFRC:256)
#pragma config FWDT_RWDTPS = PS2147483648// Run Mode Watchdog Timer Post Scaler select bits (1:2147483648)
#pragma config FWDT_WDTWIN = WIN25      // Watchdog Timer Window Size Select bits (WDT Window is 25% of WDT period)
#pragma config FWDT_WDTEN = SW          // Watchdog Timer Enable bit (WDT is controlled by software, use WDTCON.ON bit)
#pragma config FWDT_WDTRSTEN = OFF      // Watchdog Timer Reset Enable bit (WDT event generates a trap)

// FCPBKUP
#pragma config FCPBKUP_CP = OFF         // Memory protection enable bit backup (Memory protection is disabled)
#pragma config FCPBKUP_CRC = OFF        // CRC enable bit backup (CRC is disabled)
#pragma config FCPBKUP_WPUCA = OFF      // User configuration areas write protection enable bits backup (Protection is disabled)

// FICDBKUP
#pragma config FICDBKUP_JTAGEN = OFF    // JTAG enable bit backup (JTAG is disabled)

// FDEVOPTBKUP
#pragma config FDEVOPTBKUP_ALTI2C1 = OFF// Alternate I2C1 pins selection bit backup (Primary I2C1 pins are used)
#pragma config FDEVOPTBKUP_ALTI2C2 = OFF// Alternate I2C2 pins selection bit backup (Primary I2C2 pins are used)
#pragma config FDEVOPTBKUP_BISTDIS = OFF// Hardware RAM test disable bit backup (RAM test after reset is enabled)
#pragma config FDEVOPTBKUP_SPI2PIN = OFF// SPI2 peripheral pin selection disable bit backup (SPI2 pins are selected by peripheral pin selection feature)

// FWDTBKUP
#pragma config FWDTBKUP_WINDIS = OFF    // Watchdog Timer Window Enable bit (Watchdog Timer operates in Window mode)
#pragma config FWDTBKUP_SWDTMPS = PS2147483648// Sleep Mode Watchdog Timer Post Scaler select bits backup (1:2147483648)
#pragma config FWDTBKUP_RCLKSEL = BPRC256// Watchdog Timer Clock select bits backup (WDT Run Mode uses BFRC:256)
#pragma config FWDTBKUP_RWDTPS = PS2147483648// Run Mode Watchdog Timer Post Scaler select bits backup (1:2147483648)
#pragma config FWDTBKUP_WDTWIN = WIN25  // Watchdog Timer Window Size Select bits backup (WDT Window is 25% of WDT period)
#pragma config FWDTBKUP_WDTEN = SW      // Watchdog Timer Enable bit backup (WDT is controlled by software, use WDTCON.ON bit)
#pragma config FWDTBKUP_WDTRSTEN = OFF  // Watchdog Timer Reset Enable bit backup (WDT event generates a trap)

// FPR0CTRL
#pragma config FPR0CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR0CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR0CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR0CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR0CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR0CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR0ST
#pragma config FPR0ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR0END
#pragma config FPR0END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FPR1CTRL
#pragma config FPR1CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR1CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR1CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR1CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR1CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR1CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR1ST
#pragma config FPR1ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR1END
#pragma config FPR1END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FPR2CTRL
#pragma config FPR2CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR2CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR2CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR2CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR2CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR2CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR2ST
#pragma config FPR2ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR2END
#pragma config FPR2END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FPR3CTRL
#pragma config FPR3CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR3CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR3CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR3CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR3CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR3CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR3ST
#pragma config FPR3ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR3END
#pragma config FPR3END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FPR4CTRL
#pragma config FPR4CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR4CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR4CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR4CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR4CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR4CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR4ST
#pragma config FPR4ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR4END
#pragma config FPR4END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FPR5CTRL
#pragma config FPR5CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR5CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR5CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR5CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR5CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR5CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR5ST
#pragma config FPR5ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR5END
#pragma config FPR5END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FPR6CTRL
#pragma config FPR6CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR6CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR6CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR6CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR6CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR6CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR6ST
#pragma config FPR6ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR6END
#pragma config FPR6END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FPR7CTRL
#pragma config FPR7CTRL_RDIS = ON       // Region protection disable bit (Protection is disabled)
#pragma config FPR7CTRL_EX = ON         // Execute permission bit (Code execution is enabled)
#pragma config FPR7CTRL_RD = ON         // Read permission bit (Read operation is enabled)
#pragma config FPR7CTRL_WR = ON         // Write permission bit (Write and erase operation is enabled)
#pragma config FPR7CTRL_CRC = ON        // CRC enable bit (NVM controller CRC calculation is enabled)
#pragma config FPR7CTRL_RTYPE = FIRMWARE// Region type selection bits (Firmware configurable region)

// FPR7ST
#pragma config FPR7ST_START = 0x0       // Region start address bits (Enter Hexadecimal value)

// FPR7END
#pragma config FPR7END_END = 0x0        // Region end address bits (Enter Hexadecimal value)

// FIRT
#pragma config FIRT_IRT = OFF           // Immutable Root of Trust enable bit (Immutable root of trust (IRT) regions are disabled)

// FSECDBG
#pragma config FSECDBG_SECDBG = OFF     // Secure debug enable bit (Secure debug is disabled)

// FPED
#pragma config FPED_ICSPPED = OFF       // ICSP Program and Erase disable bit (ICSP can program and erase)

// FEPUCB
#pragma config FEPUCB_EPUCB = 0xFFFFFFFF// UCB erase protection bits (Enter Hexadecimal value)

// FWPUCB
#pragma config FWPUCB_WPUCB = 0xFFFFFFFF// UCB write protection bits (Enter Hexadecimal value)

// FPR0CTRLBKUP
#pragma config FPR0CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR0CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR0CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR0CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR0CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR0CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR0STBKUP
#pragma config FPR0STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR0ENDBKUP
#pragma config FPR0ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FPR1CTRLBKUP
#pragma config FPR1CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR1CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR1CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR1CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR1CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR1CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR1STBKUP
#pragma config FPR1STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR1ENDBKUP
#pragma config FPR1ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FPR2CTRLBKUP
#pragma config FPR2CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR2CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR2CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR2CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR2CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR2CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR2STBKUP
#pragma config FPR2STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR2ENDBKUP
#pragma config FPR2ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FPR3CTRLBKUP
#pragma config FPR3CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR3CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR3CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR3CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR3CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR3CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR3STBKUP
#pragma config FPR3STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR3ENDBKUP
#pragma config FPR3ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FPR4CTRLBKUP
#pragma config FPR4CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR4CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR4CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR4CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR4CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR4CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR4STBKUP
#pragma config FPR4STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR4ENDBKUP
#pragma config FPR4ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FPR5CTRLBKUP
#pragma config FPR5CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR5CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR5CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR5CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR5CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR5CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR5STBKUP
#pragma config FPR5STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR5ENDBKUP
#pragma config FPR5ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FPR6CTRLBKUP
#pragma config FPR6CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR6CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR6CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR6CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR6CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR6CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR6STBKUP
#pragma config FPR6STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR6ENDBKUP
#pragma config FPR6ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FPR7CTRLBKUP
#pragma config FPR7CTRLBKUP_RDIS = ON   // Region protection disable bit backup (Protection is disabled)
#pragma config FPR7CTRLBKUP_EX = ON     // Execute permission bit backup (Code execution is enabled)
#pragma config FPR7CTRLBKUP_RD = ON     // Read permission bit backup (Read operation is enabled)
#pragma config FPR7CTRLBKUP_WR = ON     // Write permission bit backup (Write and erase operation is enabled)
#pragma config FPR7CTRLBKUP_CRC = ON    // CRC enable bit backup (NVM controller CRC calculation is enabled)
#pragma config FPR7CTRLBKUP_RTYPE = FIRMWARE// Region type selection bits backup (Firmware configurable region)

// FPR7STBKUP
#pragma config FPR7STBKUP_START = 0x0   // Region start address bits backup (Enter Hexadecimal value)

// FPR7ENDBKUP
#pragma config FPR7ENDBKUP_END = 0x0    // Region end address bits backup (Enter Hexadecimal value)

// FIRTBKUP
#pragma config FIRTBKUP_IRT = OFF       // Immutable Root of Trust enable bit backup (Immutable root of trust (IRT) regions are disabled)

// FSECDBGBKUP
#pragma config FSECDBGBKUP_SECDBG = OFF // Secure debug enable bit backup (Secure debug is disabled)

// FPEDBKUP
#pragma config FPEDBKUP_ICSPPED = OFF   // ICSP Program and Erase disable bit backup (ICSP can program and erase)

// FEPUCBBKUP
#pragma config FEPUCBBKUP_EPUCB = 0xFFFFFFFF// UCB erase protection bits backup (Enter Hexadecimal value)

// FWPUCBBKUP
#pragma config FWPUCBBKUP_WPUCB = 0xFFFFFFFF// UCB write protection bits backup (Enter Hexadecimal value)


// </editor-fold>