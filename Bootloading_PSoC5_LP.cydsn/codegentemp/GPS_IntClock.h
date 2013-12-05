/*******************************************************************************
* File Name: GPS_IntClock.h
* Version 2.0
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_GPS_IntClock_H)
#define CY_CLOCK_GPS_IntClock_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
* Conditional Compilation Parameters
***************************************/

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component cy_clock_v2_0 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*        Function Prototypes
***************************************/

void GPS_IntClock_Start(void) ;
void GPS_IntClock_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void GPS_IntClock_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void GPS_IntClock_StandbyPower(uint8 state) ;
void GPS_IntClock_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 GPS_IntClock_GetDividerRegister(void) ;
void GPS_IntClock_SetModeRegister(uint8 modeBitMask) ;
void GPS_IntClock_ClearModeRegister(uint8 modeBitMask) ;
uint8 GPS_IntClock_GetModeRegister(void) ;
void GPS_IntClock_SetSourceRegister(uint8 clkSource) ;
uint8 GPS_IntClock_GetSourceRegister(void) ;
#if defined(GPS_IntClock__CFG3)
void GPS_IntClock_SetPhaseRegister(uint8 clkPhase) ;
uint8 GPS_IntClock_GetPhaseRegister(void) ;
#endif /* defined(GPS_IntClock__CFG3) */

#define GPS_IntClock_Enable()                       GPS_IntClock_Start()
#define GPS_IntClock_Disable()                      GPS_IntClock_Stop()
#define GPS_IntClock_SetDivider(clkDivider)         GPS_IntClock_SetDividerRegister(clkDivider, 1)
#define GPS_IntClock_SetDividerValue(clkDivider)    GPS_IntClock_SetDividerRegister((clkDivider) - 1, 1)
#define GPS_IntClock_SetMode(clkMode)               GPS_IntClock_SetModeRegister(clkMode)
#define GPS_IntClock_SetSource(clkSource)           GPS_IntClock_SetSourceRegister(clkSource)
#if defined(GPS_IntClock__CFG3)
#define GPS_IntClock_SetPhase(clkPhase)             GPS_IntClock_SetPhaseRegister(clkPhase)
#define GPS_IntClock_SetPhaseValue(clkPhase)        GPS_IntClock_SetPhaseRegister((clkPhase) + 1)
#endif /* defined(GPS_IntClock__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define GPS_IntClock_CLKEN              (* (reg8 *) GPS_IntClock__PM_ACT_CFG)
#define GPS_IntClock_CLKEN_PTR          ((reg8 *) GPS_IntClock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define GPS_IntClock_CLKSTBY            (* (reg8 *) GPS_IntClock__PM_STBY_CFG)
#define GPS_IntClock_CLKSTBY_PTR        ((reg8 *) GPS_IntClock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define GPS_IntClock_DIV_LSB            (* (reg8 *) GPS_IntClock__CFG0)
#define GPS_IntClock_DIV_LSB_PTR        ((reg8 *) GPS_IntClock__CFG0)
#define GPS_IntClock_DIV_PTR            ((reg16 *) GPS_IntClock__CFG0)

/* Clock MSB divider configuration register. */
#define GPS_IntClock_DIV_MSB            (* (reg8 *) GPS_IntClock__CFG1)
#define GPS_IntClock_DIV_MSB_PTR        ((reg8 *) GPS_IntClock__CFG1)

/* Mode and source configuration register */
#define GPS_IntClock_MOD_SRC            (* (reg8 *) GPS_IntClock__CFG2)
#define GPS_IntClock_MOD_SRC_PTR        ((reg8 *) GPS_IntClock__CFG2)

#if defined(GPS_IntClock__CFG3)
/* Analog clock phase configuration register */
#define GPS_IntClock_PHASE              (* (reg8 *) GPS_IntClock__CFG3)
#define GPS_IntClock_PHASE_PTR          ((reg8 *) GPS_IntClock__CFG3)
#endif /* defined(GPS_IntClock__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define GPS_IntClock_CLKEN_MASK         GPS_IntClock__PM_ACT_MSK
#define GPS_IntClock_CLKSTBY_MASK       GPS_IntClock__PM_STBY_MSK

/* CFG2 field masks */
#define GPS_IntClock_SRC_SEL_MSK        GPS_IntClock__CFG2_SRC_SEL_MASK
#define GPS_IntClock_MODE_MASK          (~(GPS_IntClock_SRC_SEL_MSK))

#if defined(GPS_IntClock__CFG3)
/* CFG3 phase mask */
#define GPS_IntClock_PHASE_MASK         GPS_IntClock__CFG3_PHASE_DLY_MASK
#endif /* defined(GPS_IntClock__CFG3) */

#endif /* CY_CLOCK_GPS_IntClock_H */


/* [] END OF FILE */
