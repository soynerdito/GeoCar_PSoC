/*******************************************************************************
* File Name: BLUE_IntClock.h
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

#if !defined(CY_CLOCK_BLUE_IntClock_H)
#define CY_CLOCK_BLUE_IntClock_H

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

void BLUE_IntClock_Start(void) ;
void BLUE_IntClock_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void BLUE_IntClock_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void BLUE_IntClock_StandbyPower(uint8 state) ;
void BLUE_IntClock_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 BLUE_IntClock_GetDividerRegister(void) ;
void BLUE_IntClock_SetModeRegister(uint8 modeBitMask) ;
void BLUE_IntClock_ClearModeRegister(uint8 modeBitMask) ;
uint8 BLUE_IntClock_GetModeRegister(void) ;
void BLUE_IntClock_SetSourceRegister(uint8 clkSource) ;
uint8 BLUE_IntClock_GetSourceRegister(void) ;
#if defined(BLUE_IntClock__CFG3)
void BLUE_IntClock_SetPhaseRegister(uint8 clkPhase) ;
uint8 BLUE_IntClock_GetPhaseRegister(void) ;
#endif /* defined(BLUE_IntClock__CFG3) */

#define BLUE_IntClock_Enable()                       BLUE_IntClock_Start()
#define BLUE_IntClock_Disable()                      BLUE_IntClock_Stop()
#define BLUE_IntClock_SetDivider(clkDivider)         BLUE_IntClock_SetDividerRegister(clkDivider, 1)
#define BLUE_IntClock_SetDividerValue(clkDivider)    BLUE_IntClock_SetDividerRegister((clkDivider) - 1, 1)
#define BLUE_IntClock_SetMode(clkMode)               BLUE_IntClock_SetModeRegister(clkMode)
#define BLUE_IntClock_SetSource(clkSource)           BLUE_IntClock_SetSourceRegister(clkSource)
#if defined(BLUE_IntClock__CFG3)
#define BLUE_IntClock_SetPhase(clkPhase)             BLUE_IntClock_SetPhaseRegister(clkPhase)
#define BLUE_IntClock_SetPhaseValue(clkPhase)        BLUE_IntClock_SetPhaseRegister((clkPhase) + 1)
#endif /* defined(BLUE_IntClock__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define BLUE_IntClock_CLKEN              (* (reg8 *) BLUE_IntClock__PM_ACT_CFG)
#define BLUE_IntClock_CLKEN_PTR          ((reg8 *) BLUE_IntClock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define BLUE_IntClock_CLKSTBY            (* (reg8 *) BLUE_IntClock__PM_STBY_CFG)
#define BLUE_IntClock_CLKSTBY_PTR        ((reg8 *) BLUE_IntClock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define BLUE_IntClock_DIV_LSB            (* (reg8 *) BLUE_IntClock__CFG0)
#define BLUE_IntClock_DIV_LSB_PTR        ((reg8 *) BLUE_IntClock__CFG0)
#define BLUE_IntClock_DIV_PTR            ((reg16 *) BLUE_IntClock__CFG0)

/* Clock MSB divider configuration register. */
#define BLUE_IntClock_DIV_MSB            (* (reg8 *) BLUE_IntClock__CFG1)
#define BLUE_IntClock_DIV_MSB_PTR        ((reg8 *) BLUE_IntClock__CFG1)

/* Mode and source configuration register */
#define BLUE_IntClock_MOD_SRC            (* (reg8 *) BLUE_IntClock__CFG2)
#define BLUE_IntClock_MOD_SRC_PTR        ((reg8 *) BLUE_IntClock__CFG2)

#if defined(BLUE_IntClock__CFG3)
/* Analog clock phase configuration register */
#define BLUE_IntClock_PHASE              (* (reg8 *) BLUE_IntClock__CFG3)
#define BLUE_IntClock_PHASE_PTR          ((reg8 *) BLUE_IntClock__CFG3)
#endif /* defined(BLUE_IntClock__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define BLUE_IntClock_CLKEN_MASK         BLUE_IntClock__PM_ACT_MSK
#define BLUE_IntClock_CLKSTBY_MASK       BLUE_IntClock__PM_STBY_MSK

/* CFG2 field masks */
#define BLUE_IntClock_SRC_SEL_MSK        BLUE_IntClock__CFG2_SRC_SEL_MASK
#define BLUE_IntClock_MODE_MASK          (~(BLUE_IntClock_SRC_SEL_MSK))

#if defined(BLUE_IntClock__CFG3)
/* CFG3 phase mask */
#define BLUE_IntClock_PHASE_MASK         BLUE_IntClock__CFG3_PHASE_DLY_MASK
#endif /* defined(BLUE_IntClock__CFG3) */

#endif /* CY_CLOCK_BLUE_IntClock_H */


/* [] END OF FILE */
