/*******************************************************************************
* File Name: PSOC4_IntClock.h
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

#if !defined(CY_CLOCK_PSOC4_IntClock_H)
#define CY_CLOCK_PSOC4_IntClock_H

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

void PSOC4_IntClock_Start(void) ;
void PSOC4_IntClock_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void PSOC4_IntClock_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void PSOC4_IntClock_StandbyPower(uint8 state) ;
void PSOC4_IntClock_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 PSOC4_IntClock_GetDividerRegister(void) ;
void PSOC4_IntClock_SetModeRegister(uint8 modeBitMask) ;
void PSOC4_IntClock_ClearModeRegister(uint8 modeBitMask) ;
uint8 PSOC4_IntClock_GetModeRegister(void) ;
void PSOC4_IntClock_SetSourceRegister(uint8 clkSource) ;
uint8 PSOC4_IntClock_GetSourceRegister(void) ;
#if defined(PSOC4_IntClock__CFG3)
void PSOC4_IntClock_SetPhaseRegister(uint8 clkPhase) ;
uint8 PSOC4_IntClock_GetPhaseRegister(void) ;
#endif /* defined(PSOC4_IntClock__CFG3) */

#define PSOC4_IntClock_Enable()                       PSOC4_IntClock_Start()
#define PSOC4_IntClock_Disable()                      PSOC4_IntClock_Stop()
#define PSOC4_IntClock_SetDivider(clkDivider)         PSOC4_IntClock_SetDividerRegister(clkDivider, 1)
#define PSOC4_IntClock_SetDividerValue(clkDivider)    PSOC4_IntClock_SetDividerRegister((clkDivider) - 1, 1)
#define PSOC4_IntClock_SetMode(clkMode)               PSOC4_IntClock_SetModeRegister(clkMode)
#define PSOC4_IntClock_SetSource(clkSource)           PSOC4_IntClock_SetSourceRegister(clkSource)
#if defined(PSOC4_IntClock__CFG3)
#define PSOC4_IntClock_SetPhase(clkPhase)             PSOC4_IntClock_SetPhaseRegister(clkPhase)
#define PSOC4_IntClock_SetPhaseValue(clkPhase)        PSOC4_IntClock_SetPhaseRegister((clkPhase) + 1)
#endif /* defined(PSOC4_IntClock__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define PSOC4_IntClock_CLKEN              (* (reg8 *) PSOC4_IntClock__PM_ACT_CFG)
#define PSOC4_IntClock_CLKEN_PTR          ((reg8 *) PSOC4_IntClock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define PSOC4_IntClock_CLKSTBY            (* (reg8 *) PSOC4_IntClock__PM_STBY_CFG)
#define PSOC4_IntClock_CLKSTBY_PTR        ((reg8 *) PSOC4_IntClock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define PSOC4_IntClock_DIV_LSB            (* (reg8 *) PSOC4_IntClock__CFG0)
#define PSOC4_IntClock_DIV_LSB_PTR        ((reg8 *) PSOC4_IntClock__CFG0)
#define PSOC4_IntClock_DIV_PTR            ((reg16 *) PSOC4_IntClock__CFG0)

/* Clock MSB divider configuration register. */
#define PSOC4_IntClock_DIV_MSB            (* (reg8 *) PSOC4_IntClock__CFG1)
#define PSOC4_IntClock_DIV_MSB_PTR        ((reg8 *) PSOC4_IntClock__CFG1)

/* Mode and source configuration register */
#define PSOC4_IntClock_MOD_SRC            (* (reg8 *) PSOC4_IntClock__CFG2)
#define PSOC4_IntClock_MOD_SRC_PTR        ((reg8 *) PSOC4_IntClock__CFG2)

#if defined(PSOC4_IntClock__CFG3)
/* Analog clock phase configuration register */
#define PSOC4_IntClock_PHASE              (* (reg8 *) PSOC4_IntClock__CFG3)
#define PSOC4_IntClock_PHASE_PTR          ((reg8 *) PSOC4_IntClock__CFG3)
#endif /* defined(PSOC4_IntClock__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define PSOC4_IntClock_CLKEN_MASK         PSOC4_IntClock__PM_ACT_MSK
#define PSOC4_IntClock_CLKSTBY_MASK       PSOC4_IntClock__PM_STBY_MSK

/* CFG2 field masks */
#define PSOC4_IntClock_SRC_SEL_MSK        PSOC4_IntClock__CFG2_SRC_SEL_MASK
#define PSOC4_IntClock_MODE_MASK          (~(PSOC4_IntClock_SRC_SEL_MSK))

#if defined(PSOC4_IntClock__CFG3)
/* CFG3 phase mask */
#define PSOC4_IntClock_PHASE_MASK         PSOC4_IntClock__CFG3_PHASE_DLY_MASK
#endif /* defined(PSOC4_IntClock__CFG3) */

#endif /* CY_CLOCK_PSOC4_IntClock_H */


/* [] END OF FILE */
