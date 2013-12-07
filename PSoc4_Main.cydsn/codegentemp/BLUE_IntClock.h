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
*        Function Prototypes
***************************************/

void BLUE_IntClock_Start(void);
void BLUE_IntClock_Stop(void);

void BLUE_IntClock_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 BLUE_IntClock_GetDividerRegister(void);
uint8  BLUE_IntClock_GetFractionalDividerRegister(void);

#define BLUE_IntClock_Enable()                         BLUE_IntClock_Start()
#define BLUE_IntClock_Disable()                        BLUE_IntClock_Stop()
#define BLUE_IntClock_SetDividerRegister(clkDivider, reset)  \
                        BLUE_IntClock_SetFractionalDividerRegister((clkDivider), 0)
#define BLUE_IntClock_SetDivider(clkDivider)           BLUE_IntClock_SetDividerRegister((clkDivider), 1)
#define BLUE_IntClock_SetDividerValue(clkDivider)      BLUE_IntClock_SetDividerRegister((clkDivider) - 1, 1)


/***************************************
*             Registers
***************************************/

#define BLUE_IntClock_DIV_REG    (*(reg32 *)BLUE_IntClock__REGISTER)
#define BLUE_IntClock_ENABLE_REG BLUE_IntClock_DIV_REG

#endif /* !defined(CY_CLOCK_BLUE_IntClock_H) */

/* [] END OF FILE */
