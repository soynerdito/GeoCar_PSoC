/*******************************************************************************
* File Name: PSOC5_SCBCLK.h
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

#if !defined(CY_CLOCK_PSOC5_SCBCLK_H)
#define CY_CLOCK_PSOC5_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/

void PSOC5_SCBCLK_Start(void);
void PSOC5_SCBCLK_Stop(void);

void PSOC5_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 PSOC5_SCBCLK_GetDividerRegister(void);
uint8  PSOC5_SCBCLK_GetFractionalDividerRegister(void);

#define PSOC5_SCBCLK_Enable()                         PSOC5_SCBCLK_Start()
#define PSOC5_SCBCLK_Disable()                        PSOC5_SCBCLK_Stop()
#define PSOC5_SCBCLK_SetDividerRegister(clkDivider, reset)  \
                        PSOC5_SCBCLK_SetFractionalDividerRegister((clkDivider), 0)
#define PSOC5_SCBCLK_SetDivider(clkDivider)           PSOC5_SCBCLK_SetDividerRegister((clkDivider), 1)
#define PSOC5_SCBCLK_SetDividerValue(clkDivider)      PSOC5_SCBCLK_SetDividerRegister((clkDivider) - 1, 1)


/***************************************
*             Registers
***************************************/

#define PSOC5_SCBCLK_DIV_REG    (*(reg32 *)PSOC5_SCBCLK__REGISTER)
#define PSOC5_SCBCLK_ENABLE_REG PSOC5_SCBCLK_DIV_REG

#endif /* !defined(CY_CLOCK_PSOC5_SCBCLK_H) */

/* [] END OF FILE */
