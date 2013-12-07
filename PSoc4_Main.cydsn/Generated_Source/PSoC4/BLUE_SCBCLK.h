/*******************************************************************************
* File Name: BLUE_SCBCLK.h
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

#if !defined(CY_CLOCK_BLUE_SCBCLK_H)
#define CY_CLOCK_BLUE_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/

void BLUE_SCBCLK_Start(void);
void BLUE_SCBCLK_Stop(void);

void BLUE_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 BLUE_SCBCLK_GetDividerRegister(void);
uint8  BLUE_SCBCLK_GetFractionalDividerRegister(void);

#define BLUE_SCBCLK_Enable()                         BLUE_SCBCLK_Start()
#define BLUE_SCBCLK_Disable()                        BLUE_SCBCLK_Stop()
#define BLUE_SCBCLK_SetDividerRegister(clkDivider, reset)  \
                        BLUE_SCBCLK_SetFractionalDividerRegister((clkDivider), 0)
#define BLUE_SCBCLK_SetDivider(clkDivider)           BLUE_SCBCLK_SetDividerRegister((clkDivider), 1)
#define BLUE_SCBCLK_SetDividerValue(clkDivider)      BLUE_SCBCLK_SetDividerRegister((clkDivider) - 1, 1)


/***************************************
*             Registers
***************************************/

#define BLUE_SCBCLK_DIV_REG    (*(reg32 *)BLUE_SCBCLK__REGISTER)
#define BLUE_SCBCLK_ENABLE_REG BLUE_SCBCLK_DIV_REG

#endif /* !defined(CY_CLOCK_BLUE_SCBCLK_H) */

/* [] END OF FILE */
