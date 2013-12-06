/*******************************************************************************
* File Name: GPS_SCBCLK.h
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

#if !defined(CY_CLOCK_GPS_SCBCLK_H)
#define CY_CLOCK_GPS_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/

void GPS_SCBCLK_Start(void);
void GPS_SCBCLK_Stop(void);

void GPS_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 GPS_SCBCLK_GetDividerRegister(void);
uint8  GPS_SCBCLK_GetFractionalDividerRegister(void);

#define GPS_SCBCLK_Enable()                         GPS_SCBCLK_Start()
#define GPS_SCBCLK_Disable()                        GPS_SCBCLK_Stop()
#define GPS_SCBCLK_SetDividerRegister(clkDivider, reset)  \
                        GPS_SCBCLK_SetFractionalDividerRegister((clkDivider), 0)
#define GPS_SCBCLK_SetDivider(clkDivider)           GPS_SCBCLK_SetDividerRegister((clkDivider), 1)
#define GPS_SCBCLK_SetDividerValue(clkDivider)      GPS_SCBCLK_SetDividerRegister((clkDivider) - 1, 1)


/***************************************
*             Registers
***************************************/

#define GPS_SCBCLK_DIV_REG    (*(reg32 *)GPS_SCBCLK__REGISTER)
#define GPS_SCBCLK_ENABLE_REG GPS_SCBCLK_DIV_REG

#endif /* !defined(CY_CLOCK_GPS_SCBCLK_H) */

/* [] END OF FILE */
