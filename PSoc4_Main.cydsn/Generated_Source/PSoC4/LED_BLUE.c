/*******************************************************************************
* File Name: LED_BLUE.c  
* Version 1.90
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "LED_BLUE.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        LED_BLUE_PC =   (LED_BLUE_PC & \
                                (uint32)(~(uint32)(LED_BLUE_DRIVE_MODE_IND_MASK << (LED_BLUE_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (LED_BLUE_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: LED_BLUE_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void LED_BLUE_Write(uint8 value) 
{
    uint8 drVal = (uint8)(LED_BLUE_DR & (uint8)(~LED_BLUE_MASK));
    drVal = (drVal | ((uint8)(value << LED_BLUE_SHIFT) & LED_BLUE_MASK));
    LED_BLUE_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: LED_BLUE_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to this drive mode.
*
* Return: 
*  None
*
*******************************************************************************/
void LED_BLUE_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(LED_BLUE__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: LED_BLUE_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro LED_BLUE_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 LED_BLUE_Read(void) 
{
    return (uint8)((LED_BLUE_PS & LED_BLUE_MASK) >> LED_BLUE_SHIFT);
}


/*******************************************************************************
* Function Name: LED_BLUE_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 LED_BLUE_ReadDataReg(void) 
{
    return (uint8)((LED_BLUE_DR & LED_BLUE_MASK) >> LED_BLUE_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(LED_BLUE_INTSTAT) 

    /*******************************************************************************
    * Function Name: LED_BLUE_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 LED_BLUE_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(LED_BLUE_INTSTAT & LED_BLUE_MASK);
		LED_BLUE_INTSTAT = maskedStatus;
        return maskedStatus >> LED_BLUE_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
