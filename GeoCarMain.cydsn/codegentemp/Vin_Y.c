/*******************************************************************************
* File Name: Vin_Y.c  
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
#include "Vin_Y.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Vin_Y_PC =   (Vin_Y_PC & \
                                (uint32)(~(uint32)(Vin_Y_DRIVE_MODE_IND_MASK << (Vin_Y_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Vin_Y_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Vin_Y_Write
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
void Vin_Y_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Vin_Y_DR & (uint8)(~Vin_Y_MASK));
    drVal = (drVal | ((uint8)(value << Vin_Y_SHIFT) & Vin_Y_MASK));
    Vin_Y_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Vin_Y_SetDriveMode
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
void Vin_Y_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Vin_Y__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Vin_Y_Read
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
*  Macro Vin_Y_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Vin_Y_Read(void) 
{
    return (uint8)((Vin_Y_PS & Vin_Y_MASK) >> Vin_Y_SHIFT);
}


/*******************************************************************************
* Function Name: Vin_Y_ReadDataReg
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
uint8 Vin_Y_ReadDataReg(void) 
{
    return (uint8)((Vin_Y_DR & Vin_Y_MASK) >> Vin_Y_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Vin_Y_INTSTAT) 

    /*******************************************************************************
    * Function Name: Vin_Y_ClearInterrupt
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
    uint8 Vin_Y_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Vin_Y_INTSTAT & Vin_Y_MASK);
		Vin_Y_INTSTAT = maskedStatus;
        return maskedStatus >> Vin_Y_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
