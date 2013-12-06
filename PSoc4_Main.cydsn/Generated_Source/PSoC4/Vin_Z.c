/*******************************************************************************
* File Name: Vin_Z.c  
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
#include "Vin_Z.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Vin_Z_PC =   (Vin_Z_PC & \
                                (uint32)(~(uint32)(Vin_Z_DRIVE_MODE_IND_MASK << (Vin_Z_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Vin_Z_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Vin_Z_Write
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
void Vin_Z_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Vin_Z_DR & (uint8)(~Vin_Z_MASK));
    drVal = (drVal | ((uint8)(value << Vin_Z_SHIFT) & Vin_Z_MASK));
    Vin_Z_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Vin_Z_SetDriveMode
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
void Vin_Z_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Vin_Z__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Vin_Z_Read
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
*  Macro Vin_Z_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Vin_Z_Read(void) 
{
    return (uint8)((Vin_Z_PS & Vin_Z_MASK) >> Vin_Z_SHIFT);
}


/*******************************************************************************
* Function Name: Vin_Z_ReadDataReg
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
uint8 Vin_Z_ReadDataReg(void) 
{
    return (uint8)((Vin_Z_DR & Vin_Z_MASK) >> Vin_Z_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Vin_Z_INTSTAT) 

    /*******************************************************************************
    * Function Name: Vin_Z_ClearInterrupt
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
    uint8 Vin_Z_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Vin_Z_INTSTAT & Vin_Z_MASK);
		Vin_Z_INTSTAT = maskedStatus;
        return maskedStatus >> Vin_Z_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
