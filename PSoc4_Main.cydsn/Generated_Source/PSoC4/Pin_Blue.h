/*******************************************************************************
* File Name: Pin_Blue.h  
* Version 1.90
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Pin_Blue_H) /* Pins Pin_Blue_H */
#define CY_PINS_Pin_Blue_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Pin_Blue_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Pin_Blue_Write(uint8 value) ;
void    Pin_Blue_SetDriveMode(uint8 mode) ;
uint8   Pin_Blue_ReadDataReg(void) ;
uint8   Pin_Blue_Read(void) ;
uint8   Pin_Blue_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Pin_Blue_DRIVE_MODE_BITS        (3)
#define Pin_Blue_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Pin_Blue_DRIVE_MODE_BITS))
#define Pin_Blue_DRIVE_MODE_SHIFT       (0x00u)
#define Pin_Blue_DRIVE_MODE_MASK        (0x07u << Pin_Blue_DRIVE_MODE_SHIFT)

#define Pin_Blue_DM_ALG_HIZ         (0x00u << Pin_Blue_DRIVE_MODE_SHIFT)
#define Pin_Blue_DM_DIG_HIZ         (0x01u << Pin_Blue_DRIVE_MODE_SHIFT)
#define Pin_Blue_DM_RES_UP          (0x02u << Pin_Blue_DRIVE_MODE_SHIFT)
#define Pin_Blue_DM_RES_DWN         (0x03u << Pin_Blue_DRIVE_MODE_SHIFT)
#define Pin_Blue_DM_OD_LO           (0x04u << Pin_Blue_DRIVE_MODE_SHIFT)
#define Pin_Blue_DM_OD_HI           (0x05u << Pin_Blue_DRIVE_MODE_SHIFT)
#define Pin_Blue_DM_STRONG          (0x06u << Pin_Blue_DRIVE_MODE_SHIFT)
#define Pin_Blue_DM_RES_UPDWN       (0x07u << Pin_Blue_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define Pin_Blue_MASK               Pin_Blue__MASK
#define Pin_Blue_SHIFT              Pin_Blue__SHIFT
#define Pin_Blue_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Pin_Blue_PS                     (* (reg32 *) Pin_Blue__PS)
/* Port Configuration */
#define Pin_Blue_PC                     (* (reg32 *) Pin_Blue__PC)
/* Data Register */
#define Pin_Blue_DR                     (* (reg32 *) Pin_Blue__DR)
/* Input Buffer Disable Override */
#define Pin_Blue_INP_DIS                (* (reg32 *) Pin_Blue__PC2)


#if defined(Pin_Blue__INTSTAT)  /* Interrupt Registers */

    #define Pin_Blue_INTSTAT                (* (reg32 *) Pin_Blue__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins Pin_Blue_H */


/* [] END OF FILE */
