/*******************************************************************************
* File Name: LED_BLUE.h  
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

#if !defined(CY_PINS_LED_BLUE_H) /* Pins LED_BLUE_H */
#define CY_PINS_LED_BLUE_H

#include "cytypes.h"
#include "cyfitter.h"
#include "LED_BLUE_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    LED_BLUE_Write(uint8 value) ;
void    LED_BLUE_SetDriveMode(uint8 mode) ;
uint8   LED_BLUE_ReadDataReg(void) ;
uint8   LED_BLUE_Read(void) ;
uint8   LED_BLUE_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define LED_BLUE_DRIVE_MODE_BITS        (3)
#define LED_BLUE_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - LED_BLUE_DRIVE_MODE_BITS))
#define LED_BLUE_DRIVE_MODE_SHIFT       (0x00u)
#define LED_BLUE_DRIVE_MODE_MASK        (0x07u << LED_BLUE_DRIVE_MODE_SHIFT)

#define LED_BLUE_DM_ALG_HIZ         (0x00u << LED_BLUE_DRIVE_MODE_SHIFT)
#define LED_BLUE_DM_DIG_HIZ         (0x01u << LED_BLUE_DRIVE_MODE_SHIFT)
#define LED_BLUE_DM_RES_UP          (0x02u << LED_BLUE_DRIVE_MODE_SHIFT)
#define LED_BLUE_DM_RES_DWN         (0x03u << LED_BLUE_DRIVE_MODE_SHIFT)
#define LED_BLUE_DM_OD_LO           (0x04u << LED_BLUE_DRIVE_MODE_SHIFT)
#define LED_BLUE_DM_OD_HI           (0x05u << LED_BLUE_DRIVE_MODE_SHIFT)
#define LED_BLUE_DM_STRONG          (0x06u << LED_BLUE_DRIVE_MODE_SHIFT)
#define LED_BLUE_DM_RES_UPDWN       (0x07u << LED_BLUE_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define LED_BLUE_MASK               LED_BLUE__MASK
#define LED_BLUE_SHIFT              LED_BLUE__SHIFT
#define LED_BLUE_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define LED_BLUE_PS                     (* (reg32 *) LED_BLUE__PS)
/* Port Configuration */
#define LED_BLUE_PC                     (* (reg32 *) LED_BLUE__PC)
/* Data Register */
#define LED_BLUE_DR                     (* (reg32 *) LED_BLUE__DR)
/* Input Buffer Disable Override */
#define LED_BLUE_INP_DIS                (* (reg32 *) LED_BLUE__PC2)


#if defined(LED_BLUE__INTSTAT)  /* Interrupt Registers */

    #define LED_BLUE_INTSTAT                (* (reg32 *) LED_BLUE__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins LED_BLUE_H */


/* [] END OF FILE */
