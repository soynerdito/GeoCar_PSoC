/*******************************************************************************
* File Name: Vin_Y.h  
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

#if !defined(CY_PINS_Vin_Y_H) /* Pins Vin_Y_H */
#define CY_PINS_Vin_Y_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Vin_Y_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Vin_Y_Write(uint8 value) ;
void    Vin_Y_SetDriveMode(uint8 mode) ;
uint8   Vin_Y_ReadDataReg(void) ;
uint8   Vin_Y_Read(void) ;
uint8   Vin_Y_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Vin_Y_DRIVE_MODE_BITS        (3)
#define Vin_Y_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Vin_Y_DRIVE_MODE_BITS))
#define Vin_Y_DRIVE_MODE_SHIFT       (0x00u)
#define Vin_Y_DRIVE_MODE_MASK        (0x07u << Vin_Y_DRIVE_MODE_SHIFT)

#define Vin_Y_DM_ALG_HIZ         (0x00u << Vin_Y_DRIVE_MODE_SHIFT)
#define Vin_Y_DM_DIG_HIZ         (0x01u << Vin_Y_DRIVE_MODE_SHIFT)
#define Vin_Y_DM_RES_UP          (0x02u << Vin_Y_DRIVE_MODE_SHIFT)
#define Vin_Y_DM_RES_DWN         (0x03u << Vin_Y_DRIVE_MODE_SHIFT)
#define Vin_Y_DM_OD_LO           (0x04u << Vin_Y_DRIVE_MODE_SHIFT)
#define Vin_Y_DM_OD_HI           (0x05u << Vin_Y_DRIVE_MODE_SHIFT)
#define Vin_Y_DM_STRONG          (0x06u << Vin_Y_DRIVE_MODE_SHIFT)
#define Vin_Y_DM_RES_UPDWN       (0x07u << Vin_Y_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define Vin_Y_MASK               Vin_Y__MASK
#define Vin_Y_SHIFT              Vin_Y__SHIFT
#define Vin_Y_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Vin_Y_PS                     (* (reg32 *) Vin_Y__PS)
/* Port Configuration */
#define Vin_Y_PC                     (* (reg32 *) Vin_Y__PC)
/* Data Register */
#define Vin_Y_DR                     (* (reg32 *) Vin_Y__DR)
/* Input Buffer Disable Override */
#define Vin_Y_INP_DIS                (* (reg32 *) Vin_Y__PC2)


#if defined(Vin_Y__INTSTAT)  /* Interrupt Registers */

    #define Vin_Y_INTSTAT                (* (reg32 *) Vin_Y__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins Vin_Y_H */


/* [] END OF FILE */
