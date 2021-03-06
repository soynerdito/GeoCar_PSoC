/*******************************************************************************
* File Name: PSOC5_rx.h  
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

#if !defined(CY_PINS_PSOC5_rx_H) /* Pins PSOC5_rx_H */
#define CY_PINS_PSOC5_rx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "PSOC5_rx_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    PSOC5_rx_Write(uint8 value) ;
void    PSOC5_rx_SetDriveMode(uint8 mode) ;
uint8   PSOC5_rx_ReadDataReg(void) ;
uint8   PSOC5_rx_Read(void) ;
uint8   PSOC5_rx_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define PSOC5_rx_DRIVE_MODE_BITS        (3)
#define PSOC5_rx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - PSOC5_rx_DRIVE_MODE_BITS))
#define PSOC5_rx_DRIVE_MODE_SHIFT       (0x00u)
#define PSOC5_rx_DRIVE_MODE_MASK        (0x07u << PSOC5_rx_DRIVE_MODE_SHIFT)

#define PSOC5_rx_DM_ALG_HIZ         (0x00u << PSOC5_rx_DRIVE_MODE_SHIFT)
#define PSOC5_rx_DM_DIG_HIZ         (0x01u << PSOC5_rx_DRIVE_MODE_SHIFT)
#define PSOC5_rx_DM_RES_UP          (0x02u << PSOC5_rx_DRIVE_MODE_SHIFT)
#define PSOC5_rx_DM_RES_DWN         (0x03u << PSOC5_rx_DRIVE_MODE_SHIFT)
#define PSOC5_rx_DM_OD_LO           (0x04u << PSOC5_rx_DRIVE_MODE_SHIFT)
#define PSOC5_rx_DM_OD_HI           (0x05u << PSOC5_rx_DRIVE_MODE_SHIFT)
#define PSOC5_rx_DM_STRONG          (0x06u << PSOC5_rx_DRIVE_MODE_SHIFT)
#define PSOC5_rx_DM_RES_UPDWN       (0x07u << PSOC5_rx_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define PSOC5_rx_MASK               PSOC5_rx__MASK
#define PSOC5_rx_SHIFT              PSOC5_rx__SHIFT
#define PSOC5_rx_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define PSOC5_rx_PS                     (* (reg32 *) PSOC5_rx__PS)
/* Port Configuration */
#define PSOC5_rx_PC                     (* (reg32 *) PSOC5_rx__PC)
/* Data Register */
#define PSOC5_rx_DR                     (* (reg32 *) PSOC5_rx__DR)
/* Input Buffer Disable Override */
#define PSOC5_rx_INP_DIS                (* (reg32 *) PSOC5_rx__PC2)


#if defined(PSOC5_rx__INTSTAT)  /* Interrupt Registers */

    #define PSOC5_rx_INTSTAT                (* (reg32 *) PSOC5_rx__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins PSOC5_rx_H */


/* [] END OF FILE */
