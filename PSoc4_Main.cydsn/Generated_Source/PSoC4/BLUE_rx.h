/*******************************************************************************
* File Name: BLUE_rx.h  
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

#if !defined(CY_PINS_BLUE_rx_H) /* Pins BLUE_rx_H */
#define CY_PINS_BLUE_rx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "BLUE_rx_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    BLUE_rx_Write(uint8 value) ;
void    BLUE_rx_SetDriveMode(uint8 mode) ;
uint8   BLUE_rx_ReadDataReg(void) ;
uint8   BLUE_rx_Read(void) ;
uint8   BLUE_rx_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define BLUE_rx_DRIVE_MODE_BITS        (3)
#define BLUE_rx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - BLUE_rx_DRIVE_MODE_BITS))
#define BLUE_rx_DRIVE_MODE_SHIFT       (0x00u)
#define BLUE_rx_DRIVE_MODE_MASK        (0x07u << BLUE_rx_DRIVE_MODE_SHIFT)

#define BLUE_rx_DM_ALG_HIZ         (0x00u << BLUE_rx_DRIVE_MODE_SHIFT)
#define BLUE_rx_DM_DIG_HIZ         (0x01u << BLUE_rx_DRIVE_MODE_SHIFT)
#define BLUE_rx_DM_RES_UP          (0x02u << BLUE_rx_DRIVE_MODE_SHIFT)
#define BLUE_rx_DM_RES_DWN         (0x03u << BLUE_rx_DRIVE_MODE_SHIFT)
#define BLUE_rx_DM_OD_LO           (0x04u << BLUE_rx_DRIVE_MODE_SHIFT)
#define BLUE_rx_DM_OD_HI           (0x05u << BLUE_rx_DRIVE_MODE_SHIFT)
#define BLUE_rx_DM_STRONG          (0x06u << BLUE_rx_DRIVE_MODE_SHIFT)
#define BLUE_rx_DM_RES_UPDWN       (0x07u << BLUE_rx_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define BLUE_rx_MASK               BLUE_rx__MASK
#define BLUE_rx_SHIFT              BLUE_rx__SHIFT
#define BLUE_rx_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define BLUE_rx_PS                     (* (reg32 *) BLUE_rx__PS)
/* Port Configuration */
#define BLUE_rx_PC                     (* (reg32 *) BLUE_rx__PC)
/* Data Register */
#define BLUE_rx_DR                     (* (reg32 *) BLUE_rx__DR)
/* Input Buffer Disable Override */
#define BLUE_rx_INP_DIS                (* (reg32 *) BLUE_rx__PC2)


#if defined(BLUE_rx__INTSTAT)  /* Interrupt Registers */

    #define BLUE_rx_INTSTAT                (* (reg32 *) BLUE_rx__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins BLUE_rx_H */


/* [] END OF FILE */
