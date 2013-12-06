/*******************************************************************************
* File Name: GPS_rx.h  
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

#if !defined(CY_PINS_GPS_rx_H) /* Pins GPS_rx_H */
#define CY_PINS_GPS_rx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "GPS_rx_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    GPS_rx_Write(uint8 value) ;
void    GPS_rx_SetDriveMode(uint8 mode) ;
uint8   GPS_rx_ReadDataReg(void) ;
uint8   GPS_rx_Read(void) ;
uint8   GPS_rx_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define GPS_rx_DRIVE_MODE_BITS        (3)
#define GPS_rx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - GPS_rx_DRIVE_MODE_BITS))
#define GPS_rx_DRIVE_MODE_SHIFT       (0x00u)
#define GPS_rx_DRIVE_MODE_MASK        (0x07u << GPS_rx_DRIVE_MODE_SHIFT)

#define GPS_rx_DM_ALG_HIZ         (0x00u << GPS_rx_DRIVE_MODE_SHIFT)
#define GPS_rx_DM_DIG_HIZ         (0x01u << GPS_rx_DRIVE_MODE_SHIFT)
#define GPS_rx_DM_RES_UP          (0x02u << GPS_rx_DRIVE_MODE_SHIFT)
#define GPS_rx_DM_RES_DWN         (0x03u << GPS_rx_DRIVE_MODE_SHIFT)
#define GPS_rx_DM_OD_LO           (0x04u << GPS_rx_DRIVE_MODE_SHIFT)
#define GPS_rx_DM_OD_HI           (0x05u << GPS_rx_DRIVE_MODE_SHIFT)
#define GPS_rx_DM_STRONG          (0x06u << GPS_rx_DRIVE_MODE_SHIFT)
#define GPS_rx_DM_RES_UPDWN       (0x07u << GPS_rx_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define GPS_rx_MASK               GPS_rx__MASK
#define GPS_rx_SHIFT              GPS_rx__SHIFT
#define GPS_rx_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define GPS_rx_PS                     (* (reg32 *) GPS_rx__PS)
/* Port Configuration */
#define GPS_rx_PC                     (* (reg32 *) GPS_rx__PC)
/* Data Register */
#define GPS_rx_DR                     (* (reg32 *) GPS_rx__DR)
/* Input Buffer Disable Override */
#define GPS_rx_INP_DIS                (* (reg32 *) GPS_rx__PC2)


#if defined(GPS_rx__INTSTAT)  /* Interrupt Registers */

    #define GPS_rx_INTSTAT                (* (reg32 *) GPS_rx__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins GPS_rx_H */


/* [] END OF FILE */
