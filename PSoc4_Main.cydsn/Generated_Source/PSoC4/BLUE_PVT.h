/*******************************************************************************
* File Name: .h
* Version 1.10
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component in I2C mode.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_BLUE_H)
#define CY_SCB_PVT_BLUE_H

#include "BLUE.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define BLUE_SetI2CExtClkInterruptMode(interruptMask) BLUE_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define BLUE_ClearI2CExtClkInterruptSource(interruptMask) BLUE_CLEAR_INTR_I2C_EC(interruptMask)
#define BLUE_GetI2CExtClkInterruptSource()                (BLUE_INTR_I2C_EC_REG)
#define BLUE_GetI2CExtClkInterruptMode()                  (BLUE_INTR_I2C_EC_MASK_REG)
#define BLUE_GetI2CExtClkInterruptSourceMasked()          (BLUE_INTR_I2C_EC_MASKED_REG)

/* APIs to service INTR_SPI_EC register */
#define BLUE_SetSpiExtClkInterruptMode(interruptMask) BLUE_WRITE_INTR_SPI_EC_MASK(interruptMask)
#define BLUE_ClearSpiExtClkInterruptSource(interruptMask) BLUE_CLEAR_INTR_SPI_EC(interruptMask)
#define BLUE_GetExtSpiClkInterruptSource()                 (BLUE_INTR_SPI_EC_REG)
#define BLUE_GetExtSpiClkInterruptMode()                   (BLUE_INTR_SPI_EC_MASK_REG)
#define BLUE_GetExtSpiClkInterruptSourceMasked()           (BLUE_INTR_SPI_EC_MASKED_REG)

#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void BLUE_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx);
#endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */

void BLUE_DisableTxPinsInputBuffer(void);
void BLUE_EnableTxPinsInputBuffer(void);


/**********************************
*     Vars with External Linkage
**********************************/

extern cyisraddress BLUE_customIntrHandler;
extern BLUE_BACKUP_STRUCT BLUE_backup;

#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    extern uint8 BLUE_scbMode;
    extern uint8 BLUE_scbEnableWake;
    extern uint8 BLUE_scbEnableIntr;

    /* I2C config vars */
    extern uint8 BLUE_mode;
    extern uint8 BLUE_acceptAddr;

    /* SPI/UART config vars */
    extern volatile uint8 * BLUE_rxBuffer;
    extern uint8   BLUE_rxDataBits;
    extern uint32  BLUE_rxBufferSize;

    extern volatile uint8 * BLUE_txBuffer;
    extern uint8   BLUE_txDataBits;
    extern uint32  BLUE_txBufferSize;

    /* EZI2C config vars */
    extern uint8 BLUE_numberOfAddr;
    extern uint8 BLUE_subAddrSize;
#endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */

#endif /* (CY_SCB_PVT_BLUE_H) */


/* [] END OF FILE */
