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

#if !defined(CY_SCB_PVT_PSOC5_H)
#define CY_SCB_PVT_PSOC5_H

#include "PSOC5.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define PSOC5_SetI2CExtClkInterruptMode(interruptMask) PSOC5_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define PSOC5_ClearI2CExtClkInterruptSource(interruptMask) PSOC5_CLEAR_INTR_I2C_EC(interruptMask)
#define PSOC5_GetI2CExtClkInterruptSource()                (PSOC5_INTR_I2C_EC_REG)
#define PSOC5_GetI2CExtClkInterruptMode()                  (PSOC5_INTR_I2C_EC_MASK_REG)
#define PSOC5_GetI2CExtClkInterruptSourceMasked()          (PSOC5_INTR_I2C_EC_MASKED_REG)

/* APIs to service INTR_SPI_EC register */
#define PSOC5_SetSpiExtClkInterruptMode(interruptMask) PSOC5_WRITE_INTR_SPI_EC_MASK(interruptMask)
#define PSOC5_ClearSpiExtClkInterruptSource(interruptMask) PSOC5_CLEAR_INTR_SPI_EC(interruptMask)
#define PSOC5_GetExtSpiClkInterruptSource()                 (PSOC5_INTR_SPI_EC_REG)
#define PSOC5_GetExtSpiClkInterruptMode()                   (PSOC5_INTR_SPI_EC_MASK_REG)
#define PSOC5_GetExtSpiClkInterruptSourceMasked()           (PSOC5_INTR_SPI_EC_MASKED_REG)

#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void PSOC5_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx);
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

void PSOC5_DisableTxPinsInputBuffer(void);
void PSOC5_EnableTxPinsInputBuffer(void);


/**********************************
*     Vars with External Linkage
**********************************/

extern cyisraddress PSOC5_customIntrHandler;
extern PSOC5_BACKUP_STRUCT PSOC5_backup;

#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    extern uint8 PSOC5_scbMode;
    extern uint8 PSOC5_scbEnableWake;
    extern uint8 PSOC5_scbEnableIntr;

    /* I2C config vars */
    extern uint8 PSOC5_mode;
    extern uint8 PSOC5_acceptAddr;

    /* SPI/UART config vars */
    extern volatile uint8 * PSOC5_rxBuffer;
    extern uint8   PSOC5_rxDataBits;
    extern uint32  PSOC5_rxBufferSize;

    extern volatile uint8 * PSOC5_txBuffer;
    extern uint8   PSOC5_txDataBits;
    extern uint32  PSOC5_txBufferSize;

    /* EZI2C config vars */
    extern uint8 PSOC5_numberOfAddr;
    extern uint8 PSOC5_subAddrSize;
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

#endif /* (CY_SCB_PVT_PSOC5_H) */


/* [] END OF FILE */
