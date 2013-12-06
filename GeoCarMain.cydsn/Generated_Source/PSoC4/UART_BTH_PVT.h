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

#if !defined(CY_SCB_PVT_UART_BTH_H)
#define CY_SCB_PVT_UART_BTH_H

#include "UART_BTH.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define UART_BTH_SetI2CExtClkInterruptMode(interruptMask) UART_BTH_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define UART_BTH_ClearI2CExtClkInterruptSource(interruptMask) UART_BTH_CLEAR_INTR_I2C_EC(interruptMask)
#define UART_BTH_GetI2CExtClkInterruptSource()                (UART_BTH_INTR_I2C_EC_REG)
#define UART_BTH_GetI2CExtClkInterruptMode()                  (UART_BTH_INTR_I2C_EC_MASK_REG)
#define UART_BTH_GetI2CExtClkInterruptSourceMasked()          (UART_BTH_INTR_I2C_EC_MASKED_REG)

/* APIs to service INTR_SPI_EC register */
#define UART_BTH_SetSpiExtClkInterruptMode(interruptMask) UART_BTH_WRITE_INTR_SPI_EC_MASK(interruptMask)
#define UART_BTH_ClearSpiExtClkInterruptSource(interruptMask) UART_BTH_CLEAR_INTR_SPI_EC(interruptMask)
#define UART_BTH_GetExtSpiClkInterruptSource()                 (UART_BTH_INTR_SPI_EC_REG)
#define UART_BTH_GetExtSpiClkInterruptMode()                   (UART_BTH_INTR_SPI_EC_MASK_REG)
#define UART_BTH_GetExtSpiClkInterruptSourceMasked()           (UART_BTH_INTR_SPI_EC_MASKED_REG)

#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void UART_BTH_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx);
#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */

void UART_BTH_DisableTxPinsInputBuffer(void);
void UART_BTH_EnableTxPinsInputBuffer(void);


/**********************************
*     Vars with External Linkage
**********************************/

extern cyisraddress UART_BTH_customIntrHandler;
extern UART_BTH_BACKUP_STRUCT UART_BTH_backup;

#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    extern uint8 UART_BTH_scbMode;
    extern uint8 UART_BTH_scbEnableWake;
    extern uint8 UART_BTH_scbEnableIntr;

    /* I2C config vars */
    extern uint8 UART_BTH_mode;
    extern uint8 UART_BTH_acceptAddr;

    /* SPI/UART config vars */
    extern volatile uint8 * UART_BTH_rxBuffer;
    extern uint8   UART_BTH_rxDataBits;
    extern uint32  UART_BTH_rxBufferSize;

    extern volatile uint8 * UART_BTH_txBuffer;
    extern uint8   UART_BTH_txDataBits;
    extern uint32  UART_BTH_txBufferSize;

    /* EZI2C config vars */
    extern uint8 UART_BTH_numberOfAddr;
    extern uint8 UART_BTH_subAddrSize;
#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */

#endif /* (CY_SCB_PVT_UART_BTH_H) */


/* [] END OF FILE */
