/*******************************************************************************
* File Name: PSOC5_SPI_UART_PVT.h
* Version 1.10
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component in SPI and UART modes.
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

#if !defined(CY_SCB_SPI_UART_PVT_PSOC5_H)
#define CY_SCB_SPI_UART_PVT_PSOC5_H

#include "PSOC5_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  PSOC5_rxBufferHead;
    extern volatile uint32  PSOC5_rxBufferTail;
    extern volatile uint8   PSOC5_rxBufferOverflow;
#endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

#if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  PSOC5_txBufferHead;
    extern volatile uint32  PSOC5_txBufferTail;
#endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */

#if(PSOC5_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 PSOC5_rxBufferInternal[PSOC5_RX_BUFFER_SIZE];
#endif /* (PSOC5_INTERNAL_RX_SW_BUFFER) */

#if(PSOC5_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 PSOC5_txBufferInternal[PSOC5_TX_BUFFER_SIZE];
#endif /* (PSOC5_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(PSOC5_SCB_MODE_SPI_CONST_CFG)
    void PSOC5_SpiInit(void);
#endif /* (PSOC5_SCB_MODE_SPI_CONST_CFG) */

#if(PSOC5_SPI_WAKE_ENABLE_CONST)
    void PSOC5_SpiSaveConfig(void);
    void PSOC5_SpiRestoreConfig(void);
#endif /* (PSOC5_SPI_WAKE_ENABLE_CONST) */

#if(PSOC5_SCB_MODE_UART_CONST_CFG)
    void PSOC5_UartInit(void);
#endif /* (PSOC5_SCB_MODE_UART_CONST_CFG) */

#if(PSOC5_UART_WAKE_ENABLE_CONST)
    void PSOC5_UartSaveConfig(void);
    void PSOC5_UartRestoreConfig(void);
#endif /* (PSOC5_UART_WAKE_ENABLE_CONST) */

/* Interrupt processing */
#define PSOC5_SpiUartEnableIntRx(intSourceMask)  PSOC5_SetRxInterruptMode(intSourceMask)
#define PSOC5_SpiUartEnableIntTx(intSourceMask)  PSOC5_SetTxInterruptMode(intSourceMask)
uint32 PSOC5_SpiUartDisableIntRx(void);
uint32 PSOC5_SpiUartDisableIntTx(void);

#endif /* (CY_SCB_SPI_UART_PVT_PSOC5_H) */


/* [] END OF FILE */
