/*******************************************************************************
* File Name: UART_BTH_SPI_UART_PVT.h
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

#if !defined(CY_SCB_SPI_UART_PVT_UART_BTH_H)
#define CY_SCB_SPI_UART_PVT_UART_BTH_H

#include "UART_BTH_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if(UART_BTH_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  UART_BTH_rxBufferHead;
    extern volatile uint32  UART_BTH_rxBufferTail;
    extern volatile uint8   UART_BTH_rxBufferOverflow;
#endif /* (UART_BTH_INTERNAL_RX_SW_BUFFER_CONST) */

#if(UART_BTH_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  UART_BTH_txBufferHead;
    extern volatile uint32  UART_BTH_txBufferTail;
#endif /* (UART_BTH_INTERNAL_TX_SW_BUFFER_CONST) */

#if(UART_BTH_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 UART_BTH_rxBufferInternal[UART_BTH_RX_BUFFER_SIZE];
#endif /* (UART_BTH_INTERNAL_RX_SW_BUFFER) */

#if(UART_BTH_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 UART_BTH_txBufferInternal[UART_BTH_TX_BUFFER_SIZE];
#endif /* (UART_BTH_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(UART_BTH_SCB_MODE_SPI_CONST_CFG)
    void UART_BTH_SpiInit(void);
#endif /* (UART_BTH_SCB_MODE_SPI_CONST_CFG) */

#if(UART_BTH_SPI_WAKE_ENABLE_CONST)
    void UART_BTH_SpiSaveConfig(void);
    void UART_BTH_SpiRestoreConfig(void);
#endif /* (UART_BTH_SPI_WAKE_ENABLE_CONST) */

#if(UART_BTH_SCB_MODE_UART_CONST_CFG)
    void UART_BTH_UartInit(void);
#endif /* (UART_BTH_SCB_MODE_UART_CONST_CFG) */

#if(UART_BTH_UART_WAKE_ENABLE_CONST)
    void UART_BTH_UartSaveConfig(void);
    void UART_BTH_UartRestoreConfig(void);
#endif /* (UART_BTH_UART_WAKE_ENABLE_CONST) */

/* Interrupt processing */
#define UART_BTH_SpiUartEnableIntRx(intSourceMask)  UART_BTH_SetRxInterruptMode(intSourceMask)
#define UART_BTH_SpiUartEnableIntTx(intSourceMask)  UART_BTH_SetTxInterruptMode(intSourceMask)
uint32 UART_BTH_SpiUartDisableIntRx(void);
uint32 UART_BTH_SpiUartDisableIntTx(void);

#endif /* (CY_SCB_SPI_UART_PVT_UART_BTH_H) */


/* [] END OF FILE */
