/*******************************************************************************
* File Name: BLUE_SPI_UART_PVT.h
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

#if !defined(CY_SCB_SPI_UART_PVT_BLUE_H)
#define CY_SCB_SPI_UART_PVT_BLUE_H

#include "BLUE_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  BLUE_rxBufferHead;
    extern volatile uint32  BLUE_rxBufferTail;
    extern volatile uint8   BLUE_rxBufferOverflow;
#endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

#if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  BLUE_txBufferHead;
    extern volatile uint32  BLUE_txBufferTail;
#endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */

#if(BLUE_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 BLUE_rxBufferInternal[BLUE_RX_BUFFER_SIZE];
#endif /* (BLUE_INTERNAL_RX_SW_BUFFER) */

#if(BLUE_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 BLUE_txBufferInternal[BLUE_TX_BUFFER_SIZE];
#endif /* (BLUE_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(BLUE_SCB_MODE_SPI_CONST_CFG)
    void BLUE_SpiInit(void);
#endif /* (BLUE_SCB_MODE_SPI_CONST_CFG) */

#if(BLUE_SPI_WAKE_ENABLE_CONST)
    void BLUE_SpiSaveConfig(void);
    void BLUE_SpiRestoreConfig(void);
#endif /* (BLUE_SPI_WAKE_ENABLE_CONST) */

#if(BLUE_SCB_MODE_UART_CONST_CFG)
    void BLUE_UartInit(void);
#endif /* (BLUE_SCB_MODE_UART_CONST_CFG) */

#if(BLUE_UART_WAKE_ENABLE_CONST)
    void BLUE_UartSaveConfig(void);
    void BLUE_UartRestoreConfig(void);
#endif /* (BLUE_UART_WAKE_ENABLE_CONST) */

/* Interrupt processing */
#define BLUE_SpiUartEnableIntRx(intSourceMask)  BLUE_SetRxInterruptMode(intSourceMask)
#define BLUE_SpiUartEnableIntTx(intSourceMask)  BLUE_SetTxInterruptMode(intSourceMask)
uint32 BLUE_SpiUartDisableIntRx(void);
uint32 BLUE_SpiUartDisableIntTx(void);

#endif /* (CY_SCB_SPI_UART_PVT_BLUE_H) */


/* [] END OF FILE */
