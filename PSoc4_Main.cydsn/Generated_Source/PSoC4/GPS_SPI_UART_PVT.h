/*******************************************************************************
* File Name: GPS_SPI_UART_PVT.h
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

#if !defined(CY_SCB_SPI_UART_PVT_GPS_H)
#define CY_SCB_SPI_UART_PVT_GPS_H

#include "GPS_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  GPS_rxBufferHead;
    extern volatile uint32  GPS_rxBufferTail;
    extern volatile uint8   GPS_rxBufferOverflow;
#endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

#if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  GPS_txBufferHead;
    extern volatile uint32  GPS_txBufferTail;
#endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */

#if(GPS_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 GPS_rxBufferInternal[GPS_RX_BUFFER_SIZE];
#endif /* (GPS_INTERNAL_RX_SW_BUFFER) */

#if(GPS_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 GPS_txBufferInternal[GPS_TX_BUFFER_SIZE];
#endif /* (GPS_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(GPS_SCB_MODE_SPI_CONST_CFG)
    void GPS_SpiInit(void);
#endif /* (GPS_SCB_MODE_SPI_CONST_CFG) */

#if(GPS_SPI_WAKE_ENABLE_CONST)
    void GPS_SpiSaveConfig(void);
    void GPS_SpiRestoreConfig(void);
#endif /* (GPS_SPI_WAKE_ENABLE_CONST) */

#if(GPS_SCB_MODE_UART_CONST_CFG)
    void GPS_UartInit(void);
#endif /* (GPS_SCB_MODE_UART_CONST_CFG) */

#if(GPS_UART_WAKE_ENABLE_CONST)
    void GPS_UartSaveConfig(void);
    void GPS_UartRestoreConfig(void);
#endif /* (GPS_UART_WAKE_ENABLE_CONST) */

/* Interrupt processing */
#define GPS_SpiUartEnableIntRx(intSourceMask)  GPS_SetRxInterruptMode(intSourceMask)
#define GPS_SpiUartEnableIntTx(intSourceMask)  GPS_SetTxInterruptMode(intSourceMask)
uint32 GPS_SpiUartDisableIntRx(void);
uint32 GPS_SpiUartDisableIntTx(void);

#endif /* (CY_SCB_SPI_UART_PVT_GPS_H) */


/* [] END OF FILE */
