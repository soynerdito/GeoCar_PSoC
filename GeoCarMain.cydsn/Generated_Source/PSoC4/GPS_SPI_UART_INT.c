/*******************************************************************************
* File Name: GPS_SPI_UART_INT.c
* Version 1.10
*
* Description:
*  This file provides the source code to the Interrupt Servive Routine for
*  the SCB Component in SPI and UART modes.
*
* Note:
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "GPS_PVT.h"
#include "GPS_SPI_UART_PVT.h"


/*******************************************************************************
* Function Name: GPS_SPI_UART_ISR
********************************************************************************
*
* Summary:
*  Handles Interrupt Service Routine for SCB SPI or UART modes.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
CY_ISR(GPS_SPI_UART_ISR)
{
    #if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locHead;
        uint32 dataRx;
    #endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */

    if(NULL != GPS_customIntrHandler)
    {
        GPS_customIntrHandler(); /* Call customer routine if needed */
    }

    #if(GPS_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        GPS_ClearSpiExtClkInterruptSource(GPS_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if(GPS_CHECK_RX_SW_BUFFER)
    {
        /* Get data from RX FIFO */
        if(GPS_CHECK_INTR_RX_MASKED(GPS_INTR_RX_NOT_EMPTY))
        {
            while(0u != GPS_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = GPS_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (GPS_rxBufferHead + 1u);

                /* Adjust local head index */
                if(GPS_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == GPS_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    GPS_rxBufferOverflow = (uint8) GPS_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    GPS_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    GPS_rxBufferHead = locHead;
                }
            }

            GPS_ClearRxInterruptSource(GPS_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if(GPS_CHECK_TX_SW_BUFFER)
    {
        if(GPS_CHECK_INTR_TX_MASKED(GPS_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(GPS_FIFO_SIZE != GPS_GET_TX_FIFO_ENTRIES)
            {
                /* There is a data in TX software buffer */
                if(GPS_txBufferHead != GPS_txBufferTail)
                {
                    /* Mode local tail index */
                    locTail = (GPS_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(GPS_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    GPS_TX_FIFO_WR_REG = GPS_GetWordFromTxBuffer(locTail);

                    /* Mode tail index */
                    GPS_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is EMPTY: end of transmitting */
                    GPS_DISABLE_INTR_TX(GPS_INTR_TX_NOT_FULL);
                    break;
                }
            }

            GPS_ClearTxInterruptSource(GPS_INTR_TX_NOT_FULL);
        }
    }
    #endif
}


/* [] END OF FILE */
