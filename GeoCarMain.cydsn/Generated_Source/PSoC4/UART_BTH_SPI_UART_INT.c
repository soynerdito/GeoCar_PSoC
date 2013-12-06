/*******************************************************************************
* File Name: UART_BTH_SPI_UART_INT.c
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

#include "UART_BTH_PVT.h"
#include "UART_BTH_SPI_UART_PVT.h"


/*******************************************************************************
* Function Name: UART_BTH_SPI_UART_ISR
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
CY_ISR(UART_BTH_SPI_UART_ISR)
{
    #if(UART_BTH_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locHead;
        uint32 dataRx;
    #endif /* (UART_BTH_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(UART_BTH_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (UART_BTH_INTERNAL_TX_SW_BUFFER_CONST) */

    if(NULL != UART_BTH_customIntrHandler)
    {
        UART_BTH_customIntrHandler(); /* Call customer routine if needed */
    }

    #if(UART_BTH_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        UART_BTH_ClearSpiExtClkInterruptSource(UART_BTH_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if(UART_BTH_CHECK_RX_SW_BUFFER)
    {
        /* Get data from RX FIFO */
        if(UART_BTH_CHECK_INTR_RX_MASKED(UART_BTH_INTR_RX_NOT_EMPTY))
        {
            while(0u != UART_BTH_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = UART_BTH_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (UART_BTH_rxBufferHead + 1u);

                /* Adjust local head index */
                if(UART_BTH_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == UART_BTH_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    UART_BTH_rxBufferOverflow = (uint8) UART_BTH_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    UART_BTH_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    UART_BTH_rxBufferHead = locHead;
                }
            }

            UART_BTH_ClearRxInterruptSource(UART_BTH_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if(UART_BTH_CHECK_TX_SW_BUFFER)
    {
        if(UART_BTH_CHECK_INTR_TX_MASKED(UART_BTH_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(UART_BTH_FIFO_SIZE != UART_BTH_GET_TX_FIFO_ENTRIES)
            {
                /* There is a data in TX software buffer */
                if(UART_BTH_txBufferHead != UART_BTH_txBufferTail)
                {
                    /* Mode local tail index */
                    locTail = (UART_BTH_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(UART_BTH_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    UART_BTH_TX_FIFO_WR_REG = UART_BTH_GetWordFromTxBuffer(locTail);

                    /* Mode tail index */
                    UART_BTH_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is EMPTY: end of transmitting */
                    UART_BTH_DISABLE_INTR_TX(UART_BTH_INTR_TX_NOT_FULL);
                    break;
                }
            }

            UART_BTH_ClearTxInterruptSource(UART_BTH_INTR_TX_NOT_FULL);
        }
    }
    #endif
}


/* [] END OF FILE */
