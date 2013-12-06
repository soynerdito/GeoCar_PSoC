/*******************************************************************************
* File Name: PSOC5_SPI_UART_INT.c
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

#include "PSOC5_PVT.h"
#include "PSOC5_SPI_UART_PVT.h"


/*******************************************************************************
* Function Name: PSOC5_SPI_UART_ISR
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
CY_ISR(PSOC5_SPI_UART_ISR)
{
    #if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locHead;
        uint32 dataRx;
    #endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */

    if(NULL != PSOC5_customIntrHandler)
    {
        PSOC5_customIntrHandler(); /* Call customer routine if needed */
    }

    #if(PSOC5_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        PSOC5_ClearSpiExtClkInterruptSource(PSOC5_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if(PSOC5_CHECK_RX_SW_BUFFER)
    {
        /* Get data from RX FIFO */
        if(PSOC5_CHECK_INTR_RX_MASKED(PSOC5_INTR_RX_NOT_EMPTY))
        {
            while(0u != PSOC5_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = PSOC5_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (PSOC5_rxBufferHead + 1u);

                /* Adjust local head index */
                if(PSOC5_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == PSOC5_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    PSOC5_rxBufferOverflow = (uint8) PSOC5_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    PSOC5_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    PSOC5_rxBufferHead = locHead;
                }
            }

            PSOC5_ClearRxInterruptSource(PSOC5_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if(PSOC5_CHECK_TX_SW_BUFFER)
    {
        if(PSOC5_CHECK_INTR_TX_MASKED(PSOC5_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(PSOC5_FIFO_SIZE != PSOC5_GET_TX_FIFO_ENTRIES)
            {
                /* There is a data in TX software buffer */
                if(PSOC5_txBufferHead != PSOC5_txBufferTail)
                {
                    /* Mode local tail index */
                    locTail = (PSOC5_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(PSOC5_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    PSOC5_TX_FIFO_WR_REG = PSOC5_GetWordFromTxBuffer(locTail);

                    /* Mode tail index */
                    PSOC5_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is EMPTY: end of transmitting */
                    PSOC5_DISABLE_INTR_TX(PSOC5_INTR_TX_NOT_FULL);
                    break;
                }
            }

            PSOC5_ClearTxInterruptSource(PSOC5_INTR_TX_NOT_FULL);
        }
    }
    #endif
}


/* [] END OF FILE */
