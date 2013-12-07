/*******************************************************************************
* File Name: BLUE_SPI_UART_INT.c
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

#include "BLUE_PVT.h"
#include "BLUE_SPI_UART_PVT.h"


/*******************************************************************************
* Function Name: BLUE_SPI_UART_ISR
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
CY_ISR(BLUE_SPI_UART_ISR)
{
    #if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locHead;
        uint32 dataRx;
    #endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */

    if(NULL != BLUE_customIntrHandler)
    {
        BLUE_customIntrHandler(); /* Call customer routine if needed */
    }

    #if(BLUE_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        BLUE_ClearSpiExtClkInterruptSource(BLUE_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if(BLUE_CHECK_RX_SW_BUFFER)
    {
        /* Get data from RX FIFO */
        if(BLUE_CHECK_INTR_RX_MASKED(BLUE_INTR_RX_NOT_EMPTY))
        {
            while(0u != BLUE_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = BLUE_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (BLUE_rxBufferHead + 1u);

                /* Adjust local head index */
                if(BLUE_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == BLUE_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    BLUE_rxBufferOverflow = (uint8) BLUE_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    BLUE_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    BLUE_rxBufferHead = locHead;
                }
            }

            BLUE_ClearRxInterruptSource(BLUE_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if(BLUE_CHECK_TX_SW_BUFFER)
    {
        if(BLUE_CHECK_INTR_TX_MASKED(BLUE_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(BLUE_FIFO_SIZE != BLUE_GET_TX_FIFO_ENTRIES)
            {
                /* There is a data in TX software buffer */
                if(BLUE_txBufferHead != BLUE_txBufferTail)
                {
                    /* Mode local tail index */
                    locTail = (BLUE_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(BLUE_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    BLUE_TX_FIFO_WR_REG = BLUE_GetWordFromTxBuffer(locTail);

                    /* Mode tail index */
                    BLUE_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is EMPTY: end of transmitting */
                    BLUE_DISABLE_INTR_TX(BLUE_INTR_TX_NOT_FULL);
                    break;
                }
            }

            BLUE_ClearTxInterruptSource(BLUE_INTR_TX_NOT_FULL);
        }
    }
    #endif
}


/* [] END OF FILE */
