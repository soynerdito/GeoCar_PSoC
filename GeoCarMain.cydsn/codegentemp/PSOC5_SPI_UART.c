/*******************************************************************************
* File Name: PSOC5_SPI_UART.c
* Version 1.10
*
* Description:
*  This file provides the source code to the API for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
*******************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PSOC5_PVT.h"
#include "PSOC5_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
    volatile uint32 PSOC5_rxBufferHead;
    volatile uint32 PSOC5_rxBufferTail;
    volatile uint8  PSOC5_rxBufferOverflow;
#endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

#if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
    volatile uint32 PSOC5_txBufferHead;
    volatile uint32 PSOC5_txBufferTail;
#endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */

#if(PSOC5_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 PSOC5_rxBufferInternal[PSOC5_RX_BUFFER_SIZE];
#endif /* (PSOC5_INTERNAL_RX_SW_BUFFER) */

#if(PSOC5_INTERNAL_TX_SW_BUFFER)
    volatile uint8 PSOC5_txBufferInternal[PSOC5_TX_BUFFER_SIZE];
#endif /* (PSOC5_INTERNAL_TX_SW_BUFFER) */


#if(PSOC5_RX_DIRECTION)

    /*******************************************************************************
    * Function Name: PSOC5_SpiUartReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer. Undefined data will
    *  be returned if the RX buffer is empty.
    *  Call PSOC5_SpiUartGetRxBufferSize() to return buffer size.
    *   - RX software buffer disabled: Returns data element retrieved from RX FIFO.
    *   - RX software buffer enabled: Returns data element from the software
    *     receive buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Next data element from the receive buffer.
    *
    * Global Variables:
    *  Look into PSOC5_SpiInit for description.
    *
    *******************************************************************************/
    uint32 PSOC5_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

        #if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(PSOC5_CHECK_RX_SW_BUFFER)
        {
            if(PSOC5_rxBufferHead != PSOC5_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (PSOC5_rxBufferTail + 1u);

                if(PSOC5_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data fron RX software buffer */
                rxData = PSOC5_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                PSOC5_rxBufferTail = locTail;
            }
        }
        #else
        {
            rxData = PSOC5_RX_FIFO_RD_REG; /* Read data from RX FIFO */
        }
        #endif

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PSOC5_SpiUartGetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received data elements in the receive buffer.
    *   - RX software buffer disabled: returns the number of used entries in
    *     RX FIFO.
    *   - RX software buffer enabled: returns the number of elements which were
    *     placed in receive buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Number of received data elements
    *
    *******************************************************************************/
    uint32 PSOC5_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
        #if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locHead;
        #endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(PSOC5_CHECK_RX_SW_BUFFER)
        {
            locHead = PSOC5_rxBufferHead;

            if(locHead >= PSOC5_rxBufferTail)
            {
                size = (locHead - PSOC5_rxBufferTail);
            }
            else
            {
                size = (locHead + (PSOC5_RX_BUFFER_SIZE - PSOC5_rxBufferTail));
            }
        }
        #else
        {
            size = PSOC5_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: PSOC5_SpiUartClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clear the receive buffer and RX FIFO.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_SpiUartClearRxBuffer(void)
    {
        #if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(PSOC5_CHECK_RX_SW_BUFFER)
        {
            intSourceMask = PSOC5_SpiUartDisableIntRx();

            PSOC5_CLEAR_RX_FIFO;

            /* Flush RX software buffer */
            PSOC5_rxBufferHead     = PSOC5_rxBufferTail;
            PSOC5_rxBufferOverflow = 0u;

            /* End RX transfer */
            PSOC5_ClearRxInterruptSource(PSOC5_INTR_RX_ALL);

            PSOC5_SpiUartEnableIntRx(intSourceMask);
        }
        #else
        {
            PSOC5_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (PSOC5_RX_DIRECTION) */


#if(PSOC5_TX_DIRECTION)

    /*******************************************************************************
    * Function Name: PSOC5_SpiUartWriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a data entry into the transmit buffer to be sent at the next available
    *  bus time.
    *  This function is blocking and waits until there is space available to put the
    *  requested data in the transmit buffer.
    *
    * Parameters:
    *  txDataByte: the data to be transmitted.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_SpiUartWriteTxData(uint32 txDataByte)
    {
        #if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locHead;
            uint32 intSourceMask;
        #endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(PSOC5_CHECK_TX_SW_BUFFER)
        {
            /* Head index to put data */
            locHead = (PSOC5_txBufferHead + 1u);

            /* Adjust TX software buffer index */
            if(PSOC5_TX_BUFFER_SIZE == locHead)
            {
                locHead = 0u;
            }

            while(locHead == PSOC5_txBufferTail)
            {
                /* Wait for space in the TX software buffer */
            }

            /* The TX software buffer has at least one room */

            if((PSOC5_txBufferHead == PSOC5_txBufferTail) &&
               (PSOC5_FIFO_SIZE != PSOC5_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                PSOC5_TX_FIFO_WR_REG = txDataByte;
            }
            /* Put data in the TX software buffer */
            else
            {
                /* Clear old status of INTR_TX_EMPTY. It sets at the end of transfer: TX FIFO empty. */
                PSOC5_ClearTxInterruptSource(PSOC5_INTR_TX_NOT_FULL);

                PSOC5_PutWordInTxBuffer(locHead, txDataByte);

                PSOC5_txBufferHead = locHead;

                /* Enable interrupt to transmit */
                intSourceMask  = PSOC5_INTR_TX_NOT_FULL;
                intSourceMask |= PSOC5_GetTxInterruptMode();
                PSOC5_SpiUartEnableIntTx(intSourceMask);
            }
        }
        #else
        {
            while(PSOC5_FIFO_SIZE == PSOC5_GET_TX_FIFO_ENTRIES)
            {
                /* Block while TX FIFO is FULL */
            }

            PSOC5_TX_FIFO_WR_REG = txDataByte;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: PSOC5_SpiUartPutArray
    ********************************************************************************
    *
    * Summary:
    *  Places an array of data into the transmit buffer to be sent.
    *  This function is blocking and waits until there is a space available to put
    *  all the requested data in the transmit buffer. The array size can be greater
    *  than transmit buffer size.
    *
    * Parameters:
    *  wrBuf:  pointer to an array with data to be placed in transmit buffer.
    *  count:  number of data elements to be placed in the transmit buffer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for(i=0u; i < count; i++)
        {
            PSOC5_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: PSOC5_SpiUartGetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of elements currently in the transmit buffer.
    *  TX software buffer disabled: returns the number of used entries in TX FIFO.
    *  TX software buffer enabled: returns the number of elements currently used
    *  in the transmit buffer. This number does not include used entries in the
    *  TX FIFO. The transmit buffer size is zero until the TX FIFO is full.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Number of data elements ready to transmit.
    *
    *******************************************************************************/
    uint32 PSOC5_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
        #if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(PSOC5_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = PSOC5_txBufferTail;

            if(PSOC5_txBufferHead >= locTail)
            {
                size = (PSOC5_txBufferHead - locTail);
            }
            else
            {
                size = (PSOC5_txBufferHead + (PSOC5_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = PSOC5_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: PSOC5_SpiUartClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the transmit buffer and TX FIFO.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_SpiUartClearTxBuffer(void)
    {
        #if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(PSOC5_CHECK_TX_SW_BUFFER)
        {
            intSourceMask = PSOC5_SpiUartDisableIntTx();

            PSOC5_CLEAR_TX_FIFO;

            /* Flush TX software buffer */
            PSOC5_txBufferHead = PSOC5_txBufferTail;

            /* End TX transfer if it is in progress */
            intSourceMask &= (uint32) ~PSOC5_INTR_TX_NOT_FULL;

            PSOC5_SpiUartEnableIntTx(intSourceMask);
        }
        #else
        {
            PSOC5_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (PSOC5_TX_DIRECTION) */


/*******************************************************************************
* Function Name: PSOC5_SpiUartDisableIntRx
********************************************************************************
*
* Summary:
*  Disables RX interrupt sources.
*
* Parameters:
*  None
*
* Return:
*  Returns RX interrupt soureces enabled before function call.
*
*******************************************************************************/
uint32 PSOC5_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = PSOC5_GetRxInterruptMode();

    PSOC5_SetRxInterruptMode(PSOC5_NO_INTR_SOURCES);

    return(intSource);
}


/*******************************************************************************
* Function Name: PSOC5_SpiUartDisableIntTx
********************************************************************************
*
* Summary:
*  Disables TX interrupt sources.
*
* Parameters:
*  None
*
* Return:
*  Returns TX interrupt soureces enabled before function call.
*
*******************************************************************************/
uint32 PSOC5_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = PSOC5_GetTxInterruptMode();

    PSOC5_SetTxInterruptMode(PSOC5_NO_INTR_SOURCES);

    return(intSourceMask);
}


#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)

    /*******************************************************************************
    * Function Name: PSOC5_PutWordInRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Stores byte/word into the RX buffer.
    *  Only available in Unconfigured operation mode.
    *
    * Parameters:
    *  index:      index to store data byte/word in the RX buffer.
    *  rxDataByte: byte/word to store.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in the buffer */
        if(PSOC5_ONE_BYTE_WIDTH == PSOC5_rxDataBits)
        {
            PSOC5_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            PSOC5_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            PSOC5_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: PSOC5_GetWordFromRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Reads byte/word from RX buffer.
    *  Only available in Unconfigured operation mode.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Returns byte/word read from RX buffer.
    *
    *******************************************************************************/
    uint32 PSOC5_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if(PSOC5_ONE_BYTE_WIDTH == PSOC5_rxDataBits)
        {
            value = PSOC5_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) PSOC5_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)PSOC5_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }


    /*******************************************************************************
    * Function Name: PSOC5_PutWordInTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Stores byte/word into the TX buffer.
    * Only available in Unconfigured operation mode.
    *
    * Parameters:
    *  idx:        index to store data byte/word in the TX buffer.
    *  txDataByte: byte/word to store.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in the buffer */
        if(PSOC5_ONE_BYTE_WIDTH == PSOC5_txDataBits)
        {
            PSOC5_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            PSOC5_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            PSOC5_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: PSOC5_GetWordFromTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Reads byte/word from TX buffer.
    *  Only available in Unconfigured operation mode.
    *
    * Parameters:
    *  idx: index to get data byte/word from the TX buffer.
    *
    * Return:
    *  Returns byte/word read from TX buffer.
    *
    *******************************************************************************/
    uint32 PSOC5_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if(PSOC5_ONE_BYTE_WIDTH == PSOC5_txDataBits)
        {
            value = (uint32) PSOC5_txBuffer[idx];
        }
        else
        {
            value  = (uint32) PSOC5_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) PSOC5_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
