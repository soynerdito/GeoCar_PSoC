/*******************************************************************************
* File Name: BLUE_SPI_UART.c
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

#include "BLUE_PVT.h"
#include "BLUE_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
    volatile uint32 BLUE_rxBufferHead;
    volatile uint32 BLUE_rxBufferTail;
    volatile uint8  BLUE_rxBufferOverflow;
#endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

#if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
    volatile uint32 BLUE_txBufferHead;
    volatile uint32 BLUE_txBufferTail;
#endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */

#if(BLUE_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 BLUE_rxBufferInternal[BLUE_RX_BUFFER_SIZE];
#endif /* (BLUE_INTERNAL_RX_SW_BUFFER) */

#if(BLUE_INTERNAL_TX_SW_BUFFER)
    volatile uint8 BLUE_txBufferInternal[BLUE_TX_BUFFER_SIZE];
#endif /* (BLUE_INTERNAL_TX_SW_BUFFER) */


#if(BLUE_RX_DIRECTION)

    /*******************************************************************************
    * Function Name: BLUE_SpiUartReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer. Undefined data will
    *  be returned if the RX buffer is empty.
    *  Call BLUE_SpiUartGetRxBufferSize() to return buffer size.
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
    *  Look into BLUE_SpiInit for description.
    *
    *******************************************************************************/
    uint32 BLUE_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

        #if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(BLUE_CHECK_RX_SW_BUFFER)
        {
            if(BLUE_rxBufferHead != BLUE_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (BLUE_rxBufferTail + 1u);

                if(BLUE_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data fron RX software buffer */
                rxData = BLUE_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                BLUE_rxBufferTail = locTail;
            }
        }
        #else
        {
            rxData = BLUE_RX_FIFO_RD_REG; /* Read data from RX FIFO */
        }
        #endif

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: BLUE_SpiUartGetRxBufferSize
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
    uint32 BLUE_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
        #if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locHead;
        #endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(BLUE_CHECK_RX_SW_BUFFER)
        {
            locHead = BLUE_rxBufferHead;

            if(locHead >= BLUE_rxBufferTail)
            {
                size = (locHead - BLUE_rxBufferTail);
            }
            else
            {
                size = (locHead + (BLUE_RX_BUFFER_SIZE - BLUE_rxBufferTail));
            }
        }
        #else
        {
            size = BLUE_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: BLUE_SpiUartClearRxBuffer
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
    void BLUE_SpiUartClearRxBuffer(void)
    {
        #if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(BLUE_CHECK_RX_SW_BUFFER)
        {
            intSourceMask = BLUE_SpiUartDisableIntRx();

            BLUE_CLEAR_RX_FIFO;

            /* Flush RX software buffer */
            BLUE_rxBufferHead     = BLUE_rxBufferTail;
            BLUE_rxBufferOverflow = 0u;

            /* End RX transfer */
            BLUE_ClearRxInterruptSource(BLUE_INTR_RX_ALL);

            BLUE_SpiUartEnableIntRx(intSourceMask);
        }
        #else
        {
            BLUE_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (BLUE_RX_DIRECTION) */


#if(BLUE_TX_DIRECTION)

    /*******************************************************************************
    * Function Name: BLUE_SpiUartWriteTxData
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
    void BLUE_SpiUartWriteTxData(uint32 txDataByte)
    {
        #if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locHead;
            uint32 intSourceMask;
        #endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(BLUE_CHECK_TX_SW_BUFFER)
        {
            /* Head index to put data */
            locHead = (BLUE_txBufferHead + 1u);

            /* Adjust TX software buffer index */
            if(BLUE_TX_BUFFER_SIZE == locHead)
            {
                locHead = 0u;
            }

            while(locHead == BLUE_txBufferTail)
            {
                /* Wait for space in the TX software buffer */
            }

            /* The TX software buffer has at least one room */

            if((BLUE_txBufferHead == BLUE_txBufferTail) &&
               (BLUE_FIFO_SIZE != BLUE_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                BLUE_TX_FIFO_WR_REG = txDataByte;
            }
            /* Put data in the TX software buffer */
            else
            {
                /* Clear old status of INTR_TX_EMPTY. It sets at the end of transfer: TX FIFO empty. */
                BLUE_ClearTxInterruptSource(BLUE_INTR_TX_NOT_FULL);

                BLUE_PutWordInTxBuffer(locHead, txDataByte);

                BLUE_txBufferHead = locHead;

                /* Enable interrupt to transmit */
                intSourceMask  = BLUE_INTR_TX_NOT_FULL;
                intSourceMask |= BLUE_GetTxInterruptMode();
                BLUE_SpiUartEnableIntTx(intSourceMask);
            }
        }
        #else
        {
            while(BLUE_FIFO_SIZE == BLUE_GET_TX_FIFO_ENTRIES)
            {
                /* Block while TX FIFO is FULL */
            }

            BLUE_TX_FIFO_WR_REG = txDataByte;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: BLUE_SpiUartPutArray
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
    void BLUE_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for(i=0u; i < count; i++)
        {
            BLUE_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_SpiUartGetTxBufferSize
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
    uint32 BLUE_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
        #if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(BLUE_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = BLUE_txBufferTail;

            if(BLUE_txBufferHead >= locTail)
            {
                size = (BLUE_txBufferHead - locTail);
            }
            else
            {
                size = (BLUE_txBufferHead + (BLUE_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = BLUE_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: BLUE_SpiUartClearTxBuffer
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
    void BLUE_SpiUartClearTxBuffer(void)
    {
        #if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(BLUE_CHECK_TX_SW_BUFFER)
        {
            intSourceMask = BLUE_SpiUartDisableIntTx();

            BLUE_CLEAR_TX_FIFO;

            /* Flush TX software buffer */
            BLUE_txBufferHead = BLUE_txBufferTail;

            /* End TX transfer if it is in progress */
            intSourceMask &= (uint32) ~BLUE_INTR_TX_NOT_FULL;

            BLUE_SpiUartEnableIntTx(intSourceMask);
        }
        #else
        {
            BLUE_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (BLUE_TX_DIRECTION) */


/*******************************************************************************
* Function Name: BLUE_SpiUartDisableIntRx
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
uint32 BLUE_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = BLUE_GetRxInterruptMode();

    BLUE_SetRxInterruptMode(BLUE_NO_INTR_SOURCES);

    return(intSource);
}


/*******************************************************************************
* Function Name: BLUE_SpiUartDisableIntTx
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
uint32 BLUE_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = BLUE_GetTxInterruptMode();

    BLUE_SetTxInterruptMode(BLUE_NO_INTR_SOURCES);

    return(intSourceMask);
}


#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)

    /*******************************************************************************
    * Function Name: BLUE_PutWordInRxBuffer
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
    void BLUE_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in the buffer */
        if(BLUE_ONE_BYTE_WIDTH == BLUE_rxDataBits)
        {
            BLUE_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            BLUE_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            BLUE_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_GetWordFromRxBuffer
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
    uint32 BLUE_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if(BLUE_ONE_BYTE_WIDTH == BLUE_rxDataBits)
        {
            value = BLUE_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) BLUE_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)BLUE_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }


    /*******************************************************************************
    * Function Name: BLUE_PutWordInTxBuffer
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
    void BLUE_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in the buffer */
        if(BLUE_ONE_BYTE_WIDTH == BLUE_txDataBits)
        {
            BLUE_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            BLUE_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            BLUE_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_GetWordFromTxBuffer
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
    uint32 BLUE_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if(BLUE_ONE_BYTE_WIDTH == BLUE_txDataBits)
        {
            value = (uint32) BLUE_txBuffer[idx];
        }
        else
        {
            value  = (uint32) BLUE_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) BLUE_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }

#endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
