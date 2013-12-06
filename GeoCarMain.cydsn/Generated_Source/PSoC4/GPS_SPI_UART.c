/*******************************************************************************
* File Name: GPS_SPI_UART.c
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

#include "GPS_PVT.h"
#include "GPS_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
    volatile uint32 GPS_rxBufferHead;
    volatile uint32 GPS_rxBufferTail;
    volatile uint8  GPS_rxBufferOverflow;
#endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

#if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
    volatile uint32 GPS_txBufferHead;
    volatile uint32 GPS_txBufferTail;
#endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */

#if(GPS_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 GPS_rxBufferInternal[GPS_RX_BUFFER_SIZE];
#endif /* (GPS_INTERNAL_RX_SW_BUFFER) */

#if(GPS_INTERNAL_TX_SW_BUFFER)
    volatile uint8 GPS_txBufferInternal[GPS_TX_BUFFER_SIZE];
#endif /* (GPS_INTERNAL_TX_SW_BUFFER) */


#if(GPS_RX_DIRECTION)

    /*******************************************************************************
    * Function Name: GPS_SpiUartReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer. Undefined data will
    *  be returned if the RX buffer is empty.
    *  Call GPS_SpiUartGetRxBufferSize() to return buffer size.
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
    *  Look into GPS_SpiInit for description.
    *
    *******************************************************************************/
    uint32 GPS_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

        #if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(GPS_CHECK_RX_SW_BUFFER)
        {
            if(GPS_rxBufferHead != GPS_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (GPS_rxBufferTail + 1u);

                if(GPS_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data fron RX software buffer */
                rxData = GPS_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                GPS_rxBufferTail = locTail;
            }
        }
        #else
        {
            rxData = GPS_RX_FIFO_RD_REG; /* Read data from RX FIFO */
        }
        #endif

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: GPS_SpiUartGetRxBufferSize
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
    uint32 GPS_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
        #if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locHead;
        #endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(GPS_CHECK_RX_SW_BUFFER)
        {
            locHead = GPS_rxBufferHead;

            if(locHead >= GPS_rxBufferTail)
            {
                size = (locHead - GPS_rxBufferTail);
            }
            else
            {
                size = (locHead + (GPS_RX_BUFFER_SIZE - GPS_rxBufferTail));
            }
        }
        #else
        {
            size = GPS_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: GPS_SpiUartClearRxBuffer
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
    void GPS_SpiUartClearRxBuffer(void)
    {
        #if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(GPS_CHECK_RX_SW_BUFFER)
        {
            intSourceMask = GPS_SpiUartDisableIntRx();

            GPS_CLEAR_RX_FIFO;

            /* Flush RX software buffer */
            GPS_rxBufferHead     = GPS_rxBufferTail;
            GPS_rxBufferOverflow = 0u;

            /* End RX transfer */
            GPS_ClearRxInterruptSource(GPS_INTR_RX_ALL);

            GPS_SpiUartEnableIntRx(intSourceMask);
        }
        #else
        {
            GPS_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (GPS_RX_DIRECTION) */


#if(GPS_TX_DIRECTION)

    /*******************************************************************************
    * Function Name: GPS_SpiUartWriteTxData
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
    void GPS_SpiUartWriteTxData(uint32 txDataByte)
    {
        #if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locHead;
            uint32 intSourceMask;
        #endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(GPS_CHECK_TX_SW_BUFFER)
        {
            /* Head index to put data */
            locHead = (GPS_txBufferHead + 1u);

            /* Adjust TX software buffer index */
            if(GPS_TX_BUFFER_SIZE == locHead)
            {
                locHead = 0u;
            }

            while(locHead == GPS_txBufferTail)
            {
                /* Wait for space in the TX software buffer */
            }

            /* The TX software buffer has at least one room */

            if((GPS_txBufferHead == GPS_txBufferTail) &&
               (GPS_FIFO_SIZE != GPS_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                GPS_TX_FIFO_WR_REG = txDataByte;
            }
            /* Put data in the TX software buffer */
            else
            {
                /* Clear old status of INTR_TX_EMPTY. It sets at the end of transfer: TX FIFO empty. */
                GPS_ClearTxInterruptSource(GPS_INTR_TX_NOT_FULL);

                GPS_PutWordInTxBuffer(locHead, txDataByte);

                GPS_txBufferHead = locHead;

                /* Enable interrupt to transmit */
                intSourceMask  = GPS_INTR_TX_NOT_FULL;
                intSourceMask |= GPS_GetTxInterruptMode();
                GPS_SpiUartEnableIntTx(intSourceMask);
            }
        }
        #else
        {
            while(GPS_FIFO_SIZE == GPS_GET_TX_FIFO_ENTRIES)
            {
                /* Block while TX FIFO is FULL */
            }

            GPS_TX_FIFO_WR_REG = txDataByte;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: GPS_SpiUartPutArray
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
    void GPS_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for(i=0u; i < count; i++)
        {
            GPS_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: GPS_SpiUartGetTxBufferSize
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
    uint32 GPS_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
        #if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(GPS_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = GPS_txBufferTail;

            if(GPS_txBufferHead >= locTail)
            {
                size = (GPS_txBufferHead - locTail);
            }
            else
            {
                size = (GPS_txBufferHead + (GPS_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = GPS_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: GPS_SpiUartClearTxBuffer
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
    void GPS_SpiUartClearTxBuffer(void)
    {
        #if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(GPS_CHECK_TX_SW_BUFFER)
        {
            intSourceMask = GPS_SpiUartDisableIntTx();

            GPS_CLEAR_TX_FIFO;

            /* Flush TX software buffer */
            GPS_txBufferHead = GPS_txBufferTail;

            /* End TX transfer if it is in progress */
            intSourceMask &= (uint32) ~GPS_INTR_TX_NOT_FULL;

            GPS_SpiUartEnableIntTx(intSourceMask);
        }
        #else
        {
            GPS_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (GPS_TX_DIRECTION) */


/*******************************************************************************
* Function Name: GPS_SpiUartDisableIntRx
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
uint32 GPS_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = GPS_GetRxInterruptMode();

    GPS_SetRxInterruptMode(GPS_NO_INTR_SOURCES);

    return(intSource);
}


/*******************************************************************************
* Function Name: GPS_SpiUartDisableIntTx
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
uint32 GPS_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = GPS_GetTxInterruptMode();

    GPS_SetTxInterruptMode(GPS_NO_INTR_SOURCES);

    return(intSourceMask);
}


#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)

    /*******************************************************************************
    * Function Name: GPS_PutWordInRxBuffer
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
    void GPS_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in the buffer */
        if(GPS_ONE_BYTE_WIDTH == GPS_rxDataBits)
        {
            GPS_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            GPS_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            GPS_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: GPS_GetWordFromRxBuffer
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
    uint32 GPS_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if(GPS_ONE_BYTE_WIDTH == GPS_rxDataBits)
        {
            value = GPS_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) GPS_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)GPS_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }


    /*******************************************************************************
    * Function Name: GPS_PutWordInTxBuffer
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
    void GPS_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in the buffer */
        if(GPS_ONE_BYTE_WIDTH == GPS_txDataBits)
        {
            GPS_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            GPS_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            GPS_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: GPS_GetWordFromTxBuffer
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
    uint32 GPS_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if(GPS_ONE_BYTE_WIDTH == GPS_txDataBits)
        {
            value = (uint32) GPS_txBuffer[idx];
        }
        else
        {
            value  = (uint32) GPS_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) GPS_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
