/*******************************************************************************
* File Name: GPS_INT.c
* Version 2.30
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
* Note:
*  Any unusual or non-standard behavior should be noted here. Other-
*  wise, this section should remain blank.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "GPS.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (GPS_RX_ENABLED || GPS_HD_ENABLED) && \
     (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: GPS_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_rxBuffer - RAM buffer pointer for save received data.
    *  GPS_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  GPS_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  GPS_rxBufferOverflow - software overflow flag. Set to one
    *     when GPS_rxBufferWrite index overtakes
    *     GPS_rxBufferRead index.
    *  GPS_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when GPS_rxBufferWrite is equal to
    *    GPS_rxBufferRead
    *  GPS_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  GPS_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(GPS_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START GPS_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = GPS_RXSTATUS_REG;

        if((readData & (GPS_RX_STS_BREAK | GPS_RX_STS_PAR_ERROR |
                        GPS_RX_STS_STOP_ERROR | GPS_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START GPS_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & GPS_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (GPS_RXHW_ADDRESS_ENABLED)
                if(GPS_rxAddressMode == (uint8)GPS__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & GPS_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & GPS_RX_STS_ADDR_MATCH) != 0u)
                        {
                            GPS_rxAddressDetected = 1u;
                        }
                        else
                        {
                            GPS_rxAddressDetected = 0u;
                        }
                    }

                    readData = GPS_RXDATA_REG;
                    if(GPS_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        GPS_rxBuffer[GPS_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    GPS_rxBuffer[GPS_rxBufferWrite] = GPS_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                GPS_rxBuffer[GPS_rxBufferWrite] = GPS_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(GPS_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    GPS_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                GPS_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(GPS_rxBufferWrite >= GPS_RXBUFFERSIZE)
                {
                    GPS_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(GPS_rxBufferWrite == GPS_rxBufferRead)
                {
                    GPS_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(GPS_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        GPS_RXSTATUS_MASK_REG  &= (uint8)~GPS_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(GPS_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End GPS_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = GPS_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START GPS_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End GPS_RX_ENABLED && (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH) */


#if(GPS_TX_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: GPS_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_txBuffer - RAM buffer pointer for transmit data from.
    *  GPS_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  GPS_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(GPS_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START GPS_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((GPS_txBufferRead != GPS_txBufferWrite) &&
             ((GPS_TXSTATUS_REG & GPS_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(GPS_txBufferRead >= GPS_TXBUFFERSIZE)
            {
                GPS_txBufferRead = 0u;
            }

            GPS_TXDATA_REG = GPS_txBuffer[GPS_txBufferRead];

            /* Set next pointer. */
            GPS_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START GPS_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End GPS_TX_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH) */


/* [] END OF FILE */
