/*******************************************************************************
* File Name: BLUE_INT.c
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

#include "BLUE.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (BLUE_RX_ENABLED || BLUE_HD_ENABLED) && \
     (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: BLUE_RXISR
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
    *  BLUE_rxBuffer - RAM buffer pointer for save received data.
    *  BLUE_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  BLUE_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  BLUE_rxBufferOverflow - software overflow flag. Set to one
    *     when BLUE_rxBufferWrite index overtakes
    *     BLUE_rxBufferRead index.
    *  BLUE_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when BLUE_rxBufferWrite is equal to
    *    BLUE_rxBufferRead
    *  BLUE_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  BLUE_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(BLUE_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START BLUE_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = BLUE_RXSTATUS_REG;

        if((readData & (BLUE_RX_STS_BREAK | BLUE_RX_STS_PAR_ERROR |
                        BLUE_RX_STS_STOP_ERROR | BLUE_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START BLUE_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & BLUE_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (BLUE_RXHW_ADDRESS_ENABLED)
                if(BLUE_rxAddressMode == (uint8)BLUE__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & BLUE_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & BLUE_RX_STS_ADDR_MATCH) != 0u)
                        {
                            BLUE_rxAddressDetected = 1u;
                        }
                        else
                        {
                            BLUE_rxAddressDetected = 0u;
                        }
                    }

                    readData = BLUE_RXDATA_REG;
                    if(BLUE_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        BLUE_rxBuffer[BLUE_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    BLUE_rxBuffer[BLUE_rxBufferWrite] = BLUE_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                BLUE_rxBuffer[BLUE_rxBufferWrite] = BLUE_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(BLUE_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    BLUE_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                BLUE_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(BLUE_rxBufferWrite >= BLUE_RXBUFFERSIZE)
                {
                    BLUE_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(BLUE_rxBufferWrite == BLUE_rxBufferRead)
                {
                    BLUE_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(BLUE_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        BLUE_RXSTATUS_MASK_REG  &= (uint8)~BLUE_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(BLUE_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End BLUE_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = BLUE_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START BLUE_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End BLUE_RX_ENABLED && (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH) */


#if(BLUE_TX_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: BLUE_TXISR
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
    *  BLUE_txBuffer - RAM buffer pointer for transmit data from.
    *  BLUE_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  BLUE_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(BLUE_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START BLUE_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((BLUE_txBufferRead != BLUE_txBufferWrite) &&
             ((BLUE_TXSTATUS_REG & BLUE_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(BLUE_txBufferRead >= BLUE_TXBUFFERSIZE)
            {
                BLUE_txBufferRead = 0u;
            }

            BLUE_TXDATA_REG = BLUE_txBuffer[BLUE_txBufferRead];

            /* Set next pointer. */
            BLUE_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START BLUE_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End BLUE_TX_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH) */


/* [] END OF FILE */
