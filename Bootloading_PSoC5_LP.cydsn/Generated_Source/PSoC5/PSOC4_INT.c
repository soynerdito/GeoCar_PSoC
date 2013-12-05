/*******************************************************************************
* File Name: PSOC4_INT.c
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

#include "PSOC4.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (PSOC4_RX_ENABLED || PSOC4_HD_ENABLED) && \
     (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: PSOC4_RXISR
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
    *  PSOC4_rxBuffer - RAM buffer pointer for save received data.
    *  PSOC4_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  PSOC4_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  PSOC4_rxBufferOverflow - software overflow flag. Set to one
    *     when PSOC4_rxBufferWrite index overtakes
    *     PSOC4_rxBufferRead index.
    *  PSOC4_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when PSOC4_rxBufferWrite is equal to
    *    PSOC4_rxBufferRead
    *  PSOC4_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  PSOC4_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(PSOC4_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START PSOC4_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = PSOC4_RXSTATUS_REG;

        if((readData & (PSOC4_RX_STS_BREAK | PSOC4_RX_STS_PAR_ERROR |
                        PSOC4_RX_STS_STOP_ERROR | PSOC4_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START PSOC4_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & PSOC4_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (PSOC4_RXHW_ADDRESS_ENABLED)
                if(PSOC4_rxAddressMode == (uint8)PSOC4__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & PSOC4_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & PSOC4_RX_STS_ADDR_MATCH) != 0u)
                        {
                            PSOC4_rxAddressDetected = 1u;
                        }
                        else
                        {
                            PSOC4_rxAddressDetected = 0u;
                        }
                    }

                    readData = PSOC4_RXDATA_REG;
                    if(PSOC4_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        PSOC4_rxBuffer[PSOC4_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    PSOC4_rxBuffer[PSOC4_rxBufferWrite] = PSOC4_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                PSOC4_rxBuffer[PSOC4_rxBufferWrite] = PSOC4_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(PSOC4_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    PSOC4_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                PSOC4_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(PSOC4_rxBufferWrite >= PSOC4_RXBUFFERSIZE)
                {
                    PSOC4_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(PSOC4_rxBufferWrite == PSOC4_rxBufferRead)
                {
                    PSOC4_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(PSOC4_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        PSOC4_RXSTATUS_MASK_REG  &= (uint8)~PSOC4_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(PSOC4_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End PSOC4_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = PSOC4_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START PSOC4_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End PSOC4_RX_ENABLED && (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH) */


#if(PSOC4_TX_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: PSOC4_TXISR
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
    *  PSOC4_txBuffer - RAM buffer pointer for transmit data from.
    *  PSOC4_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  PSOC4_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(PSOC4_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START PSOC4_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((PSOC4_txBufferRead != PSOC4_txBufferWrite) &&
             ((PSOC4_TXSTATUS_REG & PSOC4_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(PSOC4_txBufferRead >= PSOC4_TXBUFFERSIZE)
            {
                PSOC4_txBufferRead = 0u;
            }

            PSOC4_TXDATA_REG = PSOC4_txBuffer[PSOC4_txBufferRead];

            /* Set next pointer. */
            PSOC4_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START PSOC4_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End PSOC4_TX_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH) */


/* [] END OF FILE */
