/*******************************************************************************
* File Name: PSOC5_UART.c
* Version 1.10
*
* Description:
*  This file provides the source code to the API for the SCB Component in
*  UART mode.
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


#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Config Structure Initialization
    ***************************************/

    const PSOC5_UART_INIT_STRUCT PSOC5_configUart =
    {
        PSOC5_UART_SUB_MODE,
        PSOC5_UART_DIRECTION,
        PSOC5_UART_DATA_BITS_NUM,
        PSOC5_UART_PARITY_TYPE,
        PSOC5_UART_STOP_BITS_NUM,
        PSOC5_UART_OVS_FACTOR,
        PSOC5_UART_IRDA_LOW_POWER,
        PSOC5_UART_MEDIAN_FILTER_ENABLE,
        PSOC5_UART_RETRY_ON_NACK,
        PSOC5_UART_IRDA_POLARITY,
        PSOC5_UART_DROP_ON_PARITY_ERR,
        PSOC5_UART_DROP_ON_FRAME_ERR,
        PSOC5_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        PSOC5_UART_MP_MODE_ENABLE,
        PSOC5_UART_MP_ACCEPT_ADDRESS,
        PSOC5_UART_MP_RX_ADDRESS,
        PSOC5_UART_MP_RX_ADDRESS_MASK,
        PSOC5_SCB_IRQ_INTERNAL,
        PSOC5_UART_INTR_RX_MASK,
        PSOC5_UART_RX_TRIGGER_LEVEL,
        PSOC5_UART_INTR_TX_MASK,
        PSOC5_UART_TX_TRIGGER_LEVEL
    };


    /*******************************************************************************
    * Function Name: PSOC5_UartInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for SPI operation.
    *
    * Parameters:
    *  config:  Pointer to a structure that contains the following ordered list of
    *           fields. These fields match the selections available in the
    *           customizer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_UartInit(const PSOC5_UART_INIT_STRUCT *config)
    {
        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due bad fucntion parameter */
        }
        else
        {
            /* Configure pins */
            PSOC5_SetPins(PSOC5_SCB_MODE_UART, config->mode, config->direction);

            /* Store internal configuration */
            PSOC5_scbMode       = (uint8) PSOC5_SCB_MODE_UART;
            PSOC5_scbEnableWake = (uint8) config->enableWake;
            PSOC5_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            PSOC5_rxBuffer      =         config->rxBuffer;
            PSOC5_rxDataBits    = (uint8) config->dataBits;
            PSOC5_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            PSOC5_txBuffer      =         config->txBuffer;
            PSOC5_txDataBits    = (uint8) config->dataBits;
            PSOC5_txBufferSize  = (uint8) config->txBufferSize;


            /* Configure UART interface */
            if(PSOC5_UART_MODE_IRDA == config->mode)
            {
                /* OVS settigns: IrDA */
                PSOC5_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (PSOC5_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (PSOC5_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settigns: UART and SmartCard */
                PSOC5_CTRL_REG  = PSOC5_GET_CTRL_OVS(config->oversample);
            }

            PSOC5_CTRL_REG     |= PSOC5_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             PSOC5_CTRL_UART;

            /* Confiuure submode: UART, SmartCard or IrDA */
            PSOC5_UART_CTRL_REG = PSOC5_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            PSOC5_UART_RX_CTRL_REG = PSOC5_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        PSOC5_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        PSOC5_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        PSOC5_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        PSOC5_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(PSOC5_UART_PARITY_NONE != config->parity)
            {
               PSOC5_UART_RX_CTRL_REG |= PSOC5_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    PSOC5_UART_RX_CTRL_PARITY_ENABLED;
            }

            PSOC5_RX_CTRL_REG      = PSOC5_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                PSOC5_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                PSOC5_GET_UART_RX_CTRL_ENABLED(config->direction);

            PSOC5_RX_FIFO_CTRL_REG = PSOC5_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            PSOC5_RX_MATCH_REG     = PSOC5_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                PSOC5_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            PSOC5_UART_TX_CTRL_REG = PSOC5_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                PSOC5_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(PSOC5_UART_PARITY_NONE != config->parity)
            {
               PSOC5_UART_TX_CTRL_REG |= PSOC5_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    PSOC5_UART_TX_CTRL_PARITY_ENABLED;
            }

            PSOC5_TX_CTRL_REG      = PSOC5_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                PSOC5_GET_UART_TX_CTRL_ENABLED(config->direction);

            PSOC5_TX_FIFO_CTRL_REG = PSOC5_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);


            /* Configure WAKE interrupt */
            #if(PSOC5_UART_RX_WAKEUP_IRQ)
                PSOC5_RX_WAKEUP_IRQ_StartEx(&PSOC5_UART_WAKEUP_ISR);
                PSOC5_RX_WAKEUP_IRQ_Disable();
            #endif /* (PSOC5_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt with UART handler */
            PSOC5_SCB_IRQ_Disable();
            PSOC5_SCB_IRQ_SetVector(&PSOC5_SPI_UART_ISR);
            PSOC5_SCB_IRQ_SetPriority((uint8)PSOC5_SCB_IRQ_INTC_PRIOR_NUMBER);

            /* Configure interrupt sources */
            PSOC5_INTR_I2C_EC_MASK_REG = PSOC5_NO_INTR_SOURCES;
            PSOC5_INTR_SPI_EC_MASK_REG = PSOC5_NO_INTR_SOURCES;
            PSOC5_INTR_SLAVE_MASK_REG  = PSOC5_NO_INTR_SOURCES;
            PSOC5_INTR_MASTER_MASK_REG = PSOC5_NO_INTR_SOURCES;
            PSOC5_INTR_RX_MASK_REG     = config->rxInterruptMask;
            PSOC5_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Clear RX buffer indexes */
            PSOC5_rxBufferHead     = 0u;
            PSOC5_rxBufferTail     = 0u;
            PSOC5_rxBufferOverflow = 0u;

            /* Clrea TX buffer indexes */
            PSOC5_txBufferHead = 0u;
            PSOC5_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: PSOC5_UartInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for SPI operation.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_UartInit(void)
    {
        /* Configure UART interface */
        PSOC5_CTRL_REG = PSOC5_UART_DEFAULT_CTRL;

        /* Confiuure submode: UART, SmartCard or IrDA */
        PSOC5_UART_CTRL_REG = PSOC5_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        PSOC5_UART_RX_CTRL_REG = PSOC5_UART_DEFAULT_UART_RX_CTRL;
        PSOC5_RX_CTRL_REG      = PSOC5_UART_DEFAULT_RX_CTRL;
        PSOC5_RX_FIFO_CTRL_REG = PSOC5_UART_DEFAULT_RX_FIFO_CTRL;
        PSOC5_RX_MATCH_REG     = PSOC5_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        PSOC5_UART_TX_CTRL_REG = PSOC5_UART_DEFAULT_UART_TX_CTRL;
        PSOC5_TX_CTRL_REG      = PSOC5_UART_DEFAULT_TX_CTRL;
        PSOC5_TX_FIFO_CTRL_REG = PSOC5_UART_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with UART handler */
        #if(PSOC5_SCB_IRQ_INTERNAL)
            PSOC5_SCB_IRQ_Disable();
            PSOC5_SCB_IRQ_SetVector(&PSOC5_SPI_UART_ISR);
            PSOC5_SCB_IRQ_SetPriority((uint8)PSOC5_SCB_IRQ_INTC_PRIOR_NUMBER);
        #endif /* (PSOC5_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
        #if(PSOC5_UART_RX_WAKEUP_IRQ)
            PSOC5_RX_WAKEUP_IRQ_StartEx(&PSOC5_UART_WAKEUP_ISR);
            PSOC5_RX_WAKEUP_IRQ_Disable();
        #endif /* (PSOC5_UART_RX_WAKEUP_IRQ) */
        
        /* Configure interrupt sources */
        PSOC5_INTR_I2C_EC_MASK_REG = PSOC5_UART_DEFAULT_INTR_I2C_EC_MASK;
        PSOC5_INTR_SPI_EC_MASK_REG = PSOC5_UART_DEFAULT_INTR_SPI_EC_MASK;
        PSOC5_INTR_SLAVE_MASK_REG  = PSOC5_UART_DEFAULT_INTR_SLAVE_MASK;
        PSOC5_INTR_MASTER_MASK_REG = PSOC5_UART_DEFAULT_INTR_MASTER_MASK;
        PSOC5_INTR_RX_MASK_REG     = PSOC5_UART_DEFAULT_INTR_RX_MASK;
        PSOC5_INTR_TX_MASK_REG     = PSOC5_UART_DEFAULT_INTR_TX_MASK;

        #if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
            PSOC5_rxBufferHead     = 0u;
            PSOC5_rxBufferTail     = 0u;
            PSOC5_rxBufferOverflow = 0u;
        #endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
            PSOC5_txBufferHead = 0u;
            PSOC5_txBufferTail = 0u;
        #endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */
    }

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: PSOC5_UartSetRxAddress
********************************************************************************
*
* Summary:
*  Sets the hardware detectable receiver address for the UART in Multiprocessor
*  mode.
*
* Parameters:
*  address: Address for hardware address detection.
*
* Return:
*  None
*
*******************************************************************************/
void PSOC5_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = PSOC5_RX_MATCH_REG;

    matchReg &= ((uint32) ~PSOC5_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & PSOC5_RX_MATCH_ADDR_MASK)); /* Set address  */

    PSOC5_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: PSOC5_UartSetRxAddressMask
********************************************************************************
*
* Summary:
*  Sets the hardware address mask for the UART in Multiprocessor mode.
*
* Parameters:
*  addressMask: Address mask.
*   0 - address bit does not care while comparison.
*   1 - address bit is significant while comparison.
*
* Return:
*  None
*
*******************************************************************************/
void PSOC5_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = PSOC5_RX_MATCH_REG;

    matchReg &= ((uint32) ~PSOC5_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << PSOC5_RX_MATCH_MASK_POS));

    PSOC5_RX_MATCH_REG = matchReg;
}


#if(PSOC5_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: PSOC5_UartGetChar
    ********************************************************************************
    *
    * Summary:
    *  Retrieves next data element from receive buffer.
    *  This function is designed for ASCII characters and returns a char
    *  where 1 to 255 is valid characters and 0 indicates an error occurred or
    *  no data is present.
    *  RX software buffer disabled: returns data element retrieved from RX FIFO.
    *  Undefined data will be returned if the RX FIFO is empty.
    *  RX software buffer enabled: Returns data element from the software receive
    *  buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Next data element from the receive buffer.
    *  ASCII character values from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    *******************************************************************************/
    uint32 PSOC5_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Read data only if there is data to read */
        if(0u != PSOC5_SpiUartGetRxBufferSize())
        {
            rxData = PSOC5_SpiUartReadRxData();
        }

        if(PSOC5_CHECK_INTR_RX(PSOC5_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occured: return zero */
            PSOC5_ClearRxInterruptSource(PSOC5_INTR_RX_ERR);
        }

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PSOC5_UartGetByte
    ********************************************************************************
    *
    * Summary:
    *  Retrieves next data element from the receive buffer, returns received byte
    *  and error condition.
    *  RX software buffer disabled: Returns data element retrieved from RX FIFO.
    *  Undefined data will be returned if the RX FIFO is empty.
    *  RX software buffer enabled: Returns data element from the software receive
    *  buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Bits 15-8 contains status and bits 7-0 contains the next data element from
    *  receive buffer. If the bits 15-8 are nonzero, an error has occurred.
    *
    *******************************************************************************/
    uint32 PSOC5_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;
        uint32 intSourceMask;

        intSourceMask = PSOC5_SpiUartDisableIntRx();

        if(0u != PSOC5_SpiUartGetRxBufferSize())
        {
             /*
             * Enable interrupt to receive more bytes: at least one byte is in
             * buffer.
             */
            PSOC5_SpiUartEnableIntRx(intSourceMask);

            /* Get received byte */
            rxData = PSOC5_SpiUartReadRxData();
        }
        else
        {
            /*
            * Read byte directly from RX FIFO: the underflow is raised in case
            * of empty. In other case the first received byte will be read.
            */
            rxData = PSOC5_RX_FIFO_RD_REG;

            /*
            * Enable interrupt to receive more bytes.
            * The RX_NOT_EMPTY interrupt is cleared by interrupt routine in case
            * byte was received and read above.
            */
            PSOC5_SpiUartEnableIntRx(intSourceMask);
        }

        /* Get and clear RX error mask */
        tmpStatus = (PSOC5_GetRxInterruptSource() & PSOC5_INTR_RX_ERR);
        PSOC5_ClearRxInterruptSource(PSOC5_INTR_RX_ERR);

        /*
        * Put together data and error status:
        * MP mode and accept address: the 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return(rxData);
    }

#endif /* (PSOC5_UART_RX_DIRECTION) */


#if(PSOC5_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: PSOC5_UartPutString
    ********************************************************************************
    *
    * Summary:
    *  Places a NULL terminated string in the transmit buffer to be sent at the
    *  next available bus time.
    *  This function is blocking and waits until there is a space available to put
    *  all requested data in transmit buffer.
    *
    * Parameters:
    *  string: pointer to the null terminated string array to be placed in the
    *          transmit buffer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data will be sent */
        while(string[bufIndex] != ((char8) 0))
        {
            PSOC5_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: PSOC5_UartPutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Places byte of data followed by a carriage return (0x0D) and line feed (0x0A)
    *  to the transmit buffer.
    *  This function is blocking and waits until there is a space available to put
    *  all requested data in transmit buffer.
    *
    * Parameters:
    *  txDataByte : the data to be transmitted.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_UartPutCRLF(uint32 txDataByte)
    {
        PSOC5_UartPutChar(txDataByte);  /* Blocks the control flow until all data will be sent */
        PSOC5_UartPutChar(0x0Du);       /* Blocks the control flow until all data will be sent */
        PSOC5_UartPutChar(0x0Au);       /* Blocks the control flow until all data will be sent */
    }
#endif /* (PSOC5_UART_TX_DIRECTION) */


#if(PSOC5_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: PSOC5_UartSaveConfig
    ********************************************************************************
    *
    * Summary:
    *  Wakeup disabled: does nothing.
    *  Wakeup enabled: clears SCB_backup.enableStateto keep component enabled while
    *  DeepSleep. Clears and enables interrupt on falling edge of rx input. The GPIO
    *  event wakes up the device and SKIP_START feature allows UART continue
    *  receiving data bytes properly. The GPIO interrupt does not track in active mode
    *  therefore requires to be cleared by this API. It makes uart wakeup single
    *  triggered event.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_UartSaveConfig(void)
    {
        /* Clear interrupt activy:
        *  - set skip start and disable RX. On GPIO wakeup the RX will be enabled.
        *  - clear rx_wake interrupt source as it triggers while normal operations.
        *  - clear wake interrupt pending state as it becomes pending in active mode.
        */

        PSOC5_UART_RX_CTRL_REG |= PSOC5_UART_RX_CTRL_SKIP_START;

        #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
            #if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
                (void) PSOC5_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
            #endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */
        #else
            #if(PSOC5_UART_RX_WAKE_PIN)
                (void) PSOC5_rx_wake_ClearInterrupt();
            #endif /* (PSOC5_UART_RX_WAKE_PIN) */
        #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

        #if(PSOC5_UART_RX_WAKEUP_IRQ)
            PSOC5_RX_WAKEUP_IRQ_ClearPending();
            PSOC5_RX_WAKEUP_IRQ_Enable();
        #endif /* (PSOC5_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: PSOC5_UartRestoreConfig
    ********************************************************************************
    *
    * Summary:
    *  Does nothing.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void PSOC5_UartRestoreConfig(void)
    {
        /* Disable RX GPIO interrupt: no more triggers in active mode */
        #if(PSOC5_UART_RX_WAKEUP_IRQ)
            PSOC5_RX_WAKEUP_IRQ_Disable();
        #endif /* (PSOC5_UART_RX_WAKEUP_IRQ) */
    }
#endif /* (PSOC5_UART_WAKE_ENABLE_CONST) */


#if(PSOC5_UART_RX_WAKEUP_IRQ)
    /*******************************************************************************
    * Function Name: PSOC5_UART_WAKEUP_ISR
    ********************************************************************************
    *
    * Summary:
    *  Handles Interrupt Service Routine for SCB UART mode GPIO wakeup event.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    CY_ISR(PSOC5_UART_WAKEUP_ISR)
    {
        /* Clear interrupt source: the event becomes multi triggerred and only disabled
        * by PSOC5_UartRestoreConfig() call.
        */
        #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
            #if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
                (void) PSOC5_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
            #endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */
        #else
            #if(PSOC5_UART_RX_WAKE_PIN)
                (void) PSOC5_rx_wake_ClearInterrupt();
            #endif /* (PSOC5_UART_RX_WAKE_PIN) */
        #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
    }
#endif /* (PSOC5_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
