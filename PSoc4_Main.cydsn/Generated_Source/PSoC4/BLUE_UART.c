/*******************************************************************************
* File Name: BLUE_UART.c
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

#include "BLUE_PVT.h"
#include "BLUE_SPI_UART_PVT.h"


#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Config Structure Initialization
    ***************************************/

    const BLUE_UART_INIT_STRUCT BLUE_configUart =
    {
        BLUE_UART_SUB_MODE,
        BLUE_UART_DIRECTION,
        BLUE_UART_DATA_BITS_NUM,
        BLUE_UART_PARITY_TYPE,
        BLUE_UART_STOP_BITS_NUM,
        BLUE_UART_OVS_FACTOR,
        BLUE_UART_IRDA_LOW_POWER,
        BLUE_UART_MEDIAN_FILTER_ENABLE,
        BLUE_UART_RETRY_ON_NACK,
        BLUE_UART_IRDA_POLARITY,
        BLUE_UART_DROP_ON_PARITY_ERR,
        BLUE_UART_DROP_ON_FRAME_ERR,
        BLUE_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        BLUE_UART_MP_MODE_ENABLE,
        BLUE_UART_MP_ACCEPT_ADDRESS,
        BLUE_UART_MP_RX_ADDRESS,
        BLUE_UART_MP_RX_ADDRESS_MASK,
        BLUE_SCB_IRQ_INTERNAL,
        BLUE_UART_INTR_RX_MASK,
        BLUE_UART_RX_TRIGGER_LEVEL,
        BLUE_UART_INTR_TX_MASK,
        BLUE_UART_TX_TRIGGER_LEVEL
    };


    /*******************************************************************************
    * Function Name: BLUE_UartInit
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
    void BLUE_UartInit(const BLUE_UART_INIT_STRUCT *config)
    {
        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due bad fucntion parameter */
        }
        else
        {
            /* Configure pins */
            BLUE_SetPins(BLUE_SCB_MODE_UART, config->mode, config->direction);

            /* Store internal configuration */
            BLUE_scbMode       = (uint8) BLUE_SCB_MODE_UART;
            BLUE_scbEnableWake = (uint8) config->enableWake;
            BLUE_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            BLUE_rxBuffer      =         config->rxBuffer;
            BLUE_rxDataBits    = (uint8) config->dataBits;
            BLUE_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            BLUE_txBuffer      =         config->txBuffer;
            BLUE_txDataBits    = (uint8) config->dataBits;
            BLUE_txBufferSize  = (uint8) config->txBufferSize;


            /* Configure UART interface */
            if(BLUE_UART_MODE_IRDA == config->mode)
            {
                /* OVS settigns: IrDA */
                BLUE_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (BLUE_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (BLUE_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settigns: UART and SmartCard */
                BLUE_CTRL_REG  = BLUE_GET_CTRL_OVS(config->oversample);
            }

            BLUE_CTRL_REG     |= BLUE_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             BLUE_CTRL_UART;

            /* Confiuure submode: UART, SmartCard or IrDA */
            BLUE_UART_CTRL_REG = BLUE_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            BLUE_UART_RX_CTRL_REG = BLUE_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        BLUE_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        BLUE_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        BLUE_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        BLUE_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(BLUE_UART_PARITY_NONE != config->parity)
            {
               BLUE_UART_RX_CTRL_REG |= BLUE_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    BLUE_UART_RX_CTRL_PARITY_ENABLED;
            }

            BLUE_RX_CTRL_REG      = BLUE_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                BLUE_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                BLUE_GET_UART_RX_CTRL_ENABLED(config->direction);

            BLUE_RX_FIFO_CTRL_REG = BLUE_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            BLUE_RX_MATCH_REG     = BLUE_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                BLUE_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            BLUE_UART_TX_CTRL_REG = BLUE_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                BLUE_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(BLUE_UART_PARITY_NONE != config->parity)
            {
               BLUE_UART_TX_CTRL_REG |= BLUE_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    BLUE_UART_TX_CTRL_PARITY_ENABLED;
            }

            BLUE_TX_CTRL_REG      = BLUE_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                BLUE_GET_UART_TX_CTRL_ENABLED(config->direction);

            BLUE_TX_FIFO_CTRL_REG = BLUE_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);


            /* Configure WAKE interrupt */
            #if(BLUE_UART_RX_WAKEUP_IRQ)
                BLUE_RX_WAKEUP_IRQ_StartEx(&BLUE_UART_WAKEUP_ISR);
                BLUE_RX_WAKEUP_IRQ_Disable();
            #endif /* (BLUE_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt with UART handler */
            BLUE_SCB_IRQ_Disable();
            BLUE_SCB_IRQ_SetVector(&BLUE_SPI_UART_ISR);
            BLUE_SCB_IRQ_SetPriority((uint8)BLUE_SCB_IRQ_INTC_PRIOR_NUMBER);

            /* Configure interrupt sources */
            BLUE_INTR_I2C_EC_MASK_REG = BLUE_NO_INTR_SOURCES;
            BLUE_INTR_SPI_EC_MASK_REG = BLUE_NO_INTR_SOURCES;
            BLUE_INTR_SLAVE_MASK_REG  = BLUE_NO_INTR_SOURCES;
            BLUE_INTR_MASTER_MASK_REG = BLUE_NO_INTR_SOURCES;
            BLUE_INTR_RX_MASK_REG     = config->rxInterruptMask;
            BLUE_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Clear RX buffer indexes */
            BLUE_rxBufferHead     = 0u;
            BLUE_rxBufferTail     = 0u;
            BLUE_rxBufferOverflow = 0u;

            /* Clrea TX buffer indexes */
            BLUE_txBufferHead = 0u;
            BLUE_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: BLUE_UartInit
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
    void BLUE_UartInit(void)
    {
        /* Configure UART interface */
        BLUE_CTRL_REG = BLUE_UART_DEFAULT_CTRL;

        /* Confiuure submode: UART, SmartCard or IrDA */
        BLUE_UART_CTRL_REG = BLUE_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        BLUE_UART_RX_CTRL_REG = BLUE_UART_DEFAULT_UART_RX_CTRL;
        BLUE_RX_CTRL_REG      = BLUE_UART_DEFAULT_RX_CTRL;
        BLUE_RX_FIFO_CTRL_REG = BLUE_UART_DEFAULT_RX_FIFO_CTRL;
        BLUE_RX_MATCH_REG     = BLUE_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        BLUE_UART_TX_CTRL_REG = BLUE_UART_DEFAULT_UART_TX_CTRL;
        BLUE_TX_CTRL_REG      = BLUE_UART_DEFAULT_TX_CTRL;
        BLUE_TX_FIFO_CTRL_REG = BLUE_UART_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with UART handler */
        #if(BLUE_SCB_IRQ_INTERNAL)
            BLUE_SCB_IRQ_Disable();
            BLUE_SCB_IRQ_SetVector(&BLUE_SPI_UART_ISR);
            BLUE_SCB_IRQ_SetPriority((uint8)BLUE_SCB_IRQ_INTC_PRIOR_NUMBER);
        #endif /* (BLUE_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
        #if(BLUE_UART_RX_WAKEUP_IRQ)
            BLUE_RX_WAKEUP_IRQ_StartEx(&BLUE_UART_WAKEUP_ISR);
            BLUE_RX_WAKEUP_IRQ_Disable();
        #endif /* (BLUE_UART_RX_WAKEUP_IRQ) */
        
        /* Configure interrupt sources */
        BLUE_INTR_I2C_EC_MASK_REG = BLUE_UART_DEFAULT_INTR_I2C_EC_MASK;
        BLUE_INTR_SPI_EC_MASK_REG = BLUE_UART_DEFAULT_INTR_SPI_EC_MASK;
        BLUE_INTR_SLAVE_MASK_REG  = BLUE_UART_DEFAULT_INTR_SLAVE_MASK;
        BLUE_INTR_MASTER_MASK_REG = BLUE_UART_DEFAULT_INTR_MASTER_MASK;
        BLUE_INTR_RX_MASK_REG     = BLUE_UART_DEFAULT_INTR_RX_MASK;
        BLUE_INTR_TX_MASK_REG     = BLUE_UART_DEFAULT_INTR_TX_MASK;

        #if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
            BLUE_rxBufferHead     = 0u;
            BLUE_rxBufferTail     = 0u;
            BLUE_rxBufferOverflow = 0u;
        #endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
            BLUE_txBufferHead = 0u;
            BLUE_txBufferTail = 0u;
        #endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */
    }

#endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: BLUE_UartSetRxAddress
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
void BLUE_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = BLUE_RX_MATCH_REG;

    matchReg &= ((uint32) ~BLUE_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & BLUE_RX_MATCH_ADDR_MASK)); /* Set address  */

    BLUE_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: BLUE_UartSetRxAddressMask
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
void BLUE_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = BLUE_RX_MATCH_REG;

    matchReg &= ((uint32) ~BLUE_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << BLUE_RX_MATCH_MASK_POS));

    BLUE_RX_MATCH_REG = matchReg;
}


#if(BLUE_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: BLUE_UartGetChar
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
    uint32 BLUE_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Read data only if there is data to read */
        if(0u != BLUE_SpiUartGetRxBufferSize())
        {
            rxData = BLUE_SpiUartReadRxData();
        }

        if(BLUE_CHECK_INTR_RX(BLUE_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occured: return zero */
            BLUE_ClearRxInterruptSource(BLUE_INTR_RX_ERR);
        }

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: BLUE_UartGetByte
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
    uint32 BLUE_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;
        uint32 intSourceMask;

        intSourceMask = BLUE_SpiUartDisableIntRx();

        if(0u != BLUE_SpiUartGetRxBufferSize())
        {
             /*
             * Enable interrupt to receive more bytes: at least one byte is in
             * buffer.
             */
            BLUE_SpiUartEnableIntRx(intSourceMask);

            /* Get received byte */
            rxData = BLUE_SpiUartReadRxData();
        }
        else
        {
            /*
            * Read byte directly from RX FIFO: the underflow is raised in case
            * of empty. In other case the first received byte will be read.
            */
            rxData = BLUE_RX_FIFO_RD_REG;

            /*
            * Enable interrupt to receive more bytes.
            * The RX_NOT_EMPTY interrupt is cleared by interrupt routine in case
            * byte was received and read above.
            */
            BLUE_SpiUartEnableIntRx(intSourceMask);
        }

        /* Get and clear RX error mask */
        tmpStatus = (BLUE_GetRxInterruptSource() & BLUE_INTR_RX_ERR);
        BLUE_ClearRxInterruptSource(BLUE_INTR_RX_ERR);

        /*
        * Put together data and error status:
        * MP mode and accept address: the 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return(rxData);
    }

#endif /* (BLUE_UART_RX_DIRECTION) */


#if(BLUE_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: BLUE_UartPutString
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
    void BLUE_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data will be sent */
        while(string[bufIndex] != ((char8) 0))
        {
            BLUE_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_UartPutCRLF
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
    void BLUE_UartPutCRLF(uint32 txDataByte)
    {
        BLUE_UartPutChar(txDataByte);  /* Blocks the control flow until all data will be sent */
        BLUE_UartPutChar(0x0Du);       /* Blocks the control flow until all data will be sent */
        BLUE_UartPutChar(0x0Au);       /* Blocks the control flow until all data will be sent */
    }
#endif /* (BLUE_UART_TX_DIRECTION) */


#if(BLUE_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: BLUE_UartSaveConfig
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
    void BLUE_UartSaveConfig(void)
    {
        /* Clear interrupt activy:
        *  - set skip start and disable RX. On GPIO wakeup the RX will be enabled.
        *  - clear rx_wake interrupt source as it triggers while normal operations.
        *  - clear wake interrupt pending state as it becomes pending in active mode.
        */

        BLUE_UART_RX_CTRL_REG |= BLUE_UART_RX_CTRL_SKIP_START;

        #if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
            #if(BLUE_MOSI_SCL_RX_WAKE_PIN)
                (void) BLUE_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
            #endif /* (BLUE_MOSI_SCL_RX_WAKE_PIN) */
        #else
            #if(BLUE_UART_RX_WAKE_PIN)
                (void) BLUE_rx_wake_ClearInterrupt();
            #endif /* (BLUE_UART_RX_WAKE_PIN) */
        #endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */

        #if(BLUE_UART_RX_WAKEUP_IRQ)
            BLUE_RX_WAKEUP_IRQ_ClearPending();
            BLUE_RX_WAKEUP_IRQ_Enable();
        #endif /* (BLUE_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: BLUE_UartRestoreConfig
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
    void BLUE_UartRestoreConfig(void)
    {
        /* Disable RX GPIO interrupt: no more triggers in active mode */
        #if(BLUE_UART_RX_WAKEUP_IRQ)
            BLUE_RX_WAKEUP_IRQ_Disable();
        #endif /* (BLUE_UART_RX_WAKEUP_IRQ) */
    }
#endif /* (BLUE_UART_WAKE_ENABLE_CONST) */


#if(BLUE_UART_RX_WAKEUP_IRQ)
    /*******************************************************************************
    * Function Name: BLUE_UART_WAKEUP_ISR
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
    CY_ISR(BLUE_UART_WAKEUP_ISR)
    {
        /* Clear interrupt source: the event becomes multi triggerred and only disabled
        * by BLUE_UartRestoreConfig() call.
        */
        #if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
            #if(BLUE_MOSI_SCL_RX_WAKE_PIN)
                (void) BLUE_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
            #endif /* (BLUE_MOSI_SCL_RX_WAKE_PIN) */
        #else
            #if(BLUE_UART_RX_WAKE_PIN)
                (void) BLUE_rx_wake_ClearInterrupt();
            #endif /* (BLUE_UART_RX_WAKE_PIN) */
        #endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */
    }
#endif /* (BLUE_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
