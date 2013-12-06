/*******************************************************************************
* File Name: GPS_UART.c
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

#include "GPS_PVT.h"
#include "GPS_SPI_UART_PVT.h"


#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Config Structure Initialization
    ***************************************/

    const GPS_UART_INIT_STRUCT GPS_configUart =
    {
        GPS_UART_SUB_MODE,
        GPS_UART_DIRECTION,
        GPS_UART_DATA_BITS_NUM,
        GPS_UART_PARITY_TYPE,
        GPS_UART_STOP_BITS_NUM,
        GPS_UART_OVS_FACTOR,
        GPS_UART_IRDA_LOW_POWER,
        GPS_UART_MEDIAN_FILTER_ENABLE,
        GPS_UART_RETRY_ON_NACK,
        GPS_UART_IRDA_POLARITY,
        GPS_UART_DROP_ON_PARITY_ERR,
        GPS_UART_DROP_ON_FRAME_ERR,
        GPS_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        GPS_UART_MP_MODE_ENABLE,
        GPS_UART_MP_ACCEPT_ADDRESS,
        GPS_UART_MP_RX_ADDRESS,
        GPS_UART_MP_RX_ADDRESS_MASK,
        GPS_SCB_IRQ_INTERNAL,
        GPS_UART_INTR_RX_MASK,
        GPS_UART_RX_TRIGGER_LEVEL,
        GPS_UART_INTR_TX_MASK,
        GPS_UART_TX_TRIGGER_LEVEL
    };


    /*******************************************************************************
    * Function Name: GPS_UartInit
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
    void GPS_UartInit(const GPS_UART_INIT_STRUCT *config)
    {
        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due bad fucntion parameter */
        }
        else
        {
            /* Configure pins */
            GPS_SetPins(GPS_SCB_MODE_UART, config->mode, config->direction);

            /* Store internal configuration */
            GPS_scbMode       = (uint8) GPS_SCB_MODE_UART;
            GPS_scbEnableWake = (uint8) config->enableWake;
            GPS_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            GPS_rxBuffer      =         config->rxBuffer;
            GPS_rxDataBits    = (uint8) config->dataBits;
            GPS_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            GPS_txBuffer      =         config->txBuffer;
            GPS_txDataBits    = (uint8) config->dataBits;
            GPS_txBufferSize  = (uint8) config->txBufferSize;


            /* Configure UART interface */
            if(GPS_UART_MODE_IRDA == config->mode)
            {
                /* OVS settigns: IrDA */
                GPS_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (GPS_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (GPS_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settigns: UART and SmartCard */
                GPS_CTRL_REG  = GPS_GET_CTRL_OVS(config->oversample);
            }

            GPS_CTRL_REG     |= GPS_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             GPS_CTRL_UART;

            /* Confiuure submode: UART, SmartCard or IrDA */
            GPS_UART_CTRL_REG = GPS_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            GPS_UART_RX_CTRL_REG = GPS_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        GPS_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        GPS_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        GPS_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        GPS_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(GPS_UART_PARITY_NONE != config->parity)
            {
               GPS_UART_RX_CTRL_REG |= GPS_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    GPS_UART_RX_CTRL_PARITY_ENABLED;
            }

            GPS_RX_CTRL_REG      = GPS_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                GPS_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                GPS_GET_UART_RX_CTRL_ENABLED(config->direction);

            GPS_RX_FIFO_CTRL_REG = GPS_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            GPS_RX_MATCH_REG     = GPS_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                GPS_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            GPS_UART_TX_CTRL_REG = GPS_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                GPS_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(GPS_UART_PARITY_NONE != config->parity)
            {
               GPS_UART_TX_CTRL_REG |= GPS_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    GPS_UART_TX_CTRL_PARITY_ENABLED;
            }

            GPS_TX_CTRL_REG      = GPS_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                GPS_GET_UART_TX_CTRL_ENABLED(config->direction);

            GPS_TX_FIFO_CTRL_REG = GPS_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);


            /* Configure WAKE interrupt */
            #if(GPS_UART_RX_WAKEUP_IRQ)
                GPS_RX_WAKEUP_IRQ_StartEx(&GPS_UART_WAKEUP_ISR);
                GPS_RX_WAKEUP_IRQ_Disable();
            #endif /* (GPS_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt with UART handler */
            GPS_SCB_IRQ_Disable();
            GPS_SCB_IRQ_SetVector(&GPS_SPI_UART_ISR);
            GPS_SCB_IRQ_SetPriority((uint8)GPS_SCB_IRQ_INTC_PRIOR_NUMBER);

            /* Configure interrupt sources */
            GPS_INTR_I2C_EC_MASK_REG = GPS_NO_INTR_SOURCES;
            GPS_INTR_SPI_EC_MASK_REG = GPS_NO_INTR_SOURCES;
            GPS_INTR_SLAVE_MASK_REG  = GPS_NO_INTR_SOURCES;
            GPS_INTR_MASTER_MASK_REG = GPS_NO_INTR_SOURCES;
            GPS_INTR_RX_MASK_REG     = config->rxInterruptMask;
            GPS_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Clear RX buffer indexes */
            GPS_rxBufferHead     = 0u;
            GPS_rxBufferTail     = 0u;
            GPS_rxBufferOverflow = 0u;

            /* Clrea TX buffer indexes */
            GPS_txBufferHead = 0u;
            GPS_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: GPS_UartInit
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
    void GPS_UartInit(void)
    {
        /* Configure UART interface */
        GPS_CTRL_REG = GPS_UART_DEFAULT_CTRL;

        /* Confiuure submode: UART, SmartCard or IrDA */
        GPS_UART_CTRL_REG = GPS_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        GPS_UART_RX_CTRL_REG = GPS_UART_DEFAULT_UART_RX_CTRL;
        GPS_RX_CTRL_REG      = GPS_UART_DEFAULT_RX_CTRL;
        GPS_RX_FIFO_CTRL_REG = GPS_UART_DEFAULT_RX_FIFO_CTRL;
        GPS_RX_MATCH_REG     = GPS_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        GPS_UART_TX_CTRL_REG = GPS_UART_DEFAULT_UART_TX_CTRL;
        GPS_TX_CTRL_REG      = GPS_UART_DEFAULT_TX_CTRL;
        GPS_TX_FIFO_CTRL_REG = GPS_UART_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with UART handler */
        #if(GPS_SCB_IRQ_INTERNAL)
            GPS_SCB_IRQ_Disable();
            GPS_SCB_IRQ_SetVector(&GPS_SPI_UART_ISR);
            GPS_SCB_IRQ_SetPriority((uint8)GPS_SCB_IRQ_INTC_PRIOR_NUMBER);
        #endif /* (GPS_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
        #if(GPS_UART_RX_WAKEUP_IRQ)
            GPS_RX_WAKEUP_IRQ_StartEx(&GPS_UART_WAKEUP_ISR);
            GPS_RX_WAKEUP_IRQ_Disable();
        #endif /* (GPS_UART_RX_WAKEUP_IRQ) */
        
        /* Configure interrupt sources */
        GPS_INTR_I2C_EC_MASK_REG = GPS_UART_DEFAULT_INTR_I2C_EC_MASK;
        GPS_INTR_SPI_EC_MASK_REG = GPS_UART_DEFAULT_INTR_SPI_EC_MASK;
        GPS_INTR_SLAVE_MASK_REG  = GPS_UART_DEFAULT_INTR_SLAVE_MASK;
        GPS_INTR_MASTER_MASK_REG = GPS_UART_DEFAULT_INTR_MASTER_MASK;
        GPS_INTR_RX_MASK_REG     = GPS_UART_DEFAULT_INTR_RX_MASK;
        GPS_INTR_TX_MASK_REG     = GPS_UART_DEFAULT_INTR_TX_MASK;

        #if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
            GPS_rxBufferHead     = 0u;
            GPS_rxBufferTail     = 0u;
            GPS_rxBufferOverflow = 0u;
        #endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
            GPS_txBufferHead = 0u;
            GPS_txBufferTail = 0u;
        #endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */
    }

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: GPS_UartSetRxAddress
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
void GPS_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = GPS_RX_MATCH_REG;

    matchReg &= ((uint32) ~GPS_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & GPS_RX_MATCH_ADDR_MASK)); /* Set address  */

    GPS_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: GPS_UartSetRxAddressMask
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
void GPS_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = GPS_RX_MATCH_REG;

    matchReg &= ((uint32) ~GPS_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << GPS_RX_MATCH_MASK_POS));

    GPS_RX_MATCH_REG = matchReg;
}


#if(GPS_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: GPS_UartGetChar
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
    uint32 GPS_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Read data only if there is data to read */
        if(0u != GPS_SpiUartGetRxBufferSize())
        {
            rxData = GPS_SpiUartReadRxData();
        }

        if(GPS_CHECK_INTR_RX(GPS_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occured: return zero */
            GPS_ClearRxInterruptSource(GPS_INTR_RX_ERR);
        }

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: GPS_UartGetByte
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
    uint32 GPS_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;
        uint32 intSourceMask;

        intSourceMask = GPS_SpiUartDisableIntRx();

        if(0u != GPS_SpiUartGetRxBufferSize())
        {
             /*
             * Enable interrupt to receive more bytes: at least one byte is in
             * buffer.
             */
            GPS_SpiUartEnableIntRx(intSourceMask);

            /* Get received byte */
            rxData = GPS_SpiUartReadRxData();
        }
        else
        {
            /*
            * Read byte directly from RX FIFO: the underflow is raised in case
            * of empty. In other case the first received byte will be read.
            */
            rxData = GPS_RX_FIFO_RD_REG;

            /*
            * Enable interrupt to receive more bytes.
            * The RX_NOT_EMPTY interrupt is cleared by interrupt routine in case
            * byte was received and read above.
            */
            GPS_SpiUartEnableIntRx(intSourceMask);
        }

        /* Get and clear RX error mask */
        tmpStatus = (GPS_GetRxInterruptSource() & GPS_INTR_RX_ERR);
        GPS_ClearRxInterruptSource(GPS_INTR_RX_ERR);

        /*
        * Put together data and error status:
        * MP mode and accept address: the 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return(rxData);
    }

#endif /* (GPS_UART_RX_DIRECTION) */


#if(GPS_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: GPS_UartPutString
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
    void GPS_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data will be sent */
        while(string[bufIndex] != ((char8) 0))
        {
            GPS_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: GPS_UartPutCRLF
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
    void GPS_UartPutCRLF(uint32 txDataByte)
    {
        GPS_UartPutChar(txDataByte);  /* Blocks the control flow until all data will be sent */
        GPS_UartPutChar(0x0Du);       /* Blocks the control flow until all data will be sent */
        GPS_UartPutChar(0x0Au);       /* Blocks the control flow until all data will be sent */
    }
#endif /* (GPS_UART_TX_DIRECTION) */


#if(GPS_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: GPS_UartSaveConfig
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
    void GPS_UartSaveConfig(void)
    {
        /* Clear interrupt activy:
        *  - set skip start and disable RX. On GPIO wakeup the RX will be enabled.
        *  - clear rx_wake interrupt source as it triggers while normal operations.
        *  - clear wake interrupt pending state as it becomes pending in active mode.
        */

        GPS_UART_RX_CTRL_REG |= GPS_UART_RX_CTRL_SKIP_START;

        #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
            #if(GPS_MOSI_SCL_RX_WAKE_PIN)
                (void) GPS_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
            #endif /* (GPS_MOSI_SCL_RX_WAKE_PIN) */
        #else
            #if(GPS_UART_RX_WAKE_PIN)
                (void) GPS_rx_wake_ClearInterrupt();
            #endif /* (GPS_UART_RX_WAKE_PIN) */
        #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

        #if(GPS_UART_RX_WAKEUP_IRQ)
            GPS_RX_WAKEUP_IRQ_ClearPending();
            GPS_RX_WAKEUP_IRQ_Enable();
        #endif /* (GPS_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: GPS_UartRestoreConfig
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
    void GPS_UartRestoreConfig(void)
    {
        /* Disable RX GPIO interrupt: no more triggers in active mode */
        #if(GPS_UART_RX_WAKEUP_IRQ)
            GPS_RX_WAKEUP_IRQ_Disable();
        #endif /* (GPS_UART_RX_WAKEUP_IRQ) */
    }
#endif /* (GPS_UART_WAKE_ENABLE_CONST) */


#if(GPS_UART_RX_WAKEUP_IRQ)
    /*******************************************************************************
    * Function Name: GPS_UART_WAKEUP_ISR
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
    CY_ISR(GPS_UART_WAKEUP_ISR)
    {
        /* Clear interrupt source: the event becomes multi triggerred and only disabled
        * by GPS_UartRestoreConfig() call.
        */
        #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
            #if(GPS_MOSI_SCL_RX_WAKE_PIN)
                (void) GPS_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
            #endif /* (GPS_MOSI_SCL_RX_WAKE_PIN) */
        #else
            #if(GPS_UART_RX_WAKE_PIN)
                (void) GPS_rx_wake_ClearInterrupt();
            #endif /* (GPS_UART_RX_WAKE_PIN) */
        #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
    }
#endif /* (GPS_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
