/*******************************************************************************
* File Name: GPS.c
* Version 2.30
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "GPS.h"
#include "CyLib.h"
#if(GPS_INTERNAL_CLOCK_USED)
    #include "GPS_IntClock.h"
#endif /* End GPS_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 GPS_initVar = 0u;
#if( GPS_TX_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))
    volatile uint8 GPS_txBuffer[GPS_TXBUFFERSIZE];
    volatile uint8 GPS_txBufferRead = 0u;
    uint8 GPS_txBufferWrite = 0u;
#endif /* End GPS_TX_ENABLED */
#if( ( GPS_RX_ENABLED || GPS_HD_ENABLED ) && \
     (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH) )
    volatile uint8 GPS_rxBuffer[GPS_RXBUFFERSIZE];
    volatile uint8 GPS_rxBufferRead = 0u;
    volatile uint8 GPS_rxBufferWrite = 0u;
    volatile uint8 GPS_rxBufferLoopDetect = 0u;
    volatile uint8 GPS_rxBufferOverflow = 0u;
    #if (GPS_RXHW_ADDRESS_ENABLED)
        volatile uint8 GPS_rxAddressMode = GPS_RXADDRESSMODE;
        volatile uint8 GPS_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End GPS_RX_ENABLED */


/*******************************************************************************
* Function Name: GPS_Start
********************************************************************************
*
* Summary:
*  Initialize and Enable the UART component.
*  Enable the clock input to enable operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The GPS_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the GPS_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void GPS_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(GPS_initVar == 0u)
    {
        GPS_Init();
        GPS_initVar = 1u;
    }
    GPS_Enable();
}


/*******************************************************************************
* Function Name: GPS_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  GPS_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void GPS_Init(void) 
{
    #if(GPS_RX_ENABLED || GPS_HD_ENABLED)

        #if(GPS_RX_INTERRUPT_ENABLED && (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(GPS_RX_VECT_NUM, &GPS_RXISR);
            CyIntSetPriority(GPS_RX_VECT_NUM, GPS_RX_PRIOR_NUM);
        #endif /* End GPS_RX_INTERRUPT_ENABLED */

        #if (GPS_RXHW_ADDRESS_ENABLED)
            GPS_SetRxAddressMode(GPS_RXAddressMode);
            GPS_SetRxAddress1(GPS_RXHWADDRESS1);
            GPS_SetRxAddress2(GPS_RXHWADDRESS2);
        #endif /* End GPS_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        GPS_RXBITCTR_PERIOD_REG = GPS_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        GPS_RXSTATUS_MASK_REG  = GPS_INIT_RX_INTERRUPTS_MASK;
    #endif /* End GPS_RX_ENABLED || GPS_HD_ENABLED*/

    #if(GPS_TX_ENABLED)
        #if(GPS_TX_INTERRUPT_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(GPS_TX_VECT_NUM, &GPS_TXISR);
            CyIntSetPriority(GPS_TX_VECT_NUM, GPS_TX_PRIOR_NUM);
        #endif /* End GPS_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(GPS_TXCLKGEN_DP)
            GPS_TXBITCLKGEN_CTR_REG = GPS_BIT_CENTER;
            GPS_TXBITCLKTX_COMPLETE_REG = (GPS_NUMBER_OF_DATA_BITS +
                        GPS_NUMBER_OF_START_BIT) * GPS_OVER_SAMPLE_COUNT;
        #else
            GPS_TXBITCTR_PERIOD_REG = ((GPS_NUMBER_OF_DATA_BITS +
                        GPS_NUMBER_OF_START_BIT) * GPS_OVER_SAMPLE_8) - 1u;
        #endif /* End GPS_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(GPS_TX_INTERRUPT_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))
            GPS_TXSTATUS_MASK_REG = GPS_TX_STS_FIFO_EMPTY;
        #else
            GPS_TXSTATUS_MASK_REG = GPS_INIT_TX_INTERRUPTS_MASK;
        #endif /*End GPS_TX_INTERRUPT_ENABLED*/

    #endif /* End GPS_TX_ENABLED */

    #if(GPS_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        GPS_WriteControlRegister( \
            (GPS_ReadControlRegister() & (uint8)~GPS_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(GPS_PARITY_TYPE << GPS_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End GPS_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: GPS_Enable
********************************************************************************
*
* Summary:
*  Enables the UART block operation
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  GPS_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void GPS_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(GPS_RX_ENABLED || GPS_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        GPS_RXBITCTR_CONTROL_REG |= GPS_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        GPS_RXSTATUS_ACTL_REG  |= GPS_INT_ENABLE;
        #if(GPS_RX_INTERRUPT_ENABLED && (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH))
            CyIntEnable(GPS_RX_VECT_NUM);
            #if (GPS_RXHW_ADDRESS_ENABLED)
                GPS_rxAddressDetected = 0u;
            #endif /* End GPS_RXHW_ADDRESS_ENABLED */
        #endif /* End GPS_RX_INTERRUPT_ENABLED */
    #endif /* End GPS_RX_ENABLED || GPS_HD_ENABLED*/

    #if(GPS_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!GPS_TXCLKGEN_DP)
            GPS_TXBITCTR_CONTROL_REG |= GPS_CNTR_ENABLE;
        #endif /* End GPS_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        GPS_TXSTATUS_ACTL_REG |= GPS_INT_ENABLE;
        #if(GPS_TX_INTERRUPT_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))
            CyIntEnable(GPS_TX_VECT_NUM);
        #endif /* End GPS_TX_INTERRUPT_ENABLED*/
     #endif /* End GPS_TX_ENABLED */

    #if(GPS_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        GPS_IntClock_Start();
    #endif /* End GPS_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: GPS_Stop
********************************************************************************
*
* Summary:
*  Disable the UART component
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void GPS_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(GPS_RX_ENABLED || GPS_HD_ENABLED)
        GPS_RXBITCTR_CONTROL_REG &= (uint8)~GPS_CNTR_ENABLE;
    #endif /* End GPS_RX_ENABLED */

    #if(GPS_TX_ENABLED)
        #if(!GPS_TXCLKGEN_DP)
            GPS_TXBITCTR_CONTROL_REG &= (uint8)~GPS_CNTR_ENABLE;
        #endif /* End GPS_TXCLKGEN_DP */
    #endif /* GPS_TX_ENABLED */

    #if(GPS_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        GPS_IntClock_Stop();
    #endif /* End GPS_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(GPS_RX_ENABLED || GPS_HD_ENABLED)
        GPS_RXSTATUS_ACTL_REG  &= (uint8)~GPS_INT_ENABLE;
        #if(GPS_RX_INTERRUPT_ENABLED && (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH))
            GPS_DisableRxInt();
        #endif /* End GPS_RX_INTERRUPT_ENABLED */
    #endif /* End GPS_RX_ENABLED */

    #if(GPS_TX_ENABLED)
        GPS_TXSTATUS_ACTL_REG &= (uint8)~GPS_INT_ENABLE;
        #if(GPS_TX_INTERRUPT_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))
            GPS_DisableTxInt();
        #endif /* End GPS_TX_INTERRUPT_ENABLED */
    #endif /* End GPS_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: GPS_ReadControlRegister
********************************************************************************
*
* Summary:
*  Read the current state of the control register
*
* Parameters:
*  None.
*
* Return:
*  Current state of the control register.
*
*******************************************************************************/
uint8 GPS_ReadControlRegister(void) 
{
    #if( GPS_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(GPS_CONTROL_REG);
    #endif /* End GPS_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: GPS_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  GPS_WriteControlRegister(uint8 control) 
{
    #if( GPS_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       GPS_CONTROL_REG = control;
    #endif /* End GPS_CONTROL_REG_REMOVED */
}


#if(GPS_RX_ENABLED || GPS_HD_ENABLED)

    #if(GPS_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: GPS_EnableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void GPS_EnableRxInt(void) 
        {
            CyIntEnable(GPS_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: GPS_DisableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void GPS_DisableRxInt(void) 
        {
            CyIntDisable(GPS_RX_VECT_NUM);
        }

    #endif /* GPS_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: GPS_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  IntSrc:  An or'd combination of the desired status bit masks (defined in
    *           the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void GPS_SetRxInterruptMode(uint8 intSrc) 
    {
        GPS_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: GPS_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns data in RX Data register without checking status register to
    *  determine if data is valid
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  GPS_rxBuffer - RAM buffer pointer for save received data.
    *  GPS_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  GPS_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  GPS_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 GPS_ReadRxData(void) 
    {
        uint8 rxData;

        #if(GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_DisableRxInt();
            #endif /* GPS_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = GPS_rxBufferRead;
            loc_rxBufferWrite = GPS_rxBufferWrite;

            if( (GPS_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = GPS_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= GPS_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                GPS_rxBufferRead = loc_rxBufferRead;

                if(GPS_rxBufferLoopDetect != 0u )
                {
                    GPS_rxBufferLoopDetect = 0u;
                    #if( (GPS_RX_INTERRUPT_ENABLED) && (GPS_FLOW_CONTROL != 0u) && \
                         (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( GPS_HD_ENABLED )
                            if((GPS_CONTROL_REG & GPS_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                GPS_RXSTATUS_MASK_REG  |= GPS_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            GPS_RXSTATUS_MASK_REG  |= GPS_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end GPS_HD_ENABLED */
                    #endif /* GPS_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = GPS_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_EnableRxInt();
            #endif /* End GPS_RX_INTERRUPT_ENABLED */

        #else /* GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = GPS_RXDATA_REG;

        #endif /* GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: GPS_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the current state of the status register
    *  And detect software buffer overflow.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Global Variables:
    *  GPS_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   GPS_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   GPS_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 GPS_ReadRxStatus(void) 
    {
        uint8 status;

        status = GPS_RXSTATUS_REG & GPS_RX_HW_MASK;

        #if(GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH)
            if( GPS_rxBufferOverflow != 0u )
            {
                status |= GPS_RX_STS_SOFT_BUFF_OVER;
                GPS_rxBufferOverflow = 0u;
            }
        #endif /* GPS_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: GPS_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, if data is not available or an error
    *  condition exists, zero is returned; otherwise, character is read and
    *  returned.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  GPS_rxBuffer - RAM buffer pointer for save received data.
    *  GPS_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  GPS_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  GPS_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 GPS_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_DisableRxInt();
            #endif /* GPS_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = GPS_rxBufferRead;
            loc_rxBufferWrite = GPS_rxBufferWrite;

            if( (GPS_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = GPS_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= GPS_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                GPS_rxBufferRead = loc_rxBufferRead;

                if(GPS_rxBufferLoopDetect > 0u )
                {
                    GPS_rxBufferLoopDetect = 0u;
                    #if( (GPS_RX_INTERRUPT_ENABLED) && (GPS_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( GPS_HD_ENABLED )
                            if((GPS_CONTROL_REG & GPS_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                GPS_RXSTATUS_MASK_REG  |= GPS_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            GPS_RXSTATUS_MASK_REG  |= GPS_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end GPS_HD_ENABLED */
                    #endif /* GPS_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = GPS_RXSTATUS_REG;
                if((rxStatus & GPS_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = GPS_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (GPS_RX_STS_BREAK | GPS_RX_STS_PAR_ERROR |
                                   GPS_RX_STS_STOP_ERROR | GPS_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_EnableRxInt();
            #endif /* GPS_RX_INTERRUPT_ENABLED */

        #else /* GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH */

            rxStatus =GPS_RXSTATUS_REG;
            if((rxStatus & GPS_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = GPS_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (GPS_RX_STS_BREAK | GPS_RX_STS_PAR_ERROR |
                               GPS_RX_STS_STOP_ERROR | GPS_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: GPS_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Grab the next available byte of data from the recieve FIFO
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains Status Register and LSB contains UART RX data
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 GPS_GetByte(void) 
    {
        return ( ((uint16)GPS_ReadRxStatus() << 8u) | GPS_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: GPS_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of bytes left in the RX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Integer count of the number of bytes left
    *  in the RX buffer
    *
    * Global Variables:
    *  GPS_rxBufferWrite - used to calculate left bytes.
    *  GPS_rxBufferRead - used to calculate left bytes.
    *  GPS_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 GPS_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_DisableRxInt();
            #endif /* GPS_RX_INTERRUPT_ENABLED */

            if(GPS_rxBufferRead == GPS_rxBufferWrite)
            {
                if(GPS_rxBufferLoopDetect > 0u)
                {
                    size = GPS_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(GPS_rxBufferRead < GPS_rxBufferWrite)
            {
                size = (GPS_rxBufferWrite - GPS_rxBufferRead);
            }
            else
            {
                size = (GPS_RXBUFFERSIZE - GPS_rxBufferRead) + GPS_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_EnableRxInt();
            #endif /* End GPS_RX_INTERRUPT_ENABLED */

        #else /* GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((GPS_RXSTATUS_REG & GPS_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: GPS_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the RX RAM buffer by setting the read and write pointers both to zero.
    *  Clears hardware RX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_rxBufferWrite - cleared to zero.
    *  GPS_rxBufferRead - cleared to zero.
    *  GPS_rxBufferLoopDetect - cleared to zero.
    *  GPS_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *******************************************************************************/
    void GPS_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        GPS_RXDATA_AUX_CTL_REG |=  GPS_RX_FIFO_CLR;
        GPS_RXDATA_AUX_CTL_REG &= (uint8)~GPS_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_DisableRxInt();
            #endif /* End GPS_RX_INTERRUPT_ENABLED */

            GPS_rxBufferRead = 0u;
            GPS_rxBufferWrite = 0u;
            GPS_rxBufferLoopDetect = 0u;
            GPS_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(GPS_RX_INTERRUPT_ENABLED)
                GPS_EnableRxInt();
            #endif /* End GPS_RX_INTERRUPT_ENABLED */
        #endif /* End GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: GPS_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  GPS__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  GPS__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  GPS__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  GPS__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  GPS__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  GPS_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void GPS_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(GPS_RXHW_ADDRESS_ENABLED)
            #if(GPS_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* GPS_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = GPS_CONTROL_REG & (uint8)~GPS_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << GPS_CTRL_RXADDR_MODE0_SHIFT);
                GPS_CONTROL_REG = tmpCtrl;
                #if(GPS_RX_INTERRUPT_ENABLED && \
                   (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH) )
                    GPS_rxAddressMode = addressMode;
                    GPS_rxAddressDetected = 0u;
                #endif /* End GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH*/
            #endif /* End GPS_CONTROL_REG_REMOVED */
        #else /* GPS_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End GPS_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: GPS_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Set the first hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void GPS_SetRxAddress1(uint8 address) 

    {
        GPS_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: GPS_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Set the second hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void GPS_SetRxAddress2(uint8 address) 
    {
        GPS_RXADDRESS2_REG = address;
    }

#endif  /* GPS_RX_ENABLED || GPS_HD_ENABLED*/


#if( (GPS_TX_ENABLED) || (GPS_HD_ENABLED) )

    #if(GPS_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: GPS_EnableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void GPS_EnableTxInt(void) 
        {
            CyIntEnable(GPS_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: GPS_DisableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void GPS_DisableTxInt(void) 
        {
            CyIntDisable(GPS_TX_VECT_NUM);
        }

    #endif /* GPS_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: GPS_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  intSrc: An or'd combination of the desired status bit masks (defined in
    *          the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void GPS_SetTxInterruptMode(uint8 intSrc) 
    {
        GPS_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: GPS_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Write a byte of data to the Transmit FIFO or TX buffer to be sent when the
    *  bus is available. WriteTxData sends a byte without checking for buffer room
    *  or status. It is up to the user to separately check status.
    *
    * Parameters:
    *  TXDataByte: byte of data to place in the transmit FIFO
    *
    * Return:
    * void
    *
    * Global Variables:
    *  GPS_txBuffer - RAM buffer pointer for save data for transmission
    *  GPS_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  GPS_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  GPS_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void GPS_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(GPS_initVar != 0u)
        {
            #if(GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(GPS_TX_INTERRUPT_ENABLED)
                    GPS_DisableTxInt();
                #endif /* End GPS_TX_INTERRUPT_ENABLED */

                if( (GPS_txBufferRead == GPS_txBufferWrite) &&
                    ((GPS_TXSTATUS_REG & GPS_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    GPS_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(GPS_txBufferWrite >= GPS_TXBUFFERSIZE)
                    {
                        GPS_txBufferWrite = 0u;
                    }

                    GPS_txBuffer[GPS_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    GPS_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(GPS_TX_INTERRUPT_ENABLED)
                    GPS_EnableTxInt();
                #endif /* End GPS_TX_INTERRUPT_ENABLED */

            #else /* GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                GPS_TXDATA_REG = txDataByte;

            #endif /* End GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: GPS_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the status register for the component
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the status register which is clear on read. It is up to
    *  the user to handle all bits in this return value accordingly, even if the bit
    *  was not enabled as an interrupt source the event happened and must be handled
    *  accordingly.
    *
    *******************************************************************************/
    uint8 GPS_ReadTxStatus(void) 
    {
        return(GPS_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: GPS_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Wait to send byte until TX register or buffer has room.
    *
    * Parameters:
    *  txDataByte: The 8-bit data value to send across the UART.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_txBuffer - RAM buffer pointer for save data for transmission
    *  GPS_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  GPS_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  GPS_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void GPS_PutChar(uint8 txDataByte) 
    {
            #if(GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((GPS_TXBUFFERSIZE > GPS_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(GPS_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = GPS_txBufferWrite;
                    loc_txBufferRead = GPS_txBufferRead;
                    #if ((GPS_TXBUFFERSIZE > GPS_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(GPS_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(GPS_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((GPS_TXSTATUS_REG & GPS_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    GPS_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= GPS_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    GPS_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((GPS_TXBUFFERSIZE > GPS_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(GPS_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    GPS_txBufferWrite = loc_txBufferWrite;
                    #if ((GPS_TXBUFFERSIZE > GPS_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(GPS_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH */

                while((GPS_TXSTATUS_REG & GPS_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                GPS_TXDATA_REG = txDataByte;

            #endif /* End GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: GPS_PutString
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: char pointer to character string of Data to Send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  This function will block if there is not enough memory to place the whole
    *  string, it will block until the entire string has been written to the
    *  transmit buffer.
    *
    *******************************************************************************/
    void GPS_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(GPS_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                GPS_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: GPS_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of Bytes to be transmitted.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void GPS_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(GPS_initVar != 0u)
        {
            do
            {
                GPS_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: GPS_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Write a character and then carriage return and line feed.
    *
    * Parameters:
    *  txDataByte: uint8 Character to send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void GPS_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(GPS_initVar != 0u)
        {
            GPS_PutChar(txDataByte);
            GPS_PutChar(0x0Du);
            GPS_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: GPS_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of space left in the TX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Integer count of the number of bytes left in the TX buffer
    *
    * Global Variables:
    *  GPS_txBufferWrite - used to calculate left space.
    *  GPS_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 GPS_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(GPS_TX_INTERRUPT_ENABLED)
                GPS_DisableTxInt();
            #endif /* End GPS_TX_INTERRUPT_ENABLED */

            if(GPS_txBufferRead == GPS_txBufferWrite)
            {
                size = 0u;
            }
            else if(GPS_txBufferRead < GPS_txBufferWrite)
            {
                size = (GPS_txBufferWrite - GPS_txBufferRead);
            }
            else
            {
                size = (GPS_TXBUFFERSIZE - GPS_txBufferRead) + GPS_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(GPS_TX_INTERRUPT_ENABLED)
                GPS_EnableTxInt();
            #endif /* End GPS_TX_INTERRUPT_ENABLED */

        #else /* GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH */

            size = GPS_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & GPS_TX_STS_FIFO_FULL) != 0u)
            {
                size = GPS_FIFO_LENGTH;
            }
            else if((size & GPS_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: GPS_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the TX RAM buffer by setting the read and write pointers both to zero.
    *  Clears the hardware TX FIFO.  Any data present in the FIFO will not be sent.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_txBufferWrite - cleared to zero.
    *  GPS_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM buffer will be lost when overwritten.
    *
    *******************************************************************************/
    void GPS_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        GPS_TXDATA_AUX_CTL_REG |=  GPS_TX_FIFO_CLR;
        GPS_TXDATA_AUX_CTL_REG &= (uint8)~GPS_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(GPS_TX_INTERRUPT_ENABLED)
                GPS_DisableTxInt();
            #endif /* End GPS_TX_INTERRUPT_ENABLED */

            GPS_txBufferRead = 0u;
            GPS_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(GPS_TX_INTERRUPT_ENABLED)
                GPS_EnableTxInt();
            #endif /* End GPS_TX_INTERRUPT_ENABLED */

        #endif /* End GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: GPS_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Write a Break command to the UART
    *
    * Parameters:
    *  uint8 retMode:  Wait mode,
    *   0 - Initialize registers for Break, sends the Break signal and return
    *       imediately.
    *   1 - Wait until Break sending is complete, reinitialize registers to normal
    *       transmission mode then return.
    *   2 - Reinitialize registers to normal transmission mode then return.
    *   3 - both steps: 0 and 1
    *       init registers for Break, send Break signal
    *       wait until Break sending is complete, reinit registers to normal
    *       transmission mode then return.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  GPS_initVar - checked to identify that the component has been
    *     initialized.
    *  tx_period - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  Trere are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Funcition will block CPU untill transmition
    *     complete.
    *  2) User may want to use bloking time if UART configured to the low speed
    *     operation
    *     Emample for this case:
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to init and use the interrupt for complete
    *     break operation.
    *     Example for this case:
    *     Init TX interrupt whith "TX - On TX Complete" parameter
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     When interrupt appear with UART_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *   Uses static variable to keep registers configuration.
    *
    *******************************************************************************/
    void GPS_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(GPS_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(GPS_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == GPS_SEND_BREAK) ||
                    (retMode == GPS_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    GPS_WriteControlRegister(GPS_ReadControlRegister() |
                                                          GPS_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    GPS_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = GPS_TXSTATUS_REG;
                    }while((tmpStat & GPS_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == GPS_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == GPS_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = GPS_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & GPS_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == GPS_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == GPS_REINIT) ||
                    (retMode == GPS_SEND_WAIT_REINIT) )
                {
                    GPS_WriteControlRegister(GPS_ReadControlRegister() &
                                                  (uint8)~GPS_CTRL_HD_SEND_BREAK);
                }

            #else /* GPS_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == GPS_SEND_BREAK) ||
                    (retMode == GPS_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (GPS_PARITY_TYPE != GPS__B_UART__NONE_REVB) || \
                                        (GPS_PARITY_TYPE_SW != 0u) )
                        GPS_WriteControlRegister(GPS_ReadControlRegister() |
                                                              GPS_CTRL_HD_SEND_BREAK);
                    #endif /* End GPS_PARITY_TYPE != GPS__B_UART__NONE_REVB  */

                    #if(GPS_TXCLKGEN_DP)
                        tx_period = GPS_TXBITCLKTX_COMPLETE_REG;
                        GPS_TXBITCLKTX_COMPLETE_REG = GPS_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = GPS_TXBITCTR_PERIOD_REG;
                        GPS_TXBITCTR_PERIOD_REG = GPS_TXBITCTR_BREAKBITS8X;
                    #endif /* End GPS_TXCLKGEN_DP */

                    /* Send zeros*/
                    GPS_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = GPS_TXSTATUS_REG;
                    }while((tmpStat & GPS_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == GPS_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == GPS_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = GPS_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & GPS_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == GPS_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == GPS_REINIT) ||
                    (retMode == GPS_SEND_WAIT_REINIT) )
                {

                    #if(GPS_TXCLKGEN_DP)
                        GPS_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        GPS_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End GPS_TXCLKGEN_DP */

                    #if( (GPS_PARITY_TYPE != GPS__B_UART__NONE_REVB) || \
                         (GPS_PARITY_TYPE_SW != 0u) )
                        GPS_WriteControlRegister(GPS_ReadControlRegister() &
                                                      (uint8)~GPS_CTRL_HD_SEND_BREAK);
                    #endif /* End GPS_PARITY_TYPE != NONE */
                }
            #endif    /* End GPS_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: GPS_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the transmit addressing mode
    *
    * Parameters:
    *  addressMode: 0 -> Space
    *               1 -> Mark
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void GPS_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( GPS_CONTROL_REG_REMOVED == 0u )
                GPS_WriteControlRegister(GPS_ReadControlRegister() |
                                                      GPS_CTRL_MARK);
            #endif /* End GPS_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( GPS_CONTROL_REG_REMOVED == 0u )
                GPS_WriteControlRegister(GPS_ReadControlRegister() &
                                                    (uint8)~GPS_CTRL_MARK);
            #endif /* End GPS_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndGPS_TX_ENABLED */

#if(GPS_HD_ENABLED)


    /*******************************************************************************
    * Function Name: GPS_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Rx configuration if required and loads the
    *  Tx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Tx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART.
    *
    * Side Effects:
    *  Disable RX interrupt mask, when software buffer has been used.
    *
    *******************************************************************************/
    void GPS_LoadTxConfig(void) 
    {
        #if((GPS_RX_INTERRUPT_ENABLED) && (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            GPS_SetRxInterruptMode(0u);
        #endif /* GPS_RX_INTERRUPT_ENABLED */

        GPS_WriteControlRegister(GPS_ReadControlRegister() | GPS_CTRL_HD_SEND);
        GPS_RXBITCTR_PERIOD_REG = GPS_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(GPS_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: GPS_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Tx configuration if required and loads the
    *  Rx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Rx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART
    *
    * Side Effects:
    *  Set RX interrupt mask based on customizer settings, when software buffer
    *  has been used.
    *
    *******************************************************************************/
    void GPS_LoadRxConfig(void) 
    {
        GPS_WriteControlRegister(GPS_ReadControlRegister() &
                                                (uint8)~GPS_CTRL_HD_SEND);
        GPS_RXBITCTR_PERIOD_REG = GPS_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(GPS_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((GPS_RX_INTERRUPT_ENABLED) && (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            GPS_SetRxInterruptMode(GPS_INIT_RX_INTERRUPTS_MASK);
        #endif /* GPS_RX_INTERRUPT_ENABLED */
    }

#endif  /* GPS_HD_ENABLED */


/* [] END OF FILE */
