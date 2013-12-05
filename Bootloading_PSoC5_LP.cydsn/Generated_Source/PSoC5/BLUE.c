/*******************************************************************************
* File Name: BLUE.c
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

#include "BLUE.h"
#include "CyLib.h"
#if(BLUE_INTERNAL_CLOCK_USED)
    #include "BLUE_IntClock.h"
#endif /* End BLUE_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 BLUE_initVar = 0u;
#if( BLUE_TX_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))
    volatile uint8 BLUE_txBuffer[BLUE_TXBUFFERSIZE];
    volatile uint8 BLUE_txBufferRead = 0u;
    uint8 BLUE_txBufferWrite = 0u;
#endif /* End BLUE_TX_ENABLED */
#if( ( BLUE_RX_ENABLED || BLUE_HD_ENABLED ) && \
     (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH) )
    volatile uint8 BLUE_rxBuffer[BLUE_RXBUFFERSIZE];
    volatile uint8 BLUE_rxBufferRead = 0u;
    volatile uint8 BLUE_rxBufferWrite = 0u;
    volatile uint8 BLUE_rxBufferLoopDetect = 0u;
    volatile uint8 BLUE_rxBufferOverflow = 0u;
    #if (BLUE_RXHW_ADDRESS_ENABLED)
        volatile uint8 BLUE_rxAddressMode = BLUE_RXADDRESSMODE;
        volatile uint8 BLUE_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End BLUE_RX_ENABLED */


/*******************************************************************************
* Function Name: BLUE_Start
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
*  The BLUE_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the BLUE_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void BLUE_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(BLUE_initVar == 0u)
    {
        BLUE_Init();
        BLUE_initVar = 1u;
    }
    BLUE_Enable();
}


/*******************************************************************************
* Function Name: BLUE_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  BLUE_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void BLUE_Init(void) 
{
    #if(BLUE_RX_ENABLED || BLUE_HD_ENABLED)

        #if(BLUE_RX_INTERRUPT_ENABLED && (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(BLUE_RX_VECT_NUM, &BLUE_RXISR);
            CyIntSetPriority(BLUE_RX_VECT_NUM, BLUE_RX_PRIOR_NUM);
        #endif /* End BLUE_RX_INTERRUPT_ENABLED */

        #if (BLUE_RXHW_ADDRESS_ENABLED)
            BLUE_SetRxAddressMode(BLUE_RXAddressMode);
            BLUE_SetRxAddress1(BLUE_RXHWADDRESS1);
            BLUE_SetRxAddress2(BLUE_RXHWADDRESS2);
        #endif /* End BLUE_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        BLUE_RXBITCTR_PERIOD_REG = BLUE_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        BLUE_RXSTATUS_MASK_REG  = BLUE_INIT_RX_INTERRUPTS_MASK;
    #endif /* End BLUE_RX_ENABLED || BLUE_HD_ENABLED*/

    #if(BLUE_TX_ENABLED)
        #if(BLUE_TX_INTERRUPT_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(BLUE_TX_VECT_NUM, &BLUE_TXISR);
            CyIntSetPriority(BLUE_TX_VECT_NUM, BLUE_TX_PRIOR_NUM);
        #endif /* End BLUE_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(BLUE_TXCLKGEN_DP)
            BLUE_TXBITCLKGEN_CTR_REG = BLUE_BIT_CENTER;
            BLUE_TXBITCLKTX_COMPLETE_REG = (BLUE_NUMBER_OF_DATA_BITS +
                        BLUE_NUMBER_OF_START_BIT) * BLUE_OVER_SAMPLE_COUNT;
        #else
            BLUE_TXBITCTR_PERIOD_REG = ((BLUE_NUMBER_OF_DATA_BITS +
                        BLUE_NUMBER_OF_START_BIT) * BLUE_OVER_SAMPLE_8) - 1u;
        #endif /* End BLUE_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(BLUE_TX_INTERRUPT_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))
            BLUE_TXSTATUS_MASK_REG = BLUE_TX_STS_FIFO_EMPTY;
        #else
            BLUE_TXSTATUS_MASK_REG = BLUE_INIT_TX_INTERRUPTS_MASK;
        #endif /*End BLUE_TX_INTERRUPT_ENABLED*/

    #endif /* End BLUE_TX_ENABLED */

    #if(BLUE_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        BLUE_WriteControlRegister( \
            (BLUE_ReadControlRegister() & (uint8)~BLUE_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(BLUE_PARITY_TYPE << BLUE_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End BLUE_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: BLUE_Enable
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
*  BLUE_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void BLUE_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(BLUE_RX_ENABLED || BLUE_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        BLUE_RXBITCTR_CONTROL_REG |= BLUE_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        BLUE_RXSTATUS_ACTL_REG  |= BLUE_INT_ENABLE;
        #if(BLUE_RX_INTERRUPT_ENABLED && (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH))
            CyIntEnable(BLUE_RX_VECT_NUM);
            #if (BLUE_RXHW_ADDRESS_ENABLED)
                BLUE_rxAddressDetected = 0u;
            #endif /* End BLUE_RXHW_ADDRESS_ENABLED */
        #endif /* End BLUE_RX_INTERRUPT_ENABLED */
    #endif /* End BLUE_RX_ENABLED || BLUE_HD_ENABLED*/

    #if(BLUE_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!BLUE_TXCLKGEN_DP)
            BLUE_TXBITCTR_CONTROL_REG |= BLUE_CNTR_ENABLE;
        #endif /* End BLUE_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        BLUE_TXSTATUS_ACTL_REG |= BLUE_INT_ENABLE;
        #if(BLUE_TX_INTERRUPT_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))
            CyIntEnable(BLUE_TX_VECT_NUM);
        #endif /* End BLUE_TX_INTERRUPT_ENABLED*/
     #endif /* End BLUE_TX_ENABLED */

    #if(BLUE_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        BLUE_IntClock_Start();
    #endif /* End BLUE_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: BLUE_Stop
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
void BLUE_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(BLUE_RX_ENABLED || BLUE_HD_ENABLED)
        BLUE_RXBITCTR_CONTROL_REG &= (uint8)~BLUE_CNTR_ENABLE;
    #endif /* End BLUE_RX_ENABLED */

    #if(BLUE_TX_ENABLED)
        #if(!BLUE_TXCLKGEN_DP)
            BLUE_TXBITCTR_CONTROL_REG &= (uint8)~BLUE_CNTR_ENABLE;
        #endif /* End BLUE_TXCLKGEN_DP */
    #endif /* BLUE_TX_ENABLED */

    #if(BLUE_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        BLUE_IntClock_Stop();
    #endif /* End BLUE_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(BLUE_RX_ENABLED || BLUE_HD_ENABLED)
        BLUE_RXSTATUS_ACTL_REG  &= (uint8)~BLUE_INT_ENABLE;
        #if(BLUE_RX_INTERRUPT_ENABLED && (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH))
            BLUE_DisableRxInt();
        #endif /* End BLUE_RX_INTERRUPT_ENABLED */
    #endif /* End BLUE_RX_ENABLED */

    #if(BLUE_TX_ENABLED)
        BLUE_TXSTATUS_ACTL_REG &= (uint8)~BLUE_INT_ENABLE;
        #if(BLUE_TX_INTERRUPT_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))
            BLUE_DisableTxInt();
        #endif /* End BLUE_TX_INTERRUPT_ENABLED */
    #endif /* End BLUE_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: BLUE_ReadControlRegister
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
uint8 BLUE_ReadControlRegister(void) 
{
    #if( BLUE_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(BLUE_CONTROL_REG);
    #endif /* End BLUE_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: BLUE_WriteControlRegister
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
void  BLUE_WriteControlRegister(uint8 control) 
{
    #if( BLUE_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       BLUE_CONTROL_REG = control;
    #endif /* End BLUE_CONTROL_REG_REMOVED */
}


#if(BLUE_RX_ENABLED || BLUE_HD_ENABLED)

    #if(BLUE_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: BLUE_EnableRxInt
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
        void BLUE_EnableRxInt(void) 
        {
            CyIntEnable(BLUE_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: BLUE_DisableRxInt
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
        void BLUE_DisableRxInt(void) 
        {
            CyIntDisable(BLUE_RX_VECT_NUM);
        }

    #endif /* BLUE_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: BLUE_SetRxInterruptMode
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
    void BLUE_SetRxInterruptMode(uint8 intSrc) 
    {
        BLUE_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: BLUE_ReadRxData
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
    *  BLUE_rxBuffer - RAM buffer pointer for save received data.
    *  BLUE_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  BLUE_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  BLUE_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 BLUE_ReadRxData(void) 
    {
        uint8 rxData;

        #if(BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_DisableRxInt();
            #endif /* BLUE_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = BLUE_rxBufferRead;
            loc_rxBufferWrite = BLUE_rxBufferWrite;

            if( (BLUE_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = BLUE_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= BLUE_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                BLUE_rxBufferRead = loc_rxBufferRead;

                if(BLUE_rxBufferLoopDetect != 0u )
                {
                    BLUE_rxBufferLoopDetect = 0u;
                    #if( (BLUE_RX_INTERRUPT_ENABLED) && (BLUE_FLOW_CONTROL != 0u) && \
                         (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( BLUE_HD_ENABLED )
                            if((BLUE_CONTROL_REG & BLUE_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                BLUE_RXSTATUS_MASK_REG  |= BLUE_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            BLUE_RXSTATUS_MASK_REG  |= BLUE_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end BLUE_HD_ENABLED */
                    #endif /* BLUE_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = BLUE_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_EnableRxInt();
            #endif /* End BLUE_RX_INTERRUPT_ENABLED */

        #else /* BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = BLUE_RXDATA_REG;

        #endif /* BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: BLUE_ReadRxStatus
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
    *  BLUE_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   BLUE_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   BLUE_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 BLUE_ReadRxStatus(void) 
    {
        uint8 status;

        status = BLUE_RXSTATUS_REG & BLUE_RX_HW_MASK;

        #if(BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH)
            if( BLUE_rxBufferOverflow != 0u )
            {
                status |= BLUE_RX_STS_SOFT_BUFF_OVER;
                BLUE_rxBufferOverflow = 0u;
            }
        #endif /* BLUE_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: BLUE_GetChar
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
    *  BLUE_rxBuffer - RAM buffer pointer for save received data.
    *  BLUE_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  BLUE_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  BLUE_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 BLUE_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_DisableRxInt();
            #endif /* BLUE_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = BLUE_rxBufferRead;
            loc_rxBufferWrite = BLUE_rxBufferWrite;

            if( (BLUE_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = BLUE_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= BLUE_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                BLUE_rxBufferRead = loc_rxBufferRead;

                if(BLUE_rxBufferLoopDetect > 0u )
                {
                    BLUE_rxBufferLoopDetect = 0u;
                    #if( (BLUE_RX_INTERRUPT_ENABLED) && (BLUE_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( BLUE_HD_ENABLED )
                            if((BLUE_CONTROL_REG & BLUE_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                BLUE_RXSTATUS_MASK_REG  |= BLUE_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            BLUE_RXSTATUS_MASK_REG  |= BLUE_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end BLUE_HD_ENABLED */
                    #endif /* BLUE_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = BLUE_RXSTATUS_REG;
                if((rxStatus & BLUE_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = BLUE_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (BLUE_RX_STS_BREAK | BLUE_RX_STS_PAR_ERROR |
                                   BLUE_RX_STS_STOP_ERROR | BLUE_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_EnableRxInt();
            #endif /* BLUE_RX_INTERRUPT_ENABLED */

        #else /* BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH */

            rxStatus =BLUE_RXSTATUS_REG;
            if((rxStatus & BLUE_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = BLUE_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (BLUE_RX_STS_BREAK | BLUE_RX_STS_PAR_ERROR |
                               BLUE_RX_STS_STOP_ERROR | BLUE_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: BLUE_GetByte
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
    uint16 BLUE_GetByte(void) 
    {
        return ( ((uint16)BLUE_ReadRxStatus() << 8u) | BLUE_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: BLUE_GetRxBufferSize
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
    *  BLUE_rxBufferWrite - used to calculate left bytes.
    *  BLUE_rxBufferRead - used to calculate left bytes.
    *  BLUE_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 BLUE_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_DisableRxInt();
            #endif /* BLUE_RX_INTERRUPT_ENABLED */

            if(BLUE_rxBufferRead == BLUE_rxBufferWrite)
            {
                if(BLUE_rxBufferLoopDetect > 0u)
                {
                    size = BLUE_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(BLUE_rxBufferRead < BLUE_rxBufferWrite)
            {
                size = (BLUE_rxBufferWrite - BLUE_rxBufferRead);
            }
            else
            {
                size = (BLUE_RXBUFFERSIZE - BLUE_rxBufferRead) + BLUE_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_EnableRxInt();
            #endif /* End BLUE_RX_INTERRUPT_ENABLED */

        #else /* BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((BLUE_RXSTATUS_REG & BLUE_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: BLUE_ClearRxBuffer
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
    *  BLUE_rxBufferWrite - cleared to zero.
    *  BLUE_rxBufferRead - cleared to zero.
    *  BLUE_rxBufferLoopDetect - cleared to zero.
    *  BLUE_rxBufferOverflow - cleared to zero.
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
    void BLUE_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        BLUE_RXDATA_AUX_CTL_REG |=  BLUE_RX_FIFO_CLR;
        BLUE_RXDATA_AUX_CTL_REG &= (uint8)~BLUE_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_DisableRxInt();
            #endif /* End BLUE_RX_INTERRUPT_ENABLED */

            BLUE_rxBufferRead = 0u;
            BLUE_rxBufferWrite = 0u;
            BLUE_rxBufferLoopDetect = 0u;
            BLUE_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(BLUE_RX_INTERRUPT_ENABLED)
                BLUE_EnableRxInt();
            #endif /* End BLUE_RX_INTERRUPT_ENABLED */
        #endif /* End BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: BLUE_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  BLUE__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  BLUE__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  BLUE__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  BLUE__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  BLUE__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  BLUE_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  BLUE_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void BLUE_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(BLUE_RXHW_ADDRESS_ENABLED)
            #if(BLUE_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* BLUE_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = BLUE_CONTROL_REG & (uint8)~BLUE_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << BLUE_CTRL_RXADDR_MODE0_SHIFT);
                BLUE_CONTROL_REG = tmpCtrl;
                #if(BLUE_RX_INTERRUPT_ENABLED && \
                   (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH) )
                    BLUE_rxAddressMode = addressMode;
                    BLUE_rxAddressDetected = 0u;
                #endif /* End BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH*/
            #endif /* End BLUE_CONTROL_REG_REMOVED */
        #else /* BLUE_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End BLUE_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: BLUE_SetRxAddress1
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
    void BLUE_SetRxAddress1(uint8 address) 

    {
        BLUE_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: BLUE_SetRxAddress2
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
    void BLUE_SetRxAddress2(uint8 address) 
    {
        BLUE_RXADDRESS2_REG = address;
    }

#endif  /* BLUE_RX_ENABLED || BLUE_HD_ENABLED*/


#if( (BLUE_TX_ENABLED) || (BLUE_HD_ENABLED) )

    #if(BLUE_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: BLUE_EnableTxInt
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
        void BLUE_EnableTxInt(void) 
        {
            CyIntEnable(BLUE_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: BLUE_DisableTxInt
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
        void BLUE_DisableTxInt(void) 
        {
            CyIntDisable(BLUE_TX_VECT_NUM);
        }

    #endif /* BLUE_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: BLUE_SetTxInterruptMode
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
    void BLUE_SetTxInterruptMode(uint8 intSrc) 
    {
        BLUE_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: BLUE_WriteTxData
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
    *  BLUE_txBuffer - RAM buffer pointer for save data for transmission
    *  BLUE_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  BLUE_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  BLUE_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void BLUE_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(BLUE_initVar != 0u)
        {
            #if(BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(BLUE_TX_INTERRUPT_ENABLED)
                    BLUE_DisableTxInt();
                #endif /* End BLUE_TX_INTERRUPT_ENABLED */

                if( (BLUE_txBufferRead == BLUE_txBufferWrite) &&
                    ((BLUE_TXSTATUS_REG & BLUE_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    BLUE_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(BLUE_txBufferWrite >= BLUE_TXBUFFERSIZE)
                    {
                        BLUE_txBufferWrite = 0u;
                    }

                    BLUE_txBuffer[BLUE_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    BLUE_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(BLUE_TX_INTERRUPT_ENABLED)
                    BLUE_EnableTxInt();
                #endif /* End BLUE_TX_INTERRUPT_ENABLED */

            #else /* BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                BLUE_TXDATA_REG = txDataByte;

            #endif /* End BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_ReadTxStatus
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
    uint8 BLUE_ReadTxStatus(void) 
    {
        return(BLUE_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: BLUE_PutChar
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
    *  BLUE_txBuffer - RAM buffer pointer for save data for transmission
    *  BLUE_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  BLUE_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  BLUE_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void BLUE_PutChar(uint8 txDataByte) 
    {
            #if(BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((BLUE_TXBUFFERSIZE > BLUE_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(BLUE_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = BLUE_txBufferWrite;
                    loc_txBufferRead = BLUE_txBufferRead;
                    #if ((BLUE_TXBUFFERSIZE > BLUE_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(BLUE_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(BLUE_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((BLUE_TXSTATUS_REG & BLUE_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    BLUE_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= BLUE_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    BLUE_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((BLUE_TXBUFFERSIZE > BLUE_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(BLUE_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    BLUE_txBufferWrite = loc_txBufferWrite;
                    #if ((BLUE_TXBUFFERSIZE > BLUE_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(BLUE_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH */

                while((BLUE_TXSTATUS_REG & BLUE_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                BLUE_TXDATA_REG = txDataByte;

            #endif /* End BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: BLUE_PutString
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
    *  BLUE_initVar - checked to identify that the component has been
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
    void BLUE_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(BLUE_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                BLUE_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_PutArray
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
    *  BLUE_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void BLUE_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(BLUE_initVar != 0u)
        {
            do
            {
                BLUE_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_PutCRLF
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
    *  BLUE_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void BLUE_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(BLUE_initVar != 0u)
        {
            BLUE_PutChar(txDataByte);
            BLUE_PutChar(0x0Du);
            BLUE_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_GetTxBufferSize
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
    *  BLUE_txBufferWrite - used to calculate left space.
    *  BLUE_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 BLUE_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(BLUE_TX_INTERRUPT_ENABLED)
                BLUE_DisableTxInt();
            #endif /* End BLUE_TX_INTERRUPT_ENABLED */

            if(BLUE_txBufferRead == BLUE_txBufferWrite)
            {
                size = 0u;
            }
            else if(BLUE_txBufferRead < BLUE_txBufferWrite)
            {
                size = (BLUE_txBufferWrite - BLUE_txBufferRead);
            }
            else
            {
                size = (BLUE_TXBUFFERSIZE - BLUE_txBufferRead) + BLUE_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(BLUE_TX_INTERRUPT_ENABLED)
                BLUE_EnableTxInt();
            #endif /* End BLUE_TX_INTERRUPT_ENABLED */

        #else /* BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH */

            size = BLUE_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & BLUE_TX_STS_FIFO_FULL) != 0u)
            {
                size = BLUE_FIFO_LENGTH;
            }
            else if((size & BLUE_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: BLUE_ClearTxBuffer
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
    *  BLUE_txBufferWrite - cleared to zero.
    *  BLUE_txBufferRead - cleared to zero.
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
    void BLUE_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        BLUE_TXDATA_AUX_CTL_REG |=  BLUE_TX_FIFO_CLR;
        BLUE_TXDATA_AUX_CTL_REG &= (uint8)~BLUE_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(BLUE_TX_INTERRUPT_ENABLED)
                BLUE_DisableTxInt();
            #endif /* End BLUE_TX_INTERRUPT_ENABLED */

            BLUE_txBufferRead = 0u;
            BLUE_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(BLUE_TX_INTERRUPT_ENABLED)
                BLUE_EnableTxInt();
            #endif /* End BLUE_TX_INTERRUPT_ENABLED */

        #endif /* End BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: BLUE_SendBreak
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
    *  BLUE_initVar - checked to identify that the component has been
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
    void BLUE_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(BLUE_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(BLUE_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == BLUE_SEND_BREAK) ||
                    (retMode == BLUE_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    BLUE_WriteControlRegister(BLUE_ReadControlRegister() |
                                                          BLUE_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    BLUE_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = BLUE_TXSTATUS_REG;
                    }while((tmpStat & BLUE_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == BLUE_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == BLUE_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = BLUE_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & BLUE_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == BLUE_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == BLUE_REINIT) ||
                    (retMode == BLUE_SEND_WAIT_REINIT) )
                {
                    BLUE_WriteControlRegister(BLUE_ReadControlRegister() &
                                                  (uint8)~BLUE_CTRL_HD_SEND_BREAK);
                }

            #else /* BLUE_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == BLUE_SEND_BREAK) ||
                    (retMode == BLUE_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (BLUE_PARITY_TYPE != BLUE__B_UART__NONE_REVB) || \
                                        (BLUE_PARITY_TYPE_SW != 0u) )
                        BLUE_WriteControlRegister(BLUE_ReadControlRegister() |
                                                              BLUE_CTRL_HD_SEND_BREAK);
                    #endif /* End BLUE_PARITY_TYPE != BLUE__B_UART__NONE_REVB  */

                    #if(BLUE_TXCLKGEN_DP)
                        tx_period = BLUE_TXBITCLKTX_COMPLETE_REG;
                        BLUE_TXBITCLKTX_COMPLETE_REG = BLUE_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = BLUE_TXBITCTR_PERIOD_REG;
                        BLUE_TXBITCTR_PERIOD_REG = BLUE_TXBITCTR_BREAKBITS8X;
                    #endif /* End BLUE_TXCLKGEN_DP */

                    /* Send zeros*/
                    BLUE_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = BLUE_TXSTATUS_REG;
                    }while((tmpStat & BLUE_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == BLUE_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == BLUE_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = BLUE_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & BLUE_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == BLUE_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == BLUE_REINIT) ||
                    (retMode == BLUE_SEND_WAIT_REINIT) )
                {

                    #if(BLUE_TXCLKGEN_DP)
                        BLUE_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        BLUE_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End BLUE_TXCLKGEN_DP */

                    #if( (BLUE_PARITY_TYPE != BLUE__B_UART__NONE_REVB) || \
                         (BLUE_PARITY_TYPE_SW != 0u) )
                        BLUE_WriteControlRegister(BLUE_ReadControlRegister() &
                                                      (uint8)~BLUE_CTRL_HD_SEND_BREAK);
                    #endif /* End BLUE_PARITY_TYPE != NONE */
                }
            #endif    /* End BLUE_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: BLUE_SetTxAddressMode
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
    void BLUE_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( BLUE_CONTROL_REG_REMOVED == 0u )
                BLUE_WriteControlRegister(BLUE_ReadControlRegister() |
                                                      BLUE_CTRL_MARK);
            #endif /* End BLUE_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( BLUE_CONTROL_REG_REMOVED == 0u )
                BLUE_WriteControlRegister(BLUE_ReadControlRegister() &
                                                    (uint8)~BLUE_CTRL_MARK);
            #endif /* End BLUE_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndBLUE_TX_ENABLED */

#if(BLUE_HD_ENABLED)


    /*******************************************************************************
    * Function Name: BLUE_LoadTxConfig
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
    void BLUE_LoadTxConfig(void) 
    {
        #if((BLUE_RX_INTERRUPT_ENABLED) && (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            BLUE_SetRxInterruptMode(0u);
        #endif /* BLUE_RX_INTERRUPT_ENABLED */

        BLUE_WriteControlRegister(BLUE_ReadControlRegister() | BLUE_CTRL_HD_SEND);
        BLUE_RXBITCTR_PERIOD_REG = BLUE_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(BLUE_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: BLUE_LoadRxConfig
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
    void BLUE_LoadRxConfig(void) 
    {
        BLUE_WriteControlRegister(BLUE_ReadControlRegister() &
                                                (uint8)~BLUE_CTRL_HD_SEND);
        BLUE_RXBITCTR_PERIOD_REG = BLUE_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(BLUE_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((BLUE_RX_INTERRUPT_ENABLED) && (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            BLUE_SetRxInterruptMode(BLUE_INIT_RX_INTERRUPTS_MASK);
        #endif /* BLUE_RX_INTERRUPT_ENABLED */
    }

#endif  /* BLUE_HD_ENABLED */


/* [] END OF FILE */
