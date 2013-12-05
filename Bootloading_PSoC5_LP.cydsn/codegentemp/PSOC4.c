/*******************************************************************************
* File Name: PSOC4.c
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

#include "PSOC4.h"
#include "CyLib.h"
#if(PSOC4_INTERNAL_CLOCK_USED)
    #include "PSOC4_IntClock.h"
#endif /* End PSOC4_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 PSOC4_initVar = 0u;
#if( PSOC4_TX_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))
    volatile uint8 PSOC4_txBuffer[PSOC4_TXBUFFERSIZE];
    volatile uint8 PSOC4_txBufferRead = 0u;
    uint8 PSOC4_txBufferWrite = 0u;
#endif /* End PSOC4_TX_ENABLED */
#if( ( PSOC4_RX_ENABLED || PSOC4_HD_ENABLED ) && \
     (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH) )
    volatile uint8 PSOC4_rxBuffer[PSOC4_RXBUFFERSIZE];
    volatile uint8 PSOC4_rxBufferRead = 0u;
    volatile uint8 PSOC4_rxBufferWrite = 0u;
    volatile uint8 PSOC4_rxBufferLoopDetect = 0u;
    volatile uint8 PSOC4_rxBufferOverflow = 0u;
    #if (PSOC4_RXHW_ADDRESS_ENABLED)
        volatile uint8 PSOC4_rxAddressMode = PSOC4_RXADDRESSMODE;
        volatile uint8 PSOC4_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End PSOC4_RX_ENABLED */


/*******************************************************************************
* Function Name: PSOC4_Start
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
*  The PSOC4_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the PSOC4_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PSOC4_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(PSOC4_initVar == 0u)
    {
        PSOC4_Init();
        PSOC4_initVar = 1u;
    }
    PSOC4_Enable();
}


/*******************************************************************************
* Function Name: PSOC4_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  PSOC4_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void PSOC4_Init(void) 
{
    #if(PSOC4_RX_ENABLED || PSOC4_HD_ENABLED)

        #if(PSOC4_RX_INTERRUPT_ENABLED && (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(PSOC4_RX_VECT_NUM, &PSOC4_RXISR);
            CyIntSetPriority(PSOC4_RX_VECT_NUM, PSOC4_RX_PRIOR_NUM);
        #endif /* End PSOC4_RX_INTERRUPT_ENABLED */

        #if (PSOC4_RXHW_ADDRESS_ENABLED)
            PSOC4_SetRxAddressMode(PSOC4_RXAddressMode);
            PSOC4_SetRxAddress1(PSOC4_RXHWADDRESS1);
            PSOC4_SetRxAddress2(PSOC4_RXHWADDRESS2);
        #endif /* End PSOC4_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        PSOC4_RXBITCTR_PERIOD_REG = PSOC4_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        PSOC4_RXSTATUS_MASK_REG  = PSOC4_INIT_RX_INTERRUPTS_MASK;
    #endif /* End PSOC4_RX_ENABLED || PSOC4_HD_ENABLED*/

    #if(PSOC4_TX_ENABLED)
        #if(PSOC4_TX_INTERRUPT_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(PSOC4_TX_VECT_NUM, &PSOC4_TXISR);
            CyIntSetPriority(PSOC4_TX_VECT_NUM, PSOC4_TX_PRIOR_NUM);
        #endif /* End PSOC4_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(PSOC4_TXCLKGEN_DP)
            PSOC4_TXBITCLKGEN_CTR_REG = PSOC4_BIT_CENTER;
            PSOC4_TXBITCLKTX_COMPLETE_REG = (PSOC4_NUMBER_OF_DATA_BITS +
                        PSOC4_NUMBER_OF_START_BIT) * PSOC4_OVER_SAMPLE_COUNT;
        #else
            PSOC4_TXBITCTR_PERIOD_REG = ((PSOC4_NUMBER_OF_DATA_BITS +
                        PSOC4_NUMBER_OF_START_BIT) * PSOC4_OVER_SAMPLE_8) - 1u;
        #endif /* End PSOC4_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(PSOC4_TX_INTERRUPT_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            PSOC4_TXSTATUS_MASK_REG = PSOC4_TX_STS_FIFO_EMPTY;
        #else
            PSOC4_TXSTATUS_MASK_REG = PSOC4_INIT_TX_INTERRUPTS_MASK;
        #endif /*End PSOC4_TX_INTERRUPT_ENABLED*/

    #endif /* End PSOC4_TX_ENABLED */

    #if(PSOC4_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        PSOC4_WriteControlRegister( \
            (PSOC4_ReadControlRegister() & (uint8)~PSOC4_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(PSOC4_PARITY_TYPE << PSOC4_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End PSOC4_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: PSOC4_Enable
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
*  PSOC4_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void PSOC4_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(PSOC4_RX_ENABLED || PSOC4_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        PSOC4_RXBITCTR_CONTROL_REG |= PSOC4_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        PSOC4_RXSTATUS_ACTL_REG  |= PSOC4_INT_ENABLE;
        #if(PSOC4_RX_INTERRUPT_ENABLED && (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            CyIntEnable(PSOC4_RX_VECT_NUM);
            #if (PSOC4_RXHW_ADDRESS_ENABLED)
                PSOC4_rxAddressDetected = 0u;
            #endif /* End PSOC4_RXHW_ADDRESS_ENABLED */
        #endif /* End PSOC4_RX_INTERRUPT_ENABLED */
    #endif /* End PSOC4_RX_ENABLED || PSOC4_HD_ENABLED*/

    #if(PSOC4_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!PSOC4_TXCLKGEN_DP)
            PSOC4_TXBITCTR_CONTROL_REG |= PSOC4_CNTR_ENABLE;
        #endif /* End PSOC4_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        PSOC4_TXSTATUS_ACTL_REG |= PSOC4_INT_ENABLE;
        #if(PSOC4_TX_INTERRUPT_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            CyIntEnable(PSOC4_TX_VECT_NUM);
        #endif /* End PSOC4_TX_INTERRUPT_ENABLED*/
     #endif /* End PSOC4_TX_ENABLED */

    #if(PSOC4_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        PSOC4_IntClock_Start();
    #endif /* End PSOC4_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PSOC4_Stop
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
void PSOC4_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(PSOC4_RX_ENABLED || PSOC4_HD_ENABLED)
        PSOC4_RXBITCTR_CONTROL_REG &= (uint8)~PSOC4_CNTR_ENABLE;
    #endif /* End PSOC4_RX_ENABLED */

    #if(PSOC4_TX_ENABLED)
        #if(!PSOC4_TXCLKGEN_DP)
            PSOC4_TXBITCTR_CONTROL_REG &= (uint8)~PSOC4_CNTR_ENABLE;
        #endif /* End PSOC4_TXCLKGEN_DP */
    #endif /* PSOC4_TX_ENABLED */

    #if(PSOC4_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        PSOC4_IntClock_Stop();
    #endif /* End PSOC4_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(PSOC4_RX_ENABLED || PSOC4_HD_ENABLED)
        PSOC4_RXSTATUS_ACTL_REG  &= (uint8)~PSOC4_INT_ENABLE;
        #if(PSOC4_RX_INTERRUPT_ENABLED && (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            PSOC4_DisableRxInt();
        #endif /* End PSOC4_RX_INTERRUPT_ENABLED */
    #endif /* End PSOC4_RX_ENABLED */

    #if(PSOC4_TX_ENABLED)
        PSOC4_TXSTATUS_ACTL_REG &= (uint8)~PSOC4_INT_ENABLE;
        #if(PSOC4_TX_INTERRUPT_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            PSOC4_DisableTxInt();
        #endif /* End PSOC4_TX_INTERRUPT_ENABLED */
    #endif /* End PSOC4_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PSOC4_ReadControlRegister
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
uint8 PSOC4_ReadControlRegister(void) 
{
    #if( PSOC4_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(PSOC4_CONTROL_REG);
    #endif /* End PSOC4_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: PSOC4_WriteControlRegister
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
void  PSOC4_WriteControlRegister(uint8 control) 
{
    #if( PSOC4_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       PSOC4_CONTROL_REG = control;
    #endif /* End PSOC4_CONTROL_REG_REMOVED */
}


#if(PSOC4_RX_ENABLED || PSOC4_HD_ENABLED)

    #if(PSOC4_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: PSOC4_EnableRxInt
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
        void PSOC4_EnableRxInt(void) 
        {
            CyIntEnable(PSOC4_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: PSOC4_DisableRxInt
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
        void PSOC4_DisableRxInt(void) 
        {
            CyIntDisable(PSOC4_RX_VECT_NUM);
        }

    #endif /* PSOC4_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: PSOC4_SetRxInterruptMode
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
    void PSOC4_SetRxInterruptMode(uint8 intSrc) 
    {
        PSOC4_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: PSOC4_ReadRxData
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
    *  PSOC4_rxBuffer - RAM buffer pointer for save received data.
    *  PSOC4_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  PSOC4_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  PSOC4_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 PSOC4_ReadRxData(void) 
    {
        uint8 rxData;

        #if(PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_DisableRxInt();
            #endif /* PSOC4_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = PSOC4_rxBufferRead;
            loc_rxBufferWrite = PSOC4_rxBufferWrite;

            if( (PSOC4_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = PSOC4_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= PSOC4_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                PSOC4_rxBufferRead = loc_rxBufferRead;

                if(PSOC4_rxBufferLoopDetect != 0u )
                {
                    PSOC4_rxBufferLoopDetect = 0u;
                    #if( (PSOC4_RX_INTERRUPT_ENABLED) && (PSOC4_FLOW_CONTROL != 0u) && \
                         (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( PSOC4_HD_ENABLED )
                            if((PSOC4_CONTROL_REG & PSOC4_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                PSOC4_RXSTATUS_MASK_REG  |= PSOC4_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            PSOC4_RXSTATUS_MASK_REG  |= PSOC4_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end PSOC4_HD_ENABLED */
                    #endif /* PSOC4_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = PSOC4_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_EnableRxInt();
            #endif /* End PSOC4_RX_INTERRUPT_ENABLED */

        #else /* PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = PSOC4_RXDATA_REG;

        #endif /* PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PSOC4_ReadRxStatus
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
    *  PSOC4_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   PSOC4_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   PSOC4_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 PSOC4_ReadRxStatus(void) 
    {
        uint8 status;

        status = PSOC4_RXSTATUS_REG & PSOC4_RX_HW_MASK;

        #if(PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH)
            if( PSOC4_rxBufferOverflow != 0u )
            {
                status |= PSOC4_RX_STS_SOFT_BUFF_OVER;
                PSOC4_rxBufferOverflow = 0u;
            }
        #endif /* PSOC4_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: PSOC4_GetChar
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
    *  PSOC4_rxBuffer - RAM buffer pointer for save received data.
    *  PSOC4_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  PSOC4_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  PSOC4_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 PSOC4_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_DisableRxInt();
            #endif /* PSOC4_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = PSOC4_rxBufferRead;
            loc_rxBufferWrite = PSOC4_rxBufferWrite;

            if( (PSOC4_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = PSOC4_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= PSOC4_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                PSOC4_rxBufferRead = loc_rxBufferRead;

                if(PSOC4_rxBufferLoopDetect > 0u )
                {
                    PSOC4_rxBufferLoopDetect = 0u;
                    #if( (PSOC4_RX_INTERRUPT_ENABLED) && (PSOC4_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( PSOC4_HD_ENABLED )
                            if((PSOC4_CONTROL_REG & PSOC4_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                PSOC4_RXSTATUS_MASK_REG  |= PSOC4_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            PSOC4_RXSTATUS_MASK_REG  |= PSOC4_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end PSOC4_HD_ENABLED */
                    #endif /* PSOC4_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = PSOC4_RXSTATUS_REG;
                if((rxStatus & PSOC4_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = PSOC4_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (PSOC4_RX_STS_BREAK | PSOC4_RX_STS_PAR_ERROR |
                                   PSOC4_RX_STS_STOP_ERROR | PSOC4_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_EnableRxInt();
            #endif /* PSOC4_RX_INTERRUPT_ENABLED */

        #else /* PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH */

            rxStatus =PSOC4_RXSTATUS_REG;
            if((rxStatus & PSOC4_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = PSOC4_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (PSOC4_RX_STS_BREAK | PSOC4_RX_STS_PAR_ERROR |
                               PSOC4_RX_STS_STOP_ERROR | PSOC4_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PSOC4_GetByte
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
    uint16 PSOC4_GetByte(void) 
    {
        return ( ((uint16)PSOC4_ReadRxStatus() << 8u) | PSOC4_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: PSOC4_GetRxBufferSize
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
    *  PSOC4_rxBufferWrite - used to calculate left bytes.
    *  PSOC4_rxBufferRead - used to calculate left bytes.
    *  PSOC4_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 PSOC4_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_DisableRxInt();
            #endif /* PSOC4_RX_INTERRUPT_ENABLED */

            if(PSOC4_rxBufferRead == PSOC4_rxBufferWrite)
            {
                if(PSOC4_rxBufferLoopDetect > 0u)
                {
                    size = PSOC4_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(PSOC4_rxBufferRead < PSOC4_rxBufferWrite)
            {
                size = (PSOC4_rxBufferWrite - PSOC4_rxBufferRead);
            }
            else
            {
                size = (PSOC4_RXBUFFERSIZE - PSOC4_rxBufferRead) + PSOC4_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_EnableRxInt();
            #endif /* End PSOC4_RX_INTERRUPT_ENABLED */

        #else /* PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((PSOC4_RXSTATUS_REG & PSOC4_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: PSOC4_ClearRxBuffer
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
    *  PSOC4_rxBufferWrite - cleared to zero.
    *  PSOC4_rxBufferRead - cleared to zero.
    *  PSOC4_rxBufferLoopDetect - cleared to zero.
    *  PSOC4_rxBufferOverflow - cleared to zero.
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
    void PSOC4_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        PSOC4_RXDATA_AUX_CTL_REG |=  PSOC4_RX_FIFO_CLR;
        PSOC4_RXDATA_AUX_CTL_REG &= (uint8)~PSOC4_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_DisableRxInt();
            #endif /* End PSOC4_RX_INTERRUPT_ENABLED */

            PSOC4_rxBufferRead = 0u;
            PSOC4_rxBufferWrite = 0u;
            PSOC4_rxBufferLoopDetect = 0u;
            PSOC4_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(PSOC4_RX_INTERRUPT_ENABLED)
                PSOC4_EnableRxInt();
            #endif /* End PSOC4_RX_INTERRUPT_ENABLED */
        #endif /* End PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: PSOC4_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  PSOC4__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  PSOC4__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  PSOC4__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  PSOC4__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  PSOC4__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  PSOC4_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  PSOC4_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void PSOC4_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(PSOC4_RXHW_ADDRESS_ENABLED)
            #if(PSOC4_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* PSOC4_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = PSOC4_CONTROL_REG & (uint8)~PSOC4_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << PSOC4_CTRL_RXADDR_MODE0_SHIFT);
                PSOC4_CONTROL_REG = tmpCtrl;
                #if(PSOC4_RX_INTERRUPT_ENABLED && \
                   (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH) )
                    PSOC4_rxAddressMode = addressMode;
                    PSOC4_rxAddressDetected = 0u;
                #endif /* End PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH*/
            #endif /* End PSOC4_CONTROL_REG_REMOVED */
        #else /* PSOC4_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End PSOC4_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: PSOC4_SetRxAddress1
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
    void PSOC4_SetRxAddress1(uint8 address) 

    {
        PSOC4_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: PSOC4_SetRxAddress2
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
    void PSOC4_SetRxAddress2(uint8 address) 
    {
        PSOC4_RXADDRESS2_REG = address;
    }

#endif  /* PSOC4_RX_ENABLED || PSOC4_HD_ENABLED*/


#if( (PSOC4_TX_ENABLED) || (PSOC4_HD_ENABLED) )

    #if(PSOC4_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: PSOC4_EnableTxInt
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
        void PSOC4_EnableTxInt(void) 
        {
            CyIntEnable(PSOC4_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: PSOC4_DisableTxInt
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
        void PSOC4_DisableTxInt(void) 
        {
            CyIntDisable(PSOC4_TX_VECT_NUM);
        }

    #endif /* PSOC4_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: PSOC4_SetTxInterruptMode
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
    void PSOC4_SetTxInterruptMode(uint8 intSrc) 
    {
        PSOC4_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: PSOC4_WriteTxData
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
    *  PSOC4_txBuffer - RAM buffer pointer for save data for transmission
    *  PSOC4_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  PSOC4_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  PSOC4_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void PSOC4_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(PSOC4_initVar != 0u)
        {
            #if(PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(PSOC4_TX_INTERRUPT_ENABLED)
                    PSOC4_DisableTxInt();
                #endif /* End PSOC4_TX_INTERRUPT_ENABLED */

                if( (PSOC4_txBufferRead == PSOC4_txBufferWrite) &&
                    ((PSOC4_TXSTATUS_REG & PSOC4_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    PSOC4_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(PSOC4_txBufferWrite >= PSOC4_TXBUFFERSIZE)
                    {
                        PSOC4_txBufferWrite = 0u;
                    }

                    PSOC4_txBuffer[PSOC4_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    PSOC4_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(PSOC4_TX_INTERRUPT_ENABLED)
                    PSOC4_EnableTxInt();
                #endif /* End PSOC4_TX_INTERRUPT_ENABLED */

            #else /* PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                PSOC4_TXDATA_REG = txDataByte;

            #endif /* End PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: PSOC4_ReadTxStatus
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
    uint8 PSOC4_ReadTxStatus(void) 
    {
        return(PSOC4_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: PSOC4_PutChar
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
    *  PSOC4_txBuffer - RAM buffer pointer for save data for transmission
    *  PSOC4_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  PSOC4_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  PSOC4_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void PSOC4_PutChar(uint8 txDataByte) 
    {
            #if(PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((PSOC4_TXBUFFERSIZE > PSOC4_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(PSOC4_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = PSOC4_txBufferWrite;
                    loc_txBufferRead = PSOC4_txBufferRead;
                    #if ((PSOC4_TXBUFFERSIZE > PSOC4_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(PSOC4_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(PSOC4_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((PSOC4_TXSTATUS_REG & PSOC4_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    PSOC4_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= PSOC4_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    PSOC4_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((PSOC4_TXBUFFERSIZE > PSOC4_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(PSOC4_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    PSOC4_txBufferWrite = loc_txBufferWrite;
                    #if ((PSOC4_TXBUFFERSIZE > PSOC4_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(PSOC4_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH */

                while((PSOC4_TXSTATUS_REG & PSOC4_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                PSOC4_TXDATA_REG = txDataByte;

            #endif /* End PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: PSOC4_PutString
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
    *  PSOC4_initVar - checked to identify that the component has been
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
    void PSOC4_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(PSOC4_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                PSOC4_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: PSOC4_PutArray
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
    *  PSOC4_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void PSOC4_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(PSOC4_initVar != 0u)
        {
            do
            {
                PSOC4_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: PSOC4_PutCRLF
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
    *  PSOC4_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void PSOC4_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(PSOC4_initVar != 0u)
        {
            PSOC4_PutChar(txDataByte);
            PSOC4_PutChar(0x0Du);
            PSOC4_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: PSOC4_GetTxBufferSize
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
    *  PSOC4_txBufferWrite - used to calculate left space.
    *  PSOC4_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 PSOC4_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PSOC4_TX_INTERRUPT_ENABLED)
                PSOC4_DisableTxInt();
            #endif /* End PSOC4_TX_INTERRUPT_ENABLED */

            if(PSOC4_txBufferRead == PSOC4_txBufferWrite)
            {
                size = 0u;
            }
            else if(PSOC4_txBufferRead < PSOC4_txBufferWrite)
            {
                size = (PSOC4_txBufferWrite - PSOC4_txBufferRead);
            }
            else
            {
                size = (PSOC4_TXBUFFERSIZE - PSOC4_txBufferRead) + PSOC4_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(PSOC4_TX_INTERRUPT_ENABLED)
                PSOC4_EnableTxInt();
            #endif /* End PSOC4_TX_INTERRUPT_ENABLED */

        #else /* PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH */

            size = PSOC4_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & PSOC4_TX_STS_FIFO_FULL) != 0u)
            {
                size = PSOC4_FIFO_LENGTH;
            }
            else if((size & PSOC4_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: PSOC4_ClearTxBuffer
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
    *  PSOC4_txBufferWrite - cleared to zero.
    *  PSOC4_txBufferRead - cleared to zero.
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
    void PSOC4_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        PSOC4_TXDATA_AUX_CTL_REG |=  PSOC4_TX_FIFO_CLR;
        PSOC4_TXDATA_AUX_CTL_REG &= (uint8)~PSOC4_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PSOC4_TX_INTERRUPT_ENABLED)
                PSOC4_DisableTxInt();
            #endif /* End PSOC4_TX_INTERRUPT_ENABLED */

            PSOC4_txBufferRead = 0u;
            PSOC4_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(PSOC4_TX_INTERRUPT_ENABLED)
                PSOC4_EnableTxInt();
            #endif /* End PSOC4_TX_INTERRUPT_ENABLED */

        #endif /* End PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: PSOC4_SendBreak
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
    *  PSOC4_initVar - checked to identify that the component has been
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
    void PSOC4_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(PSOC4_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(PSOC4_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == PSOC4_SEND_BREAK) ||
                    (retMode == PSOC4_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() |
                                                          PSOC4_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    PSOC4_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = PSOC4_TXSTATUS_REG;
                    }while((tmpStat & PSOC4_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == PSOC4_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PSOC4_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = PSOC4_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & PSOC4_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == PSOC4_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PSOC4_REINIT) ||
                    (retMode == PSOC4_SEND_WAIT_REINIT) )
                {
                    PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() &
                                                  (uint8)~PSOC4_CTRL_HD_SEND_BREAK);
                }

            #else /* PSOC4_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == PSOC4_SEND_BREAK) ||
                    (retMode == PSOC4_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (PSOC4_PARITY_TYPE != PSOC4__B_UART__NONE_REVB) || \
                                        (PSOC4_PARITY_TYPE_SW != 0u) )
                        PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() |
                                                              PSOC4_CTRL_HD_SEND_BREAK);
                    #endif /* End PSOC4_PARITY_TYPE != PSOC4__B_UART__NONE_REVB  */

                    #if(PSOC4_TXCLKGEN_DP)
                        tx_period = PSOC4_TXBITCLKTX_COMPLETE_REG;
                        PSOC4_TXBITCLKTX_COMPLETE_REG = PSOC4_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = PSOC4_TXBITCTR_PERIOD_REG;
                        PSOC4_TXBITCTR_PERIOD_REG = PSOC4_TXBITCTR_BREAKBITS8X;
                    #endif /* End PSOC4_TXCLKGEN_DP */

                    /* Send zeros*/
                    PSOC4_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = PSOC4_TXSTATUS_REG;
                    }while((tmpStat & PSOC4_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == PSOC4_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PSOC4_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = PSOC4_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & PSOC4_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == PSOC4_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PSOC4_REINIT) ||
                    (retMode == PSOC4_SEND_WAIT_REINIT) )
                {

                    #if(PSOC4_TXCLKGEN_DP)
                        PSOC4_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        PSOC4_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End PSOC4_TXCLKGEN_DP */

                    #if( (PSOC4_PARITY_TYPE != PSOC4__B_UART__NONE_REVB) || \
                         (PSOC4_PARITY_TYPE_SW != 0u) )
                        PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() &
                                                      (uint8)~PSOC4_CTRL_HD_SEND_BREAK);
                    #endif /* End PSOC4_PARITY_TYPE != NONE */
                }
            #endif    /* End PSOC4_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: PSOC4_SetTxAddressMode
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
    void PSOC4_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( PSOC4_CONTROL_REG_REMOVED == 0u )
                PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() |
                                                      PSOC4_CTRL_MARK);
            #endif /* End PSOC4_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( PSOC4_CONTROL_REG_REMOVED == 0u )
                PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() &
                                                    (uint8)~PSOC4_CTRL_MARK);
            #endif /* End PSOC4_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndPSOC4_TX_ENABLED */

#if(PSOC4_HD_ENABLED)


    /*******************************************************************************
    * Function Name: PSOC4_LoadTxConfig
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
    void PSOC4_LoadTxConfig(void) 
    {
        #if((PSOC4_RX_INTERRUPT_ENABLED) && (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            PSOC4_SetRxInterruptMode(0u);
        #endif /* PSOC4_RX_INTERRUPT_ENABLED */

        PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() | PSOC4_CTRL_HD_SEND);
        PSOC4_RXBITCTR_PERIOD_REG = PSOC4_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(PSOC4_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: PSOC4_LoadRxConfig
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
    void PSOC4_LoadRxConfig(void) 
    {
        PSOC4_WriteControlRegister(PSOC4_ReadControlRegister() &
                                                (uint8)~PSOC4_CTRL_HD_SEND);
        PSOC4_RXBITCTR_PERIOD_REG = PSOC4_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(PSOC4_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((PSOC4_RX_INTERRUPT_ENABLED) && (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            PSOC4_SetRxInterruptMode(PSOC4_INIT_RX_INTERRUPTS_MASK);
        #endif /* PSOC4_RX_INTERRUPT_ENABLED */
    }

#endif  /* PSOC4_HD_ENABLED */


/* [] END OF FILE */
