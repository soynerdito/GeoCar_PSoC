/*******************************************************************************
* File Name: GPS.c
* Version 1.10
*
* Description:
*  This file provides the source code to the API for the SCB Component.
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

#if(GPS_SCB_MODE_I2C_INC)
    #include "GPS_I2C_PVT.h"
#endif /* (GPS_SCB_MODE_I2C_INC) */

#if(GPS_SCB_MODE_EZI2C_INC)
    #include "GPS_EZI2C_PVT.h"
#endif /* (GPS_SCB_MODE_EZI2C_INC) */

#if(GPS_SCB_MODE_SPI_INC || GPS_SCB_MODE_UART_INC)
    #include "GPS_SPI_UART_PVT.h"
#endif /* (GPS_SCB_MODE_SPI_INC || GPS_SCB_MODE_UART_INC) */


/**********************************
*    Run Time Configuration Vars
**********************************/

/* Stores internal component configuration for unconfigured mode */
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    uint8 GPS_scbMode = GPS_SCB_MODE_UNCONFIG;
    uint8 GPS_scbEnableWake;
    uint8 GPS_scbEnableIntr;

    /* I2C config vars */
    uint8 GPS_mode;
    uint8 GPS_acceptAddr;

    /* SPI/UART config vars */
    volatile uint8 * GPS_rxBuffer;
    uint8  GPS_rxDataBits;
    uint32 GPS_rxBufferSize;

    volatile uint8 * GPS_txBuffer;
    uint8  GPS_txDataBits;
    uint32 GPS_txBufferSize;

    /* EZI2C config vars */
    uint8 GPS_numberOfAddr;
    uint8 GPS_subAddrSize;
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */


/**********************************
*     Common SCB Vars
**********************************/

uint8 GPS_initVar = 0u;
cyisraddress GPS_customIntrHandler = NULL;


/***************************************
*    Private Function Prototypes
***************************************/

static void GPS_ScbEnableIntr(void);
static void GPS_ScbModeStop(void);


/*******************************************************************************
* Function Name: GPS_Init
********************************************************************************
*
* Summary:
*  Initializes SCB component to operate in one of selected configurations:
*  I2C, SPI, UART, EZI2C or EZSPI.
*  This function does not do any initialization when configuration is set to
*  Unconfigured SCB.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void GPS_Init(void)
{
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    if(GPS_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        GPS_initVar = 0u; /* Clear init var */
    }
    else
    {
        /* Initialization was done before call */
    }

#elif(GPS_SCB_MODE_I2C_CONST_CFG)
    GPS_I2CInit();

#elif(GPS_SCB_MODE_SPI_CONST_CFG)
    GPS_SpiInit();

#elif(GPS_SCB_MODE_UART_CONST_CFG)
    GPS_UartInit();

#elif(GPS_SCB_MODE_EZI2C_CONST_CFG)
    GPS_EzI2CInit();

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_Enable
********************************************************************************
*
* Summary:
*  Enables SCB component operation.
*  The SCB configuration should be not changed when the component is enabled.
*  Any configuration changes should be made after disabling the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void GPS_Enable(void)
{
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    if(!GPS_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        /* Enable SCB block, only if already configured */
        GPS_CTRL_REG |= GPS_CTRL_ENABLED;
        
        /* Enable interrupt */
        GPS_ScbEnableIntr();
    }
#else
    GPS_CTRL_REG |= GPS_CTRL_ENABLED; /* Enable SCB block */
    
    GPS_ScbEnableIntr();
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_Start
********************************************************************************
*
* Summary:
*  Invokes SCB_Init() and SCB_Enable().
*  After this function call the component is enabled and ready for operation.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  GPS_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void GPS_Start(void)
{
    if(0u == GPS_initVar)
    {
        GPS_initVar = 1u; /* Component was initialized */
        GPS_Init();       /* Initialize component      */
    }

    GPS_Enable();
}


/*******************************************************************************
* Function Name: GPS_Stop
********************************************************************************
*
* Summary:
*  Disables the SCB component.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void GPS_Stop(void)
{
#if(GPS_SCB_IRQ_INTERNAL)
    GPS_SCB_IRQ_Disable();     /* Disable interrupt before block */
#endif /* (GPS_SCB_IRQ_INTERNAL) */

    GPS_CTRL_REG &= (uint32) ~GPS_CTRL_ENABLED;  /* Disable SCB block */

#if(GPS_SCB_IRQ_INTERNAL)
    GPS_SCB_IRQ_ClearPending(); /* Clear pending interrupt */
#endif /* (GPS_SCB_IRQ_INTERNAL) */
    
    GPS_ScbModeStop(); /* Calls scbMode specific Stop function */
}


/*******************************************************************************
* Function Name: GPS_SetCustomInterruptHandler
********************************************************************************
*
* Summary:
*  Registers a function to be called by the internal interrupt handler.
*  First the function that is registered is called, then the internal interrupt
*  handler performs any operations such as software buffer management functions
*  before the interrupt returns.  It is user's responsibility to not break the
*  software buffer operations. Only one custom handler is supported, which is
*  the function provided by the most recent call.
*  At initialization time no custom handler is registered.
*
* Parameters:
*  func: Pointer to the function to register.
*        The value NULL indicates to remove the current custom interrupt
*        handler.
*
* Return:
*  None
*
*******************************************************************************/
void GPS_SetCustomInterruptHandler(cyisraddress func)
{
    GPS_customIntrHandler = func; /* Register interrupt handler */
}


/*******************************************************************************
* Function Name: GPS_ScbModeEnableIntr
********************************************************************************
*
* Summary:
*  Enables interrupt for specific mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void GPS_ScbEnableIntr(void)
{
#if(GPS_SCB_IRQ_INTERNAL)
    #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Enable interrupt source */
        if(0u != GPS_scbEnableIntr)
        {
            GPS_SCB_IRQ_Enable();
        }
    #else
        GPS_SCB_IRQ_Enable();
        
    #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (GPS_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: GPS_ScbModeEnableIntr
********************************************************************************
*
* Summary:
*  Calls Stop function for specific operation mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void GPS_ScbModeStop(void)
{
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    if(GPS_SCB_MODE_I2C_RUNTM_CFG)
    {
        GPS_I2CStop();
    }
    else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        GPS_EzI2CStop();
    }
    else
    {
        /* None of modes above */
    }
#elif(GPS_SCB_MODE_I2C_CONST_CFG)
    GPS_I2CStop();

#elif(GPS_SCB_MODE_EZI2C_CONST_CFG)
    GPS_EzI2CStop();

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: GPS_SetPins
    ********************************************************************************
    *
    * Summary:
    *  Sets pins settings accordingly to selected operation mode.
    *  Only available in Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when specific mode of operation
    *  selected in design time.
    *
    * Parameters:
    *  mode:      Mode of SCB operation.
    *  subMode:   Submode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  uartTxRx:  Direction for UART.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void GPS_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx)
    {
        uint32 hsiomSel[GPS_SCB_PINS_NUMBER];
        uint32 pinsDm  [GPS_SCB_PINS_NUMBER];
        uint32 i;

        /* Make all unused */
        for(i = 0u; i < GPS_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i] = GPS_HSIOM_DEF_SEL;
            pinsDm[i]   = GPS_PIN_DM_ALG_HIZ;
        }

        /* Choice the Dm and HSIOM */
        if((GPS_SCB_MODE_I2C   == mode) ||
           (GPS_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[GPS_MOSI_SCL_RX_PIN_INDEX] = GPS_HSIOM_I2C_SEL;
            hsiomSel[GPS_MISO_SDA_TX_PIN_INDEX] = GPS_HSIOM_I2C_SEL;

            pinsDm[GPS_MOSI_SCL_RX_PIN_INDEX] = GPS_PIN_DM_OD_LO;
            pinsDm[GPS_MISO_SDA_TX_PIN_INDEX] = GPS_PIN_DM_OD_LO;
        }
        else if(GPS_SCB_MODE_SPI == mode)
        {
            hsiomSel[GPS_MOSI_SCL_RX_PIN_INDEX] = GPS_HSIOM_SPI_SEL;
            hsiomSel[GPS_MISO_SDA_TX_PIN_INDEX] = GPS_HSIOM_SPI_SEL;
            hsiomSel[GPS_SCLK_PIN_INDEX]        = GPS_HSIOM_SPI_SEL;

            if(GPS_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[GPS_MOSI_SCL_RX_PIN_INDEX] = GPS_PIN_DM_DIG_HIZ;
                pinsDm[GPS_MISO_SDA_TX_PIN_INDEX] = GPS_PIN_DM_STRONG;
                pinsDm[GPS_SCLK_PIN_INDEX]        = GPS_PIN_DM_DIG_HIZ;

            #if(GPS_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[GPS_SS0_PIN_INDEX] = GPS_HSIOM_SPI_SEL;
                pinsDm  [GPS_SS0_PIN_INDEX] = GPS_PIN_DM_DIG_HIZ;
            #endif /* (GPS_SS1_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[GPS_MOSI_SCL_RX_PIN_INDEX] = GPS_PIN_DM_STRONG;
                pinsDm[GPS_MISO_SDA_TX_PIN_INDEX] = GPS_PIN_DM_DIG_HIZ;
                pinsDm[GPS_SCLK_PIN_INDEX]        = GPS_PIN_DM_STRONG;

            #if(GPS_SS0_PIN)
                hsiomSel[GPS_SS0_PIN_INDEX] = GPS_HSIOM_SPI_SEL;
                pinsDm  [GPS_SS0_PIN_INDEX] = GPS_PIN_DM_STRONG;
            #endif /* (GPS_SS0_PIN) */

            #if(GPS_SS1_PIN)
                hsiomSel[GPS_SS1_PIN_INDEX] = GPS_HSIOM_SPI_SEL;
                pinsDm  [GPS_SS1_PIN_INDEX] = GPS_PIN_DM_STRONG;
            #endif /* (GPS_SS1_PIN) */

            #if(GPS_SS2_PIN)
                hsiomSel[GPS_SS2_PIN_INDEX] = GPS_HSIOM_SPI_SEL;
                pinsDm  [GPS_SS2_PIN_INDEX] = GPS_PIN_DM_STRONG;
            #endif /* (GPS_SS2_PIN) */

            #if(GPS_SS3_PIN)
                hsiomSel[GPS_SS3_PIN_INDEX] = GPS_HSIOM_SPI_SEL;
                pinsDm  [GPS_SS3_PIN_INDEX] = GPS_PIN_DM_STRONG;
            #endif /* (GPS_SS2_PIN) */
            }
        }
        else /* UART */
        {
            if(GPS_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[GPS_MISO_SDA_TX_PIN_INDEX] = GPS_HSIOM_UART_SEL;
                pinsDm  [GPS_MISO_SDA_TX_PIN_INDEX] = GPS_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if(0u != (GPS_UART_RX & uartTxRx))
                {
                    hsiomSel[GPS_MOSI_SCL_RX_PIN_INDEX] = GPS_HSIOM_UART_SEL;
                    pinsDm  [GPS_MOSI_SCL_RX_PIN_INDEX] = GPS_PIN_DM_DIG_HIZ;
                }

                if(0u != (GPS_UART_TX & uartTxRx))
                {
                    hsiomSel[GPS_MISO_SDA_TX_PIN_INDEX] = GPS_HSIOM_UART_SEL;
                    pinsDm  [GPS_MISO_SDA_TX_PIN_INDEX] = GPS_PIN_DM_STRONG;
                }
            }
        }

        /* Condfigure pins: set HSIOM and DM */
        /* Condfigure pins: DR registers configuration remains unchanged for cyfitter_cfg() */

    #if(GPS_MOSI_SCL_RX_PIN)
        GPS_SET_HSIOM_SEL(GPS_MOSI_SCL_RX_HSIOM_REG,
                                       GPS_MOSI_SCL_RX_HSIOM_MASK,
                                       GPS_MOSI_SCL_RX_HSIOM_POS,
                                       hsiomSel[GPS_MOSI_SCL_RX_PIN_INDEX]);
    #endif /* (GPS_MOSI_SCL_RX_PIN) */

    #if(GPS_MOSI_SCL_RX_WAKE_PIN)
        GPS_SET_HSIOM_SEL(GPS_MOSI_SCL_RX_WAKE_HSIOM_REG,
                                       GPS_MOSI_SCL_RX_WAKE_HSIOM_MASK,
                                       GPS_MOSI_SCL_RX_WAKE_HSIOM_POS,
                                       hsiomSel[GPS_MOSI_SCL_RX_WAKE_PIN_INDEX]);
    #endif /* (GPS_MOSI_SCL_RX_WAKE_PIN) */

    #if(GPS_MISO_SDA_TX_PIN)
        GPS_SET_HSIOM_SEL(GPS_MISO_SDA_TX_HSIOM_REG,
                                       GPS_MISO_SDA_TX_HSIOM_MASK,
                                       GPS_MISO_SDA_TX_HSIOM_POS,
                                       hsiomSel[GPS_MISO_SDA_TX_PIN_INDEX]);
    #endif /* (GPS_MOSI_SCL_RX_PIN) */

    #if(GPS_SCLK_PIN)
        GPS_SET_HSIOM_SEL(GPS_SCLK_HSIOM_REG, GPS_SCLK_HSIOM_MASK,
                                       GPS_SCLK_HSIOM_POS, hsiomSel[GPS_SCLK_PIN_INDEX]);
    #endif /* (GPS_SCLK_PIN) */

    #if(GPS_SS0_PIN)
        GPS_SET_HSIOM_SEL(GPS_SS0_HSIOM_REG, GPS_SS0_HSIOM_MASK,
                                       GPS_SS0_HSIOM_POS, hsiomSel[GPS_SS0_PIN_INDEX]);
    #endif /* (GPS_SS1_PIN) */

    #if(GPS_SS1_PIN)
        GPS_SET_HSIOM_SEL(GPS_SS1_HSIOM_REG, GPS_SS1_HSIOM_MASK,
                                       GPS_SS1_HSIOM_POS, hsiomSel[GPS_SS1_PIN_INDEX]);
    #endif /* (GPS_SS1_PIN) */

    #if(GPS_SS2_PIN)
        GPS_SET_HSIOM_SEL(GPS_SS2_HSIOM_REG, GPS_SS2_HSIOM_MASK,
                                       GPS_SS2_HSIOM_POS, hsiomSel[GPS_SS2_PIN_INDEX]);
    #endif /* (GPS_SS2_PIN) */

    #if(GPS_SS3_PIN)
        GPS_SET_HSIOM_SEL(GPS_SS3_HSIOM_REG,  GPS_SS3_HSIOM_MASK,
                                       GPS_SS3_HSIOM_POS, hsiomSel[GPS_SS3_PIN_INDEX]);
    #endif /* (GPS_SS3_PIN) */



    #if(GPS_MOSI_SCL_RX_PIN)
        GPS_spi_mosi_i2c_scl_uart_rx_SetDriveMode((uint8)
                                                               pinsDm[GPS_MOSI_SCL_RX_PIN_INDEX]);
    #endif /* (GPS_MOSI_SCL_RX_PIN) */

    #if(GPS_MOSI_SCL_RX_WAKE_PIN)
    GPS_spi_mosi_i2c_scl_uart_rx_wake_SetDriveMode((uint8)
                                                               pinsDm[GPS_MOSI_SCL_RX_WAKE_PIN_INDEX]);

    /* Set interrupt on rising edge */
    GPS_SET_INCFG_TYPE(GPS_MOSI_SCL_RX_WAKE_INTCFG_REG,
                                    GPS_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK,
                                    GPS_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS,
                                    GPS_INTCFG_TYPE_FALLING_EDGE);

    #endif /* (GPS_MOSI_SCL_RX_WAKE_PIN) */

    #if(GPS_MISO_SDA_TX_PIN)
        GPS_spi_miso_i2c_sda_uart_tx_SetDriveMode((uint8)
                                                                    pinsDm[GPS_MISO_SDA_TX_PIN_INDEX]);
    #endif /* (GPS_MOSI_SCL_RX_PIN) */

    #if(GPS_SCLK_PIN)
        GPS_spi_sclk_SetDriveMode((uint8) pinsDm[GPS_SCLK_PIN_INDEX]);
    #endif /* (GPS_SCLK_PIN) */

    #if(GPS_SS0_PIN)
        GPS_spi_ss0_SetDriveMode((uint8) pinsDm[GPS_SS0_PIN_INDEX]);
    #endif /* (GPS_SS0_PIN) */

    #if(GPS_SS1_PIN)
        GPS_spi_ss1_SetDriveMode((uint8) pinsDm[GPS_SS1_PIN_INDEX]);
    #endif /* (GPS_SS1_PIN) */

    #if(GPS_SS2_PIN)
        GPS_spi_ss2_SetDriveMode((uint8) pinsDm[GPS_SS2_PIN_INDEX]);
    #endif /* (GPS_SS2_PIN) */

    #if(GPS_SS3_PIN)
        GPS_spi_ss3_SetDriveMode((uint8) pinsDm[GPS_SS3_PIN_INDEX]);
    #endif /* (GPS_SS3_PIN) */
    }

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
