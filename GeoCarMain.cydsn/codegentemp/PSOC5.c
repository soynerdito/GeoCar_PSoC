/*******************************************************************************
* File Name: PSOC5.c
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

#include "PSOC5_PVT.h"

#if(PSOC5_SCB_MODE_I2C_INC)
    #include "PSOC5_I2C_PVT.h"
#endif /* (PSOC5_SCB_MODE_I2C_INC) */

#if(PSOC5_SCB_MODE_EZI2C_INC)
    #include "PSOC5_EZI2C_PVT.h"
#endif /* (PSOC5_SCB_MODE_EZI2C_INC) */

#if(PSOC5_SCB_MODE_SPI_INC || PSOC5_SCB_MODE_UART_INC)
    #include "PSOC5_SPI_UART_PVT.h"
#endif /* (PSOC5_SCB_MODE_SPI_INC || PSOC5_SCB_MODE_UART_INC) */


/**********************************
*    Run Time Configuration Vars
**********************************/

/* Stores internal component configuration for unconfigured mode */
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    uint8 PSOC5_scbMode = PSOC5_SCB_MODE_UNCONFIG;
    uint8 PSOC5_scbEnableWake;
    uint8 PSOC5_scbEnableIntr;

    /* I2C config vars */
    uint8 PSOC5_mode;
    uint8 PSOC5_acceptAddr;

    /* SPI/UART config vars */
    volatile uint8 * PSOC5_rxBuffer;
    uint8  PSOC5_rxDataBits;
    uint32 PSOC5_rxBufferSize;

    volatile uint8 * PSOC5_txBuffer;
    uint8  PSOC5_txDataBits;
    uint32 PSOC5_txBufferSize;

    /* EZI2C config vars */
    uint8 PSOC5_numberOfAddr;
    uint8 PSOC5_subAddrSize;
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */


/**********************************
*     Common SCB Vars
**********************************/

uint8 PSOC5_initVar = 0u;
cyisraddress PSOC5_customIntrHandler = NULL;


/***************************************
*    Private Function Prototypes
***************************************/

static void PSOC5_ScbEnableIntr(void);
static void PSOC5_ScbModeStop(void);


/*******************************************************************************
* Function Name: PSOC5_Init
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
void PSOC5_Init(void)
{
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    if(PSOC5_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        PSOC5_initVar = 0u; /* Clear init var */
    }
    else
    {
        /* Initialization was done before call */
    }

#elif(PSOC5_SCB_MODE_I2C_CONST_CFG)
    PSOC5_I2CInit();

#elif(PSOC5_SCB_MODE_SPI_CONST_CFG)
    PSOC5_SpiInit();

#elif(PSOC5_SCB_MODE_UART_CONST_CFG)
    PSOC5_UartInit();

#elif(PSOC5_SCB_MODE_EZI2C_CONST_CFG)
    PSOC5_EzI2CInit();

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_Enable
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
void PSOC5_Enable(void)
{
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    if(!PSOC5_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        /* Enable SCB block, only if already configured */
        PSOC5_CTRL_REG |= PSOC5_CTRL_ENABLED;
        
        /* Enable interrupt */
        PSOC5_ScbEnableIntr();
    }
#else
    PSOC5_CTRL_REG |= PSOC5_CTRL_ENABLED; /* Enable SCB block */
    
    PSOC5_ScbEnableIntr();
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_Start
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
*  PSOC5_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void PSOC5_Start(void)
{
    if(0u == PSOC5_initVar)
    {
        PSOC5_initVar = 1u; /* Component was initialized */
        PSOC5_Init();       /* Initialize component      */
    }

    PSOC5_Enable();
}


/*******************************************************************************
* Function Name: PSOC5_Stop
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
void PSOC5_Stop(void)
{
#if(PSOC5_SCB_IRQ_INTERNAL)
    PSOC5_SCB_IRQ_Disable();     /* Disable interrupt before block */
#endif /* (PSOC5_SCB_IRQ_INTERNAL) */

    PSOC5_CTRL_REG &= (uint32) ~PSOC5_CTRL_ENABLED;  /* Disable SCB block */

#if(PSOC5_SCB_IRQ_INTERNAL)
    PSOC5_SCB_IRQ_ClearPending(); /* Clear pending interrupt */
#endif /* (PSOC5_SCB_IRQ_INTERNAL) */
    
    PSOC5_ScbModeStop(); /* Calls scbMode specific Stop function */
}


/*******************************************************************************
* Function Name: PSOC5_SetCustomInterruptHandler
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
void PSOC5_SetCustomInterruptHandler(cyisraddress func)
{
    PSOC5_customIntrHandler = func; /* Register interrupt handler */
}


/*******************************************************************************
* Function Name: PSOC5_ScbModeEnableIntr
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
static void PSOC5_ScbEnableIntr(void)
{
#if(PSOC5_SCB_IRQ_INTERNAL)
    #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Enable interrupt source */
        if(0u != PSOC5_scbEnableIntr)
        {
            PSOC5_SCB_IRQ_Enable();
        }
    #else
        PSOC5_SCB_IRQ_Enable();
        
    #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (PSOC5_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: PSOC5_ScbModeEnableIntr
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
static void PSOC5_ScbModeStop(void)
{
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
    {
        PSOC5_I2CStop();
    }
    else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        PSOC5_EzI2CStop();
    }
    else
    {
        /* None of modes above */
    }
#elif(PSOC5_SCB_MODE_I2C_CONST_CFG)
    PSOC5_I2CStop();

#elif(PSOC5_SCB_MODE_EZI2C_CONST_CFG)
    PSOC5_EzI2CStop();

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: PSOC5_SetPins
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
    void PSOC5_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx)
    {
        uint32 hsiomSel[PSOC5_SCB_PINS_NUMBER];
        uint32 pinsDm  [PSOC5_SCB_PINS_NUMBER];
        uint32 i;

        /* Make all unused */
        for(i = 0u; i < PSOC5_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i] = PSOC5_HSIOM_DEF_SEL;
            pinsDm[i]   = PSOC5_PIN_DM_ALG_HIZ;
        }

        /* Choice the Dm and HSIOM */
        if((PSOC5_SCB_MODE_I2C   == mode) ||
           (PSOC5_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[PSOC5_MOSI_SCL_RX_PIN_INDEX] = PSOC5_HSIOM_I2C_SEL;
            hsiomSel[PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_HSIOM_I2C_SEL;

            pinsDm[PSOC5_MOSI_SCL_RX_PIN_INDEX] = PSOC5_PIN_DM_OD_LO;
            pinsDm[PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_PIN_DM_OD_LO;
        }
        else if(PSOC5_SCB_MODE_SPI == mode)
        {
            hsiomSel[PSOC5_MOSI_SCL_RX_PIN_INDEX] = PSOC5_HSIOM_SPI_SEL;
            hsiomSel[PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_HSIOM_SPI_SEL;
            hsiomSel[PSOC5_SCLK_PIN_INDEX]        = PSOC5_HSIOM_SPI_SEL;

            if(PSOC5_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[PSOC5_MOSI_SCL_RX_PIN_INDEX] = PSOC5_PIN_DM_DIG_HIZ;
                pinsDm[PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_PIN_DM_STRONG;
                pinsDm[PSOC5_SCLK_PIN_INDEX]        = PSOC5_PIN_DM_DIG_HIZ;

            #if(PSOC5_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[PSOC5_SS0_PIN_INDEX] = PSOC5_HSIOM_SPI_SEL;
                pinsDm  [PSOC5_SS0_PIN_INDEX] = PSOC5_PIN_DM_DIG_HIZ;
            #endif /* (PSOC5_SS1_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[PSOC5_MOSI_SCL_RX_PIN_INDEX] = PSOC5_PIN_DM_STRONG;
                pinsDm[PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_PIN_DM_DIG_HIZ;
                pinsDm[PSOC5_SCLK_PIN_INDEX]        = PSOC5_PIN_DM_STRONG;

            #if(PSOC5_SS0_PIN)
                hsiomSel[PSOC5_SS0_PIN_INDEX] = PSOC5_HSIOM_SPI_SEL;
                pinsDm  [PSOC5_SS0_PIN_INDEX] = PSOC5_PIN_DM_STRONG;
            #endif /* (PSOC5_SS0_PIN) */

            #if(PSOC5_SS1_PIN)
                hsiomSel[PSOC5_SS1_PIN_INDEX] = PSOC5_HSIOM_SPI_SEL;
                pinsDm  [PSOC5_SS1_PIN_INDEX] = PSOC5_PIN_DM_STRONG;
            #endif /* (PSOC5_SS1_PIN) */

            #if(PSOC5_SS2_PIN)
                hsiomSel[PSOC5_SS2_PIN_INDEX] = PSOC5_HSIOM_SPI_SEL;
                pinsDm  [PSOC5_SS2_PIN_INDEX] = PSOC5_PIN_DM_STRONG;
            #endif /* (PSOC5_SS2_PIN) */

            #if(PSOC5_SS3_PIN)
                hsiomSel[PSOC5_SS3_PIN_INDEX] = PSOC5_HSIOM_SPI_SEL;
                pinsDm  [PSOC5_SS3_PIN_INDEX] = PSOC5_PIN_DM_STRONG;
            #endif /* (PSOC5_SS2_PIN) */
            }
        }
        else /* UART */
        {
            if(PSOC5_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_HSIOM_UART_SEL;
                pinsDm  [PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if(0u != (PSOC5_UART_RX & uartTxRx))
                {
                    hsiomSel[PSOC5_MOSI_SCL_RX_PIN_INDEX] = PSOC5_HSIOM_UART_SEL;
                    pinsDm  [PSOC5_MOSI_SCL_RX_PIN_INDEX] = PSOC5_PIN_DM_DIG_HIZ;
                }

                if(0u != (PSOC5_UART_TX & uartTxRx))
                {
                    hsiomSel[PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_HSIOM_UART_SEL;
                    pinsDm  [PSOC5_MISO_SDA_TX_PIN_INDEX] = PSOC5_PIN_DM_STRONG;
                }
            }
        }

        /* Condfigure pins: set HSIOM and DM */
        /* Condfigure pins: DR registers configuration remains unchanged for cyfitter_cfg() */

    #if(PSOC5_MOSI_SCL_RX_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_MOSI_SCL_RX_HSIOM_REG,
                                       PSOC5_MOSI_SCL_RX_HSIOM_MASK,
                                       PSOC5_MOSI_SCL_RX_HSIOM_POS,
                                       hsiomSel[PSOC5_MOSI_SCL_RX_PIN_INDEX]);
    #endif /* (PSOC5_MOSI_SCL_RX_PIN) */

    #if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_MOSI_SCL_RX_WAKE_HSIOM_REG,
                                       PSOC5_MOSI_SCL_RX_WAKE_HSIOM_MASK,
                                       PSOC5_MOSI_SCL_RX_WAKE_HSIOM_POS,
                                       hsiomSel[PSOC5_MOSI_SCL_RX_WAKE_PIN_INDEX]);
    #endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */

    #if(PSOC5_MISO_SDA_TX_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_MISO_SDA_TX_HSIOM_REG,
                                       PSOC5_MISO_SDA_TX_HSIOM_MASK,
                                       PSOC5_MISO_SDA_TX_HSIOM_POS,
                                       hsiomSel[PSOC5_MISO_SDA_TX_PIN_INDEX]);
    #endif /* (PSOC5_MOSI_SCL_RX_PIN) */

    #if(PSOC5_SCLK_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_SCLK_HSIOM_REG, PSOC5_SCLK_HSIOM_MASK,
                                       PSOC5_SCLK_HSIOM_POS, hsiomSel[PSOC5_SCLK_PIN_INDEX]);
    #endif /* (PSOC5_SCLK_PIN) */

    #if(PSOC5_SS0_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_SS0_HSIOM_REG, PSOC5_SS0_HSIOM_MASK,
                                       PSOC5_SS0_HSIOM_POS, hsiomSel[PSOC5_SS0_PIN_INDEX]);
    #endif /* (PSOC5_SS1_PIN) */

    #if(PSOC5_SS1_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_SS1_HSIOM_REG, PSOC5_SS1_HSIOM_MASK,
                                       PSOC5_SS1_HSIOM_POS, hsiomSel[PSOC5_SS1_PIN_INDEX]);
    #endif /* (PSOC5_SS1_PIN) */

    #if(PSOC5_SS2_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_SS2_HSIOM_REG, PSOC5_SS2_HSIOM_MASK,
                                       PSOC5_SS2_HSIOM_POS, hsiomSel[PSOC5_SS2_PIN_INDEX]);
    #endif /* (PSOC5_SS2_PIN) */

    #if(PSOC5_SS3_PIN)
        PSOC5_SET_HSIOM_SEL(PSOC5_SS3_HSIOM_REG,  PSOC5_SS3_HSIOM_MASK,
                                       PSOC5_SS3_HSIOM_POS, hsiomSel[PSOC5_SS3_PIN_INDEX]);
    #endif /* (PSOC5_SS3_PIN) */



    #if(PSOC5_MOSI_SCL_RX_PIN)
        PSOC5_spi_mosi_i2c_scl_uart_rx_SetDriveMode((uint8)
                                                               pinsDm[PSOC5_MOSI_SCL_RX_PIN_INDEX]);
    #endif /* (PSOC5_MOSI_SCL_RX_PIN) */

    #if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
    PSOC5_spi_mosi_i2c_scl_uart_rx_wake_SetDriveMode((uint8)
                                                               pinsDm[PSOC5_MOSI_SCL_RX_WAKE_PIN_INDEX]);

    /* Set interrupt on rising edge */
    PSOC5_SET_INCFG_TYPE(PSOC5_MOSI_SCL_RX_WAKE_INTCFG_REG,
                                    PSOC5_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK,
                                    PSOC5_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS,
                                    PSOC5_INTCFG_TYPE_FALLING_EDGE);

    #endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */

    #if(PSOC5_MISO_SDA_TX_PIN)
        PSOC5_spi_miso_i2c_sda_uart_tx_SetDriveMode((uint8)
                                                                    pinsDm[PSOC5_MISO_SDA_TX_PIN_INDEX]);
    #endif /* (PSOC5_MOSI_SCL_RX_PIN) */

    #if(PSOC5_SCLK_PIN)
        PSOC5_spi_sclk_SetDriveMode((uint8) pinsDm[PSOC5_SCLK_PIN_INDEX]);
    #endif /* (PSOC5_SCLK_PIN) */

    #if(PSOC5_SS0_PIN)
        PSOC5_spi_ss0_SetDriveMode((uint8) pinsDm[PSOC5_SS0_PIN_INDEX]);
    #endif /* (PSOC5_SS0_PIN) */

    #if(PSOC5_SS1_PIN)
        PSOC5_spi_ss1_SetDriveMode((uint8) pinsDm[PSOC5_SS1_PIN_INDEX]);
    #endif /* (PSOC5_SS1_PIN) */

    #if(PSOC5_SS2_PIN)
        PSOC5_spi_ss2_SetDriveMode((uint8) pinsDm[PSOC5_SS2_PIN_INDEX]);
    #endif /* (PSOC5_SS2_PIN) */

    #if(PSOC5_SS3_PIN)
        PSOC5_spi_ss3_SetDriveMode((uint8) pinsDm[PSOC5_SS3_PIN_INDEX]);
    #endif /* (PSOC5_SS3_PIN) */
    }

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
