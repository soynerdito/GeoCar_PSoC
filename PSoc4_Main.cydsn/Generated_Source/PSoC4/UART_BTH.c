/*******************************************************************************
* File Name: UART_BTH.c
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

#include "UART_BTH_PVT.h"

#if(UART_BTH_SCB_MODE_I2C_INC)
    #include "UART_BTH_I2C_PVT.h"
#endif /* (UART_BTH_SCB_MODE_I2C_INC) */

#if(UART_BTH_SCB_MODE_EZI2C_INC)
    #include "UART_BTH_EZI2C_PVT.h"
#endif /* (UART_BTH_SCB_MODE_EZI2C_INC) */

#if(UART_BTH_SCB_MODE_SPI_INC || UART_BTH_SCB_MODE_UART_INC)
    #include "UART_BTH_SPI_UART_PVT.h"
#endif /* (UART_BTH_SCB_MODE_SPI_INC || UART_BTH_SCB_MODE_UART_INC) */


/**********************************
*    Run Time Configuration Vars
**********************************/

/* Stores internal component configuration for unconfigured mode */
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    uint8 UART_BTH_scbMode = UART_BTH_SCB_MODE_UNCONFIG;
    uint8 UART_BTH_scbEnableWake;
    uint8 UART_BTH_scbEnableIntr;

    /* I2C config vars */
    uint8 UART_BTH_mode;
    uint8 UART_BTH_acceptAddr;

    /* SPI/UART config vars */
    volatile uint8 * UART_BTH_rxBuffer;
    uint8  UART_BTH_rxDataBits;
    uint32 UART_BTH_rxBufferSize;

    volatile uint8 * UART_BTH_txBuffer;
    uint8  UART_BTH_txDataBits;
    uint32 UART_BTH_txBufferSize;

    /* EZI2C config vars */
    uint8 UART_BTH_numberOfAddr;
    uint8 UART_BTH_subAddrSize;
#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */


/**********************************
*     Common SCB Vars
**********************************/

uint8 UART_BTH_initVar = 0u;
cyisraddress UART_BTH_customIntrHandler = NULL;


/***************************************
*    Private Function Prototypes
***************************************/

static void UART_BTH_ScbEnableIntr(void);
static void UART_BTH_ScbModeStop(void);


/*******************************************************************************
* Function Name: UART_BTH_Init
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
void UART_BTH_Init(void)
{
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    if(UART_BTH_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        UART_BTH_initVar = 0u; /* Clear init var */
    }
    else
    {
        /* Initialization was done before call */
    }

#elif(UART_BTH_SCB_MODE_I2C_CONST_CFG)
    UART_BTH_I2CInit();

#elif(UART_BTH_SCB_MODE_SPI_CONST_CFG)
    UART_BTH_SpiInit();

#elif(UART_BTH_SCB_MODE_UART_CONST_CFG)
    UART_BTH_UartInit();

#elif(UART_BTH_SCB_MODE_EZI2C_CONST_CFG)
    UART_BTH_EzI2CInit();

#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: UART_BTH_Enable
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
void UART_BTH_Enable(void)
{
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    if(!UART_BTH_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        /* Enable SCB block, only if already configured */
        UART_BTH_CTRL_REG |= UART_BTH_CTRL_ENABLED;
        
        /* Enable interrupt */
        UART_BTH_ScbEnableIntr();
    }
#else
    UART_BTH_CTRL_REG |= UART_BTH_CTRL_ENABLED; /* Enable SCB block */
    
    UART_BTH_ScbEnableIntr();
#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: UART_BTH_Start
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
*  UART_BTH_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void UART_BTH_Start(void)
{
    if(0u == UART_BTH_initVar)
    {
        UART_BTH_initVar = 1u; /* Component was initialized */
        UART_BTH_Init();       /* Initialize component      */
    }

    UART_BTH_Enable();
}


/*******************************************************************************
* Function Name: UART_BTH_Stop
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
void UART_BTH_Stop(void)
{
#if(UART_BTH_SCB_IRQ_INTERNAL)
    UART_BTH_SCB_IRQ_Disable();     /* Disable interrupt before block */
#endif /* (UART_BTH_SCB_IRQ_INTERNAL) */

    UART_BTH_CTRL_REG &= (uint32) ~UART_BTH_CTRL_ENABLED;  /* Disable SCB block */

#if(UART_BTH_SCB_IRQ_INTERNAL)
    UART_BTH_SCB_IRQ_ClearPending(); /* Clear pending interrupt */
#endif /* (UART_BTH_SCB_IRQ_INTERNAL) */
    
    UART_BTH_ScbModeStop(); /* Calls scbMode specific Stop function */
}


/*******************************************************************************
* Function Name: UART_BTH_SetCustomInterruptHandler
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
void UART_BTH_SetCustomInterruptHandler(cyisraddress func)
{
    UART_BTH_customIntrHandler = func; /* Register interrupt handler */
}


/*******************************************************************************
* Function Name: UART_BTH_ScbModeEnableIntr
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
static void UART_BTH_ScbEnableIntr(void)
{
#if(UART_BTH_SCB_IRQ_INTERNAL)
    #if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Enable interrupt source */
        if(0u != UART_BTH_scbEnableIntr)
        {
            UART_BTH_SCB_IRQ_Enable();
        }
    #else
        UART_BTH_SCB_IRQ_Enable();
        
    #endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (UART_BTH_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: UART_BTH_ScbModeEnableIntr
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
static void UART_BTH_ScbModeStop(void)
{
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    if(UART_BTH_SCB_MODE_I2C_RUNTM_CFG)
    {
        UART_BTH_I2CStop();
    }
    else if(UART_BTH_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        UART_BTH_EzI2CStop();
    }
    else
    {
        /* None of modes above */
    }
#elif(UART_BTH_SCB_MODE_I2C_CONST_CFG)
    UART_BTH_I2CStop();

#elif(UART_BTH_SCB_MODE_EZI2C_CONST_CFG)
    UART_BTH_EzI2CStop();

#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: UART_BTH_SetPins
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
    void UART_BTH_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx)
    {
        uint32 hsiomSel[UART_BTH_SCB_PINS_NUMBER];
        uint32 pinsDm  [UART_BTH_SCB_PINS_NUMBER];
        uint32 i;

        /* Make all unused */
        for(i = 0u; i < UART_BTH_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i] = UART_BTH_HSIOM_DEF_SEL;
            pinsDm[i]   = UART_BTH_PIN_DM_ALG_HIZ;
        }

        /* Choice the Dm and HSIOM */
        if((UART_BTH_SCB_MODE_I2C   == mode) ||
           (UART_BTH_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[UART_BTH_MOSI_SCL_RX_PIN_INDEX] = UART_BTH_HSIOM_I2C_SEL;
            hsiomSel[UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_HSIOM_I2C_SEL;

            pinsDm[UART_BTH_MOSI_SCL_RX_PIN_INDEX] = UART_BTH_PIN_DM_OD_LO;
            pinsDm[UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_PIN_DM_OD_LO;
        }
        else if(UART_BTH_SCB_MODE_SPI == mode)
        {
            hsiomSel[UART_BTH_MOSI_SCL_RX_PIN_INDEX] = UART_BTH_HSIOM_SPI_SEL;
            hsiomSel[UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_HSIOM_SPI_SEL;
            hsiomSel[UART_BTH_SCLK_PIN_INDEX]        = UART_BTH_HSIOM_SPI_SEL;

            if(UART_BTH_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[UART_BTH_MOSI_SCL_RX_PIN_INDEX] = UART_BTH_PIN_DM_DIG_HIZ;
                pinsDm[UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_PIN_DM_STRONG;
                pinsDm[UART_BTH_SCLK_PIN_INDEX]        = UART_BTH_PIN_DM_DIG_HIZ;

            #if(UART_BTH_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[UART_BTH_SS0_PIN_INDEX] = UART_BTH_HSIOM_SPI_SEL;
                pinsDm  [UART_BTH_SS0_PIN_INDEX] = UART_BTH_PIN_DM_DIG_HIZ;
            #endif /* (UART_BTH_SS1_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[UART_BTH_MOSI_SCL_RX_PIN_INDEX] = UART_BTH_PIN_DM_STRONG;
                pinsDm[UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_PIN_DM_DIG_HIZ;
                pinsDm[UART_BTH_SCLK_PIN_INDEX]        = UART_BTH_PIN_DM_STRONG;

            #if(UART_BTH_SS0_PIN)
                hsiomSel[UART_BTH_SS0_PIN_INDEX] = UART_BTH_HSIOM_SPI_SEL;
                pinsDm  [UART_BTH_SS0_PIN_INDEX] = UART_BTH_PIN_DM_STRONG;
            #endif /* (UART_BTH_SS0_PIN) */

            #if(UART_BTH_SS1_PIN)
                hsiomSel[UART_BTH_SS1_PIN_INDEX] = UART_BTH_HSIOM_SPI_SEL;
                pinsDm  [UART_BTH_SS1_PIN_INDEX] = UART_BTH_PIN_DM_STRONG;
            #endif /* (UART_BTH_SS1_PIN) */

            #if(UART_BTH_SS2_PIN)
                hsiomSel[UART_BTH_SS2_PIN_INDEX] = UART_BTH_HSIOM_SPI_SEL;
                pinsDm  [UART_BTH_SS2_PIN_INDEX] = UART_BTH_PIN_DM_STRONG;
            #endif /* (UART_BTH_SS2_PIN) */

            #if(UART_BTH_SS3_PIN)
                hsiomSel[UART_BTH_SS3_PIN_INDEX] = UART_BTH_HSIOM_SPI_SEL;
                pinsDm  [UART_BTH_SS3_PIN_INDEX] = UART_BTH_PIN_DM_STRONG;
            #endif /* (UART_BTH_SS2_PIN) */
            }
        }
        else /* UART */
        {
            if(UART_BTH_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_HSIOM_UART_SEL;
                pinsDm  [UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if(0u != (UART_BTH_UART_RX & uartTxRx))
                {
                    hsiomSel[UART_BTH_MOSI_SCL_RX_PIN_INDEX] = UART_BTH_HSIOM_UART_SEL;
                    pinsDm  [UART_BTH_MOSI_SCL_RX_PIN_INDEX] = UART_BTH_PIN_DM_DIG_HIZ;
                }

                if(0u != (UART_BTH_UART_TX & uartTxRx))
                {
                    hsiomSel[UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_HSIOM_UART_SEL;
                    pinsDm  [UART_BTH_MISO_SDA_TX_PIN_INDEX] = UART_BTH_PIN_DM_STRONG;
                }
            }
        }

        /* Condfigure pins: set HSIOM and DM */
        /* Condfigure pins: DR registers configuration remains unchanged for cyfitter_cfg() */

    #if(UART_BTH_MOSI_SCL_RX_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_MOSI_SCL_RX_HSIOM_REG,
                                       UART_BTH_MOSI_SCL_RX_HSIOM_MASK,
                                       UART_BTH_MOSI_SCL_RX_HSIOM_POS,
                                       hsiomSel[UART_BTH_MOSI_SCL_RX_PIN_INDEX]);
    #endif /* (UART_BTH_MOSI_SCL_RX_PIN) */

    #if(UART_BTH_MOSI_SCL_RX_WAKE_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_MOSI_SCL_RX_WAKE_HSIOM_REG,
                                       UART_BTH_MOSI_SCL_RX_WAKE_HSIOM_MASK,
                                       UART_BTH_MOSI_SCL_RX_WAKE_HSIOM_POS,
                                       hsiomSel[UART_BTH_MOSI_SCL_RX_WAKE_PIN_INDEX]);
    #endif /* (UART_BTH_MOSI_SCL_RX_WAKE_PIN) */

    #if(UART_BTH_MISO_SDA_TX_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_MISO_SDA_TX_HSIOM_REG,
                                       UART_BTH_MISO_SDA_TX_HSIOM_MASK,
                                       UART_BTH_MISO_SDA_TX_HSIOM_POS,
                                       hsiomSel[UART_BTH_MISO_SDA_TX_PIN_INDEX]);
    #endif /* (UART_BTH_MOSI_SCL_RX_PIN) */

    #if(UART_BTH_SCLK_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_SCLK_HSIOM_REG, UART_BTH_SCLK_HSIOM_MASK,
                                       UART_BTH_SCLK_HSIOM_POS, hsiomSel[UART_BTH_SCLK_PIN_INDEX]);
    #endif /* (UART_BTH_SCLK_PIN) */

    #if(UART_BTH_SS0_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_SS0_HSIOM_REG, UART_BTH_SS0_HSIOM_MASK,
                                       UART_BTH_SS0_HSIOM_POS, hsiomSel[UART_BTH_SS0_PIN_INDEX]);
    #endif /* (UART_BTH_SS1_PIN) */

    #if(UART_BTH_SS1_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_SS1_HSIOM_REG, UART_BTH_SS1_HSIOM_MASK,
                                       UART_BTH_SS1_HSIOM_POS, hsiomSel[UART_BTH_SS1_PIN_INDEX]);
    #endif /* (UART_BTH_SS1_PIN) */

    #if(UART_BTH_SS2_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_SS2_HSIOM_REG, UART_BTH_SS2_HSIOM_MASK,
                                       UART_BTH_SS2_HSIOM_POS, hsiomSel[UART_BTH_SS2_PIN_INDEX]);
    #endif /* (UART_BTH_SS2_PIN) */

    #if(UART_BTH_SS3_PIN)
        UART_BTH_SET_HSIOM_SEL(UART_BTH_SS3_HSIOM_REG,  UART_BTH_SS3_HSIOM_MASK,
                                       UART_BTH_SS3_HSIOM_POS, hsiomSel[UART_BTH_SS3_PIN_INDEX]);
    #endif /* (UART_BTH_SS3_PIN) */



    #if(UART_BTH_MOSI_SCL_RX_PIN)
        UART_BTH_spi_mosi_i2c_scl_uart_rx_SetDriveMode((uint8)
                                                               pinsDm[UART_BTH_MOSI_SCL_RX_PIN_INDEX]);
    #endif /* (UART_BTH_MOSI_SCL_RX_PIN) */

    #if(UART_BTH_MOSI_SCL_RX_WAKE_PIN)
    UART_BTH_spi_mosi_i2c_scl_uart_rx_wake_SetDriveMode((uint8)
                                                               pinsDm[UART_BTH_MOSI_SCL_RX_WAKE_PIN_INDEX]);

    /* Set interrupt on rising edge */
    UART_BTH_SET_INCFG_TYPE(UART_BTH_MOSI_SCL_RX_WAKE_INTCFG_REG,
                                    UART_BTH_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK,
                                    UART_BTH_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS,
                                    UART_BTH_INTCFG_TYPE_FALLING_EDGE);

    #endif /* (UART_BTH_MOSI_SCL_RX_WAKE_PIN) */

    #if(UART_BTH_MISO_SDA_TX_PIN)
        UART_BTH_spi_miso_i2c_sda_uart_tx_SetDriveMode((uint8)
                                                                    pinsDm[UART_BTH_MISO_SDA_TX_PIN_INDEX]);
    #endif /* (UART_BTH_MOSI_SCL_RX_PIN) */

    #if(UART_BTH_SCLK_PIN)
        UART_BTH_spi_sclk_SetDriveMode((uint8) pinsDm[UART_BTH_SCLK_PIN_INDEX]);
    #endif /* (UART_BTH_SCLK_PIN) */

    #if(UART_BTH_SS0_PIN)
        UART_BTH_spi_ss0_SetDriveMode((uint8) pinsDm[UART_BTH_SS0_PIN_INDEX]);
    #endif /* (UART_BTH_SS0_PIN) */

    #if(UART_BTH_SS1_PIN)
        UART_BTH_spi_ss1_SetDriveMode((uint8) pinsDm[UART_BTH_SS1_PIN_INDEX]);
    #endif /* (UART_BTH_SS1_PIN) */

    #if(UART_BTH_SS2_PIN)
        UART_BTH_spi_ss2_SetDriveMode((uint8) pinsDm[UART_BTH_SS2_PIN_INDEX]);
    #endif /* (UART_BTH_SS2_PIN) */

    #if(UART_BTH_SS3_PIN)
        UART_BTH_spi_ss3_SetDriveMode((uint8) pinsDm[UART_BTH_SS3_PIN_INDEX]);
    #endif /* (UART_BTH_SS3_PIN) */
    }

#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
