/*******************************************************************************
* File Name: UART_BTH_PM.c
* Version 1.10
*
* Description:
*  This file provides the source code to the Power Management support for
*  the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "UART_BTH.h"
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


/***************************************
*   Backup Structure declaration
***************************************/

UART_BTH_BACKUP_STRUCT UART_BTH_backup =
{
    0u, /* enableState */
};


/*******************************************************************************
* Function Name: UART_BTH_Sleep
********************************************************************************
*
* Summary:
*  Calls SaveConfig function fucntion for selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void UART_BTH_Sleep(void)
{
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)

    if(0u != UART_BTH_scbEnableWake)
    {
        if(UART_BTH_SCB_MODE_I2C_RUNTM_CFG)
        {
            UART_BTH_I2CSaveConfig();
        }
        else if(UART_BTH_SCB_MODE_SPI_RUNTM_CFG)
        {
            UART_BTH_SpiSaveConfig();
        }
        else if(UART_BTH_SCB_MODE_UART_RUNTM_CFG)
        {
            UART_BTH_UartSaveConfig();
        }
        else if(UART_BTH_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            UART_BTH_EzI2CSaveConfig();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {
        UART_BTH_backup.enableState = (uint8) UART_BTH_GET_CTRL_ENABLED;
        
        if(0u != UART_BTH_backup.enableState)
        {
            UART_BTH_Stop();
        }
    }
    
    UART_BTH_DisableTxPinsInputBuffer();
    
#else
    
    #if defined (UART_BTH_I2C_WAKE_ENABLE_CONST) && (UART_BTH_I2C_WAKE_ENABLE_CONST)
        UART_BTH_I2CSaveConfig();
        
    #elif defined (UART_BTH_SPI_WAKE_ENABLE_CONST) && (UART_BTH_SPI_WAKE_ENABLE_CONST)
        UART_BTH_SpiSaveConfig();
        
    #elif defined (UART_BTH_UART_WAKE_ENABLE_CONST) && (UART_BTH_UART_WAKE_ENABLE_CONST)
        UART_BTH_UartSaveConfig();
        
    #elif defined (UART_BTH_EZI2C_WAKE_ENABLE_CONST) && (UART_BTH_EZI2C_WAKE_ENABLE_CONST)
        UART_BTH_EzI2CSaveConfig();
    
    #else
        
        UART_BTH_backup.enableState = (uint8) UART_BTH_GET_CTRL_ENABLED;
        
        /* Check enable state */
        if(0u != UART_BTH_backup.enableState)
        {
            UART_BTH_Stop();
        }
        
    #endif /* defined (UART_BTH_SCB_MODE_I2C_CONST_CFG) && (UART_BTH_I2C_WAKE_ENABLE_CONST) */
    
    UART_BTH_DisableTxPinsInputBuffer();
    
#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: UART_BTH_Wakeup
********************************************************************************
*
* Summary:
*  Calls RestoreConfig function fucntion for selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void UART_BTH_Wakeup(void)
{
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)

    UART_BTH_EnableTxPinsInputBuffer();
        
    if(0u != UART_BTH_scbEnableWake)
    {
        if(UART_BTH_SCB_MODE_I2C_RUNTM_CFG)
        {
            UART_BTH_I2CRestoreConfig();
        }
        else if(UART_BTH_SCB_MODE_SPI_RUNTM_CFG)
        {
            UART_BTH_SpiRestoreConfig();
        }
        else if(UART_BTH_SCB_MODE_UART_RUNTM_CFG)
        {
            UART_BTH_UartRestoreConfig();
        }
        else if(UART_BTH_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            UART_BTH_EzI2CRestoreConfig();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {    
        /* Restore enable state */
        if(0u != UART_BTH_backup.enableState)
        {
            UART_BTH_Enable();
        }
    }

#else
    
    UART_BTH_EnableTxPinsInputBuffer();
        
    #if defined (UART_BTH_I2C_WAKE_ENABLE_CONST) && (UART_BTH_I2C_WAKE_ENABLE_CONST)
        UART_BTH_I2CRestoreConfig();
        
    #elif defined (UART_BTH_SPI_WAKE_ENABLE_CONST) && (UART_BTH_SPI_WAKE_ENABLE_CONST)
        UART_BTH_SpiRestoreConfig();
        
    #elif defined (UART_BTH_UART_WAKE_ENABLE_CONST) && (UART_BTH_UART_WAKE_ENABLE_CONST)
        UART_BTH_UartRestoreConfig();
        
    #elif defined (UART_BTH_EZI2C_WAKE_ENABLE_CONST) && (UART_BTH_EZI2C_WAKE_ENABLE_CONST)
        UART_BTH_EzI2CRestoreConfig();
    
    #else
        /* Check enable state */
        if(0u != UART_BTH_backup.enableState)
        {
            UART_BTH_Enable();
        }
        
    #endif /* (UART_BTH_I2C_WAKE_ENABLE_CONST) */

#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: UART_BTH_DisableTxPinsInputBuffer
********************************************************************************
*
* Summary:
*  Disables input buffers for TX pins. This action removes leakage current while
*  low power mode (Cypress ID 149635).
*   SCB mode is I2C and EZI2C: bus is pulled-up. Leave pins as it is.
*   SCB mode SPI:
*     Slave  - disable input buffer for MISO pin.
*     Master - disable input buffer for all pins.
*   SCB mode SmartCard: 
*     Standard and IrDA - disable input buffer for TX pin.
*     SmartCard - RX_TX pin is pulled-up. Leave pin as it is.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void UART_BTH_DisableTxPinsInputBuffer(void)
{
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    
    /* SCB mode is I2C and EZI2C: bus is pulled-up. Does nothing */
       
    if(UART_BTH_SCB_MODE_SPI_RUNTM_CFG)
    {
        if(0u != (UART_BTH_SPI_CTRL_REG & UART_BTH_SPI_CTRL_MASTER))
        /* SPI Master */
        {
        #if(UART_BTH_MOSI_SCL_RX_WAKE_PIN)
            UART_BTH_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS |= \
                                                                UART_BTH_spi_mosi_i2c_scl_uart_rx_wake_MASK;
        #endif /* (UART_BTH_MOSI_SCL_RX_WAKE_PIN) */

        #if(UART_BTH_MOSI_SCL_RX_PIN)
            UART_BTH_spi_mosi_i2c_scl_uart_rx_INP_DIS |= UART_BTH_spi_mosi_i2c_scl_uart_rx_MASK;
        #endif /* (UART_BTH_MOSI_SCL_RX_PIN) */

        #if(UART_BTH_MISO_SDA_TX_PIN)
            UART_BTH_spi_miso_i2c_sda_uart_tx_INP_DIS |= UART_BTH_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (UART_BTH_MISO_SDA_TX_PIN_PIN) */

        #if(UART_BTH_SCLK_PIN)
            UART_BTH_spi_sclk_INP_DIS |= UART_BTH_spi_sclk_MASK;
        #endif /* (UART_BTH_SCLK_PIN) */

        #if(UART_BTH_SS0_PIN)
            UART_BTH_spi_ss0_INP_DIS |= UART_BTH_spi_ss0_MASK;
        #endif /* (UART_BTH_SS1_PIN) */

        #if(UART_BTH_SS1_PIN)
            UART_BTH_spi_ss1_INP_DIS |= UART_BTH_spi_ss1_MASK;
        #endif /* (UART_BTH_SS1_PIN) */

        #if(UART_BTH_SS2_PIN)
            UART_BTH_spi_ss2_INP_DIS |= UART_BTH_spi_ss2_MASK;
        #endif /* (UART_BTH_SS2_PIN) */

        #if(UART_BTH_SS3_PIN)
            UART_BTH_spi_ss3_INP_DIS |= UART_BTH_spi_ss3_MASK;
        #endif /* (UART_BTH_SS3_PIN) */
        }
        else
        /* SPI Slave */
        {
        #if(UART_BTH_MISO_SDA_TX_PIN)
            UART_BTH_spi_miso_i2c_sda_uart_tx_INP_DIS |= UART_BTH_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (UART_BTH_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else if (UART_BTH_SCB_MODE_UART_RUNTM_CFG)
    {
        if(UART_BTH_UART_CTRL_MODE_UART_SMARTCARD != 
            (UART_BTH_UART_CTRL_REG & UART_BTH_UART_CTRL_MODE_MASK))
        /* UART Standard or IrDA */
        {
        #if(UART_BTH_MISO_SDA_TX_PIN)
            UART_BTH_spi_miso_i2c_sda_uart_tx_INP_DIS |= UART_BTH_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (UART_BTH_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else
    {
        /* Does nothing */
    }
    
#else
    
    /* SCB mode is I2C and EZI2C: bus is pulled-up. Does nothing */
        
    /* SCB mode is SPI Master */
    #if(UART_BTH_SPI_MASTER_PINS)
        UART_BTH_sclk_m_INP_DIS |= UART_BTH_sclk_m_MASK;
        UART_BTH_mosi_m_INP_DIS |= UART_BTH_mosi_m_MASK;
        UART_BTH_miso_m_INP_DIS |= UART_BTH_miso_m_MASK;
    #endif /* (UART_BTH_SPI_MASTER_PINS) */

    #if(UART_BTH_SPI_MASTER_SS0_PIN)
        UART_BTH_ss0_m_INP_DIS |= UART_BTH_ss0_m_MASK;
    #endif /* (UART_BTH_SPI_MASTER_SS0_PIN) */

    #if(UART_BTH_SPI_MASTER_SS1_PIN)
        UART_BTH_ss1_m_INP_DIS |= UART_BTH_ss1_m_MASK;
    #endif /* (UART_BTH_SPI_MASTER_SS1_PIN) */

    #if(UART_BTH_SPI_MASTER_SS2_PIN)
        UART_BTH_ss2_m_INP_DIS |= UART_BTH_ss2_m_MASK;
    #endif /* (UART_BTH_SPI_MASTER_SS2_PIN) */

    #if(UART_BTH_SPI_MASTER_SS3_PIN)
        UART_BTH_ss3_m_INP_DIS |= UART_BTH_ss3_m_MASK;
    #endif /* (UART_BTH_SPI_MASTER_SS3_PIN) */
    
    /* SCB mode is SPI Slave */
    #if(UART_BTH_SPI_SLAVE_PINS)
        UART_BTH_miso_s_INP_DIS |= UART_BTH_miso_s_MASK;
    #endif /* (UART_BTH_SPI_SLAVE_PINS) */

    /* SCB mode is UART */
    #if(UART_BTH_UART_TX_PIN)
        UART_BTH_tx_INP_DIS |= UART_BTH_tx_MASK;
    #endif /* (UART_BTH_UART_TX_PIN) */

#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: UART_BTH_EnableTxPinsInputBuffer
********************************************************************************
*
* Summary:
*  Enables input buffers for TX pins. Restore changes done byte
*UART_BTH_DisableTxPinsInputBuffer.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void UART_BTH_EnableTxPinsInputBuffer(void)
{
#if(UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG)
    if(UART_BTH_SCB_MODE_SPI_RUNTM_CFG)
    {
        if(0u != (UART_BTH_SPI_CTRL_REG & UART_BTH_SPI_CTRL_MASTER))
        /* SPI Master */
        {
        #if(UART_BTH_MOSI_SCL_RX_WAKE_PIN)
            UART_BTH_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS &= \
                                            (uint32) ~((uint32) UART_BTH_spi_mosi_i2c_scl_uart_rx_wake_MASK);
        #endif /* (UART_BTH_MOSI_SCL_RX_WAKE_PIN) */

        #if(UART_BTH_MOSI_SCL_RX_PIN)
            UART_BTH_spi_mosi_i2c_scl_uart_rx_INP_DIS &= \
                                            (uint32) ~((uint32) UART_BTH_spi_mosi_i2c_scl_uart_rx_MASK);
        #endif /* (UART_BTH_MOSI_SCL_RX_PIN) */

        #if(UART_BTH_MISO_SDA_TX_PIN)
            UART_BTH_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                            (uint32) ~((uint32) UART_BTH_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (UART_BTH_MISO_SDA_TX_PIN_PIN) */

        #if(UART_BTH_SCLK_PIN)
            UART_BTH_spi_sclk_INP_DIS &= (uint32) ~((uint32) UART_BTH_spi_sclk_MASK);
        #endif /* (UART_BTH_SCLK_PIN) */

        #if(UART_BTH_SS0_PIN)
            UART_BTH_spi_ss0_INP_DIS &= (uint32) ~((uint32) UART_BTH_spi_ss0_MASK);
        #endif /* (UART_BTH_SS1_PIN) */

        #if(UART_BTH_SS1_PIN)
            UART_BTH_spi_ss1_INP_DIS &= (uint32) ~((uint32) UART_BTH_spi_ss1_MASK);
        #endif /* (UART_BTH_SS1_PIN) */

        #if(UART_BTH_SS2_PIN)
            UART_BTH_spi_ss2_INP_DIS &= (uint32) ~((uint32) UART_BTH_spi_ss2_MASK);
        #endif /* (UART_BTH_SS2_PIN) */

        #if(UART_BTH_SS3_PIN)
            UART_BTH_spi_ss3_INP_DIS &= (uint32) ~((uint32) UART_BTH_spi_ss3_MASK);
        #endif /* (UART_BTH_SS3_PIN) */
        }
        else
        /* SPI Slave */
        {
        #if(UART_BTH_MISO_SDA_TX_PIN)
            UART_BTH_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                                (uint32) ~((uint32) UART_BTH_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (UART_BTH_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else if (UART_BTH_SCB_MODE_UART_RUNTM_CFG)
    {
        if(UART_BTH_UART_CTRL_MODE_UART_SMARTCARD != 
                (UART_BTH_UART_CTRL_REG & UART_BTH_UART_CTRL_MODE_MASK))
        /* UART Standard or IrDA */
        {
        #if(UART_BTH_MISO_SDA_TX_PIN)
            UART_BTH_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                                (uint32) ~((uint32) UART_BTH_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (UART_BTH_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else
    {
        /* Does nothing */
    }
    
#else
        
    /* SCB mode is SPI Master */
    #if(UART_BTH_SPI_MASTER_PINS)
        UART_BTH_sclk_m_INP_DIS &= (uint32) ~((uint32) UART_BTH_sclk_m_MASK);
        UART_BTH_mosi_m_INP_DIS &= (uint32) ~((uint32) UART_BTH_mosi_m_MASK);
        UART_BTH_miso_m_INP_DIS &= (uint32) ~((uint32) UART_BTH_miso_m_MASK);
    #endif /* (UART_BTH_SPI_MASTER_PINS) */

    #if(UART_BTH_SPI_MASTER_SS0_PIN)
        UART_BTH_ss0_m_INP_DIS &= (uint32) ~((uint32) UART_BTH_ss0_m_MASK);
    #endif /* (UART_BTH_SPI_MASTER_SS0_PIN) */

    #if(UART_BTH_SPI_MASTER_SS1_PIN)
        UART_BTH_ss1_m_INP_DIS &= (uint32) ~((uint32) UART_BTH_ss1_m_MASK);
    #endif /* (UART_BTH_SPI_MASTER_SS1_PIN) */

    #if(UART_BTH_SPI_MASTER_SS2_PIN)
        UART_BTH_ss2_m_INP_DIS &= (uint32) ~((uint32) UART_BTH_ss2_m_MASK);
    #endif /* (UART_BTH_SPI_MASTER_SS2_PIN) */

    #if(UART_BTH_SPI_MASTER_SS3_PIN)
        UART_BTH_ss3_m_INP_DIS &= (uint32) ~((uint32) UART_BTH_ss3_m_MASK);
    #endif /* (UART_BTH_SPI_MASTER_SS3_PIN) */
    
    /* SCB mode is SPI Slave */
    #if(UART_BTH_SPI_SLAVE_PINS)
        UART_BTH_miso_s_INP_DIS &= (uint32) ~((uint32) UART_BTH_miso_s_MASK);
    #endif /* (UART_BTH_SPI_SLAVE_PINS) */

    /* SCB mode is UART */
    #if(UART_BTH_UART_TX_PIN)
        UART_BTH_tx_INP_DIS &= (uint32) ~((uint32) UART_BTH_tx_MASK);
    #endif /* (UART_BTH_UART_TX_PIN) */

#endif /* (UART_BTH_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
