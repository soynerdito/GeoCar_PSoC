/*******************************************************************************
* File Name: PSOC5_PM.c
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

#include "PSOC5.h"
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


/***************************************
*   Backup Structure declaration
***************************************/

PSOC5_BACKUP_STRUCT PSOC5_backup =
{
    0u, /* enableState */
};


/*******************************************************************************
* Function Name: PSOC5_Sleep
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
void PSOC5_Sleep(void)
{
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)

    if(0u != PSOC5_scbEnableWake)
    {
        if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
        {
            PSOC5_I2CSaveConfig();
        }
        else if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
        {
            PSOC5_SpiSaveConfig();
        }
        else if(PSOC5_SCB_MODE_UART_RUNTM_CFG)
        {
            PSOC5_UartSaveConfig();
        }
        else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PSOC5_EzI2CSaveConfig();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {
        PSOC5_backup.enableState = (uint8) PSOC5_GET_CTRL_ENABLED;
        
        if(0u != PSOC5_backup.enableState)
        {
            PSOC5_Stop();
        }
    }
    
    PSOC5_DisableTxPinsInputBuffer();
    
#else
    
    #if defined (PSOC5_I2C_WAKE_ENABLE_CONST) && (PSOC5_I2C_WAKE_ENABLE_CONST)
        PSOC5_I2CSaveConfig();
        
    #elif defined (PSOC5_SPI_WAKE_ENABLE_CONST) && (PSOC5_SPI_WAKE_ENABLE_CONST)
        PSOC5_SpiSaveConfig();
        
    #elif defined (PSOC5_UART_WAKE_ENABLE_CONST) && (PSOC5_UART_WAKE_ENABLE_CONST)
        PSOC5_UartSaveConfig();
        
    #elif defined (PSOC5_EZI2C_WAKE_ENABLE_CONST) && (PSOC5_EZI2C_WAKE_ENABLE_CONST)
        PSOC5_EzI2CSaveConfig();
    
    #else
        
        PSOC5_backup.enableState = (uint8) PSOC5_GET_CTRL_ENABLED;
        
        /* Check enable state */
        if(0u != PSOC5_backup.enableState)
        {
            PSOC5_Stop();
        }
        
    #endif /* defined (PSOC5_SCB_MODE_I2C_CONST_CFG) && (PSOC5_I2C_WAKE_ENABLE_CONST) */
    
    PSOC5_DisableTxPinsInputBuffer();
    
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_Wakeup
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
void PSOC5_Wakeup(void)
{
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)

    PSOC5_EnableTxPinsInputBuffer();
        
    if(0u != PSOC5_scbEnableWake)
    {
        if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
        {
            PSOC5_I2CRestoreConfig();
        }
        else if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
        {
            PSOC5_SpiRestoreConfig();
        }
        else if(PSOC5_SCB_MODE_UART_RUNTM_CFG)
        {
            PSOC5_UartRestoreConfig();
        }
        else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PSOC5_EzI2CRestoreConfig();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {    
        /* Restore enable state */
        if(0u != PSOC5_backup.enableState)
        {
            PSOC5_Enable();
        }
    }

#else
    
    PSOC5_EnableTxPinsInputBuffer();
        
    #if defined (PSOC5_I2C_WAKE_ENABLE_CONST) && (PSOC5_I2C_WAKE_ENABLE_CONST)
        PSOC5_I2CRestoreConfig();
        
    #elif defined (PSOC5_SPI_WAKE_ENABLE_CONST) && (PSOC5_SPI_WAKE_ENABLE_CONST)
        PSOC5_SpiRestoreConfig();
        
    #elif defined (PSOC5_UART_WAKE_ENABLE_CONST) && (PSOC5_UART_WAKE_ENABLE_CONST)
        PSOC5_UartRestoreConfig();
        
    #elif defined (PSOC5_EZI2C_WAKE_ENABLE_CONST) && (PSOC5_EZI2C_WAKE_ENABLE_CONST)
        PSOC5_EzI2CRestoreConfig();
    
    #else
        /* Check enable state */
        if(0u != PSOC5_backup.enableState)
        {
            PSOC5_Enable();
        }
        
    #endif /* (PSOC5_I2C_WAKE_ENABLE_CONST) */

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_DisableTxPinsInputBuffer
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
void PSOC5_DisableTxPinsInputBuffer(void)
{
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    
    /* SCB mode is I2C and EZI2C: bus is pulled-up. Does nothing */
       
    if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
    {
        if(0u != (PSOC5_SPI_CTRL_REG & PSOC5_SPI_CTRL_MASTER))
        /* SPI Master */
        {
        #if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
            PSOC5_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS |= \
                                                                PSOC5_spi_mosi_i2c_scl_uart_rx_wake_MASK;
        #endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */

        #if(PSOC5_MOSI_SCL_RX_PIN)
            PSOC5_spi_mosi_i2c_scl_uart_rx_INP_DIS |= PSOC5_spi_mosi_i2c_scl_uart_rx_MASK;
        #endif /* (PSOC5_MOSI_SCL_RX_PIN) */

        #if(PSOC5_MISO_SDA_TX_PIN)
            PSOC5_spi_miso_i2c_sda_uart_tx_INP_DIS |= PSOC5_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */

        #if(PSOC5_SCLK_PIN)
            PSOC5_spi_sclk_INP_DIS |= PSOC5_spi_sclk_MASK;
        #endif /* (PSOC5_SCLK_PIN) */

        #if(PSOC5_SS0_PIN)
            PSOC5_spi_ss0_INP_DIS |= PSOC5_spi_ss0_MASK;
        #endif /* (PSOC5_SS1_PIN) */

        #if(PSOC5_SS1_PIN)
            PSOC5_spi_ss1_INP_DIS |= PSOC5_spi_ss1_MASK;
        #endif /* (PSOC5_SS1_PIN) */

        #if(PSOC5_SS2_PIN)
            PSOC5_spi_ss2_INP_DIS |= PSOC5_spi_ss2_MASK;
        #endif /* (PSOC5_SS2_PIN) */

        #if(PSOC5_SS3_PIN)
            PSOC5_spi_ss3_INP_DIS |= PSOC5_spi_ss3_MASK;
        #endif /* (PSOC5_SS3_PIN) */
        }
        else
        /* SPI Slave */
        {
        #if(PSOC5_MISO_SDA_TX_PIN)
            PSOC5_spi_miso_i2c_sda_uart_tx_INP_DIS |= PSOC5_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else if (PSOC5_SCB_MODE_UART_RUNTM_CFG)
    {
        if(PSOC5_UART_CTRL_MODE_UART_SMARTCARD != 
            (PSOC5_UART_CTRL_REG & PSOC5_UART_CTRL_MODE_MASK))
        /* UART Standard or IrDA */
        {
        #if(PSOC5_MISO_SDA_TX_PIN)
            PSOC5_spi_miso_i2c_sda_uart_tx_INP_DIS |= PSOC5_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else
    {
        /* Does nothing */
    }
    
#else
    
    /* SCB mode is I2C and EZI2C: bus is pulled-up. Does nothing */
        
    /* SCB mode is SPI Master */
    #if(PSOC5_SPI_MASTER_PINS)
        PSOC5_sclk_m_INP_DIS |= PSOC5_sclk_m_MASK;
        PSOC5_mosi_m_INP_DIS |= PSOC5_mosi_m_MASK;
        PSOC5_miso_m_INP_DIS |= PSOC5_miso_m_MASK;
    #endif /* (PSOC5_SPI_MASTER_PINS) */

    #if(PSOC5_SPI_MASTER_SS0_PIN)
        PSOC5_ss0_m_INP_DIS |= PSOC5_ss0_m_MASK;
    #endif /* (PSOC5_SPI_MASTER_SS0_PIN) */

    #if(PSOC5_SPI_MASTER_SS1_PIN)
        PSOC5_ss1_m_INP_DIS |= PSOC5_ss1_m_MASK;
    #endif /* (PSOC5_SPI_MASTER_SS1_PIN) */

    #if(PSOC5_SPI_MASTER_SS2_PIN)
        PSOC5_ss2_m_INP_DIS |= PSOC5_ss2_m_MASK;
    #endif /* (PSOC5_SPI_MASTER_SS2_PIN) */

    #if(PSOC5_SPI_MASTER_SS3_PIN)
        PSOC5_ss3_m_INP_DIS |= PSOC5_ss3_m_MASK;
    #endif /* (PSOC5_SPI_MASTER_SS3_PIN) */
    
    /* SCB mode is SPI Slave */
    #if(PSOC5_SPI_SLAVE_PINS)
        PSOC5_miso_s_INP_DIS |= PSOC5_miso_s_MASK;
    #endif /* (PSOC5_SPI_SLAVE_PINS) */

    /* SCB mode is UART */
    #if(PSOC5_UART_TX_PIN)
        PSOC5_tx_INP_DIS |= PSOC5_tx_MASK;
    #endif /* (PSOC5_UART_TX_PIN) */

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_EnableTxPinsInputBuffer
********************************************************************************
*
* Summary:
*  Enables input buffers for TX pins. Restore changes done byte
*PSOC5_DisableTxPinsInputBuffer.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PSOC5_EnableTxPinsInputBuffer(void)
{
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
    {
        if(0u != (PSOC5_SPI_CTRL_REG & PSOC5_SPI_CTRL_MASTER))
        /* SPI Master */
        {
        #if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
            PSOC5_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS &= \
                                            (uint32) ~((uint32) PSOC5_spi_mosi_i2c_scl_uart_rx_wake_MASK);
        #endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */

        #if(PSOC5_MOSI_SCL_RX_PIN)
            PSOC5_spi_mosi_i2c_scl_uart_rx_INP_DIS &= \
                                            (uint32) ~((uint32) PSOC5_spi_mosi_i2c_scl_uart_rx_MASK);
        #endif /* (PSOC5_MOSI_SCL_RX_PIN) */

        #if(PSOC5_MISO_SDA_TX_PIN)
            PSOC5_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                            (uint32) ~((uint32) PSOC5_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */

        #if(PSOC5_SCLK_PIN)
            PSOC5_spi_sclk_INP_DIS &= (uint32) ~((uint32) PSOC5_spi_sclk_MASK);
        #endif /* (PSOC5_SCLK_PIN) */

        #if(PSOC5_SS0_PIN)
            PSOC5_spi_ss0_INP_DIS &= (uint32) ~((uint32) PSOC5_spi_ss0_MASK);
        #endif /* (PSOC5_SS1_PIN) */

        #if(PSOC5_SS1_PIN)
            PSOC5_spi_ss1_INP_DIS &= (uint32) ~((uint32) PSOC5_spi_ss1_MASK);
        #endif /* (PSOC5_SS1_PIN) */

        #if(PSOC5_SS2_PIN)
            PSOC5_spi_ss2_INP_DIS &= (uint32) ~((uint32) PSOC5_spi_ss2_MASK);
        #endif /* (PSOC5_SS2_PIN) */

        #if(PSOC5_SS3_PIN)
            PSOC5_spi_ss3_INP_DIS &= (uint32) ~((uint32) PSOC5_spi_ss3_MASK);
        #endif /* (PSOC5_SS3_PIN) */
        }
        else
        /* SPI Slave */
        {
        #if(PSOC5_MISO_SDA_TX_PIN)
            PSOC5_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                                (uint32) ~((uint32) PSOC5_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else if (PSOC5_SCB_MODE_UART_RUNTM_CFG)
    {
        if(PSOC5_UART_CTRL_MODE_UART_SMARTCARD != 
                (PSOC5_UART_CTRL_REG & PSOC5_UART_CTRL_MODE_MASK))
        /* UART Standard or IrDA */
        {
        #if(PSOC5_MISO_SDA_TX_PIN)
            PSOC5_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                                (uint32) ~((uint32) PSOC5_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else
    {
        /* Does nothing */
    }
    
#else
        
    /* SCB mode is SPI Master */
    #if(PSOC5_SPI_MASTER_PINS)
        PSOC5_sclk_m_INP_DIS &= (uint32) ~((uint32) PSOC5_sclk_m_MASK);
        PSOC5_mosi_m_INP_DIS &= (uint32) ~((uint32) PSOC5_mosi_m_MASK);
        PSOC5_miso_m_INP_DIS &= (uint32) ~((uint32) PSOC5_miso_m_MASK);
    #endif /* (PSOC5_SPI_MASTER_PINS) */

    #if(PSOC5_SPI_MASTER_SS0_PIN)
        PSOC5_ss0_m_INP_DIS &= (uint32) ~((uint32) PSOC5_ss0_m_MASK);
    #endif /* (PSOC5_SPI_MASTER_SS0_PIN) */

    #if(PSOC5_SPI_MASTER_SS1_PIN)
        PSOC5_ss1_m_INP_DIS &= (uint32) ~((uint32) PSOC5_ss1_m_MASK);
    #endif /* (PSOC5_SPI_MASTER_SS1_PIN) */

    #if(PSOC5_SPI_MASTER_SS2_PIN)
        PSOC5_ss2_m_INP_DIS &= (uint32) ~((uint32) PSOC5_ss2_m_MASK);
    #endif /* (PSOC5_SPI_MASTER_SS2_PIN) */

    #if(PSOC5_SPI_MASTER_SS3_PIN)
        PSOC5_ss3_m_INP_DIS &= (uint32) ~((uint32) PSOC5_ss3_m_MASK);
    #endif /* (PSOC5_SPI_MASTER_SS3_PIN) */
    
    /* SCB mode is SPI Slave */
    #if(PSOC5_SPI_SLAVE_PINS)
        PSOC5_miso_s_INP_DIS &= (uint32) ~((uint32) PSOC5_miso_s_MASK);
    #endif /* (PSOC5_SPI_SLAVE_PINS) */

    /* SCB mode is UART */
    #if(PSOC5_UART_TX_PIN)
        PSOC5_tx_INP_DIS &= (uint32) ~((uint32) PSOC5_tx_MASK);
    #endif /* (PSOC5_UART_TX_PIN) */

#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
