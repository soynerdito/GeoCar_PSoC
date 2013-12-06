/*******************************************************************************
* File Name: GPS_PM.c
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

#include "GPS.h"
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


/***************************************
*   Backup Structure declaration
***************************************/

GPS_BACKUP_STRUCT GPS_backup =
{
    0u, /* enableState */
};


/*******************************************************************************
* Function Name: GPS_Sleep
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
void GPS_Sleep(void)
{
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)

    if(0u != GPS_scbEnableWake)
    {
        if(GPS_SCB_MODE_I2C_RUNTM_CFG)
        {
            GPS_I2CSaveConfig();
        }
        else if(GPS_SCB_MODE_SPI_RUNTM_CFG)
        {
            GPS_SpiSaveConfig();
        }
        else if(GPS_SCB_MODE_UART_RUNTM_CFG)
        {
            GPS_UartSaveConfig();
        }
        else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            GPS_EzI2CSaveConfig();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {
        GPS_backup.enableState = (uint8) GPS_GET_CTRL_ENABLED;
        
        if(0u != GPS_backup.enableState)
        {
            GPS_Stop();
        }
    }
    
    GPS_DisableTxPinsInputBuffer();
    
#else
    
    #if defined (GPS_I2C_WAKE_ENABLE_CONST) && (GPS_I2C_WAKE_ENABLE_CONST)
        GPS_I2CSaveConfig();
        
    #elif defined (GPS_SPI_WAKE_ENABLE_CONST) && (GPS_SPI_WAKE_ENABLE_CONST)
        GPS_SpiSaveConfig();
        
    #elif defined (GPS_UART_WAKE_ENABLE_CONST) && (GPS_UART_WAKE_ENABLE_CONST)
        GPS_UartSaveConfig();
        
    #elif defined (GPS_EZI2C_WAKE_ENABLE_CONST) && (GPS_EZI2C_WAKE_ENABLE_CONST)
        GPS_EzI2CSaveConfig();
    
    #else
        
        GPS_backup.enableState = (uint8) GPS_GET_CTRL_ENABLED;
        
        /* Check enable state */
        if(0u != GPS_backup.enableState)
        {
            GPS_Stop();
        }
        
    #endif /* defined (GPS_SCB_MODE_I2C_CONST_CFG) && (GPS_I2C_WAKE_ENABLE_CONST) */
    
    GPS_DisableTxPinsInputBuffer();
    
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_Wakeup
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
void GPS_Wakeup(void)
{
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)

    GPS_EnableTxPinsInputBuffer();
        
    if(0u != GPS_scbEnableWake)
    {
        if(GPS_SCB_MODE_I2C_RUNTM_CFG)
        {
            GPS_I2CRestoreConfig();
        }
        else if(GPS_SCB_MODE_SPI_RUNTM_CFG)
        {
            GPS_SpiRestoreConfig();
        }
        else if(GPS_SCB_MODE_UART_RUNTM_CFG)
        {
            GPS_UartRestoreConfig();
        }
        else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            GPS_EzI2CRestoreConfig();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {    
        /* Restore enable state */
        if(0u != GPS_backup.enableState)
        {
            GPS_Enable();
        }
    }

#else
    
    GPS_EnableTxPinsInputBuffer();
        
    #if defined (GPS_I2C_WAKE_ENABLE_CONST) && (GPS_I2C_WAKE_ENABLE_CONST)
        GPS_I2CRestoreConfig();
        
    #elif defined (GPS_SPI_WAKE_ENABLE_CONST) && (GPS_SPI_WAKE_ENABLE_CONST)
        GPS_SpiRestoreConfig();
        
    #elif defined (GPS_UART_WAKE_ENABLE_CONST) && (GPS_UART_WAKE_ENABLE_CONST)
        GPS_UartRestoreConfig();
        
    #elif defined (GPS_EZI2C_WAKE_ENABLE_CONST) && (GPS_EZI2C_WAKE_ENABLE_CONST)
        GPS_EzI2CRestoreConfig();
    
    #else
        /* Check enable state */
        if(0u != GPS_backup.enableState)
        {
            GPS_Enable();
        }
        
    #endif /* (GPS_I2C_WAKE_ENABLE_CONST) */

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_DisableTxPinsInputBuffer
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
void GPS_DisableTxPinsInputBuffer(void)
{
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    
    /* SCB mode is I2C and EZI2C: bus is pulled-up. Does nothing */
       
    if(GPS_SCB_MODE_SPI_RUNTM_CFG)
    {
        if(0u != (GPS_SPI_CTRL_REG & GPS_SPI_CTRL_MASTER))
        /* SPI Master */
        {
        #if(GPS_MOSI_SCL_RX_WAKE_PIN)
            GPS_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS |= \
                                                                GPS_spi_mosi_i2c_scl_uart_rx_wake_MASK;
        #endif /* (GPS_MOSI_SCL_RX_WAKE_PIN) */

        #if(GPS_MOSI_SCL_RX_PIN)
            GPS_spi_mosi_i2c_scl_uart_rx_INP_DIS |= GPS_spi_mosi_i2c_scl_uart_rx_MASK;
        #endif /* (GPS_MOSI_SCL_RX_PIN) */

        #if(GPS_MISO_SDA_TX_PIN)
            GPS_spi_miso_i2c_sda_uart_tx_INP_DIS |= GPS_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (GPS_MISO_SDA_TX_PIN_PIN) */

        #if(GPS_SCLK_PIN)
            GPS_spi_sclk_INP_DIS |= GPS_spi_sclk_MASK;
        #endif /* (GPS_SCLK_PIN) */

        #if(GPS_SS0_PIN)
            GPS_spi_ss0_INP_DIS |= GPS_spi_ss0_MASK;
        #endif /* (GPS_SS1_PIN) */

        #if(GPS_SS1_PIN)
            GPS_spi_ss1_INP_DIS |= GPS_spi_ss1_MASK;
        #endif /* (GPS_SS1_PIN) */

        #if(GPS_SS2_PIN)
            GPS_spi_ss2_INP_DIS |= GPS_spi_ss2_MASK;
        #endif /* (GPS_SS2_PIN) */

        #if(GPS_SS3_PIN)
            GPS_spi_ss3_INP_DIS |= GPS_spi_ss3_MASK;
        #endif /* (GPS_SS3_PIN) */
        }
        else
        /* SPI Slave */
        {
        #if(GPS_MISO_SDA_TX_PIN)
            GPS_spi_miso_i2c_sda_uart_tx_INP_DIS |= GPS_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (GPS_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else if (GPS_SCB_MODE_UART_RUNTM_CFG)
    {
        if(GPS_UART_CTRL_MODE_UART_SMARTCARD != 
            (GPS_UART_CTRL_REG & GPS_UART_CTRL_MODE_MASK))
        /* UART Standard or IrDA */
        {
        #if(GPS_MISO_SDA_TX_PIN)
            GPS_spi_miso_i2c_sda_uart_tx_INP_DIS |= GPS_spi_miso_i2c_sda_uart_tx_MASK;
        #endif /* (GPS_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else
    {
        /* Does nothing */
    }
    
#else
    
    /* SCB mode is I2C and EZI2C: bus is pulled-up. Does nothing */
        
    /* SCB mode is SPI Master */
    #if(GPS_SPI_MASTER_PINS)
        GPS_sclk_m_INP_DIS |= GPS_sclk_m_MASK;
        GPS_mosi_m_INP_DIS |= GPS_mosi_m_MASK;
        GPS_miso_m_INP_DIS |= GPS_miso_m_MASK;
    #endif /* (GPS_SPI_MASTER_PINS) */

    #if(GPS_SPI_MASTER_SS0_PIN)
        GPS_ss0_m_INP_DIS |= GPS_ss0_m_MASK;
    #endif /* (GPS_SPI_MASTER_SS0_PIN) */

    #if(GPS_SPI_MASTER_SS1_PIN)
        GPS_ss1_m_INP_DIS |= GPS_ss1_m_MASK;
    #endif /* (GPS_SPI_MASTER_SS1_PIN) */

    #if(GPS_SPI_MASTER_SS2_PIN)
        GPS_ss2_m_INP_DIS |= GPS_ss2_m_MASK;
    #endif /* (GPS_SPI_MASTER_SS2_PIN) */

    #if(GPS_SPI_MASTER_SS3_PIN)
        GPS_ss3_m_INP_DIS |= GPS_ss3_m_MASK;
    #endif /* (GPS_SPI_MASTER_SS3_PIN) */
    
    /* SCB mode is SPI Slave */
    #if(GPS_SPI_SLAVE_PINS)
        GPS_miso_s_INP_DIS |= GPS_miso_s_MASK;
    #endif /* (GPS_SPI_SLAVE_PINS) */

    /* SCB mode is UART */
    #if(GPS_UART_TX_PIN)
        GPS_tx_INP_DIS |= GPS_tx_MASK;
    #endif /* (GPS_UART_TX_PIN) */

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_EnableTxPinsInputBuffer
********************************************************************************
*
* Summary:
*  Enables input buffers for TX pins. Restore changes done byte
*GPS_DisableTxPinsInputBuffer.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void GPS_EnableTxPinsInputBuffer(void)
{
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    if(GPS_SCB_MODE_SPI_RUNTM_CFG)
    {
        if(0u != (GPS_SPI_CTRL_REG & GPS_SPI_CTRL_MASTER))
        /* SPI Master */
        {
        #if(GPS_MOSI_SCL_RX_WAKE_PIN)
            GPS_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS &= \
                                            (uint32) ~((uint32) GPS_spi_mosi_i2c_scl_uart_rx_wake_MASK);
        #endif /* (GPS_MOSI_SCL_RX_WAKE_PIN) */

        #if(GPS_MOSI_SCL_RX_PIN)
            GPS_spi_mosi_i2c_scl_uart_rx_INP_DIS &= \
                                            (uint32) ~((uint32) GPS_spi_mosi_i2c_scl_uart_rx_MASK);
        #endif /* (GPS_MOSI_SCL_RX_PIN) */

        #if(GPS_MISO_SDA_TX_PIN)
            GPS_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                            (uint32) ~((uint32) GPS_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (GPS_MISO_SDA_TX_PIN_PIN) */

        #if(GPS_SCLK_PIN)
            GPS_spi_sclk_INP_DIS &= (uint32) ~((uint32) GPS_spi_sclk_MASK);
        #endif /* (GPS_SCLK_PIN) */

        #if(GPS_SS0_PIN)
            GPS_spi_ss0_INP_DIS &= (uint32) ~((uint32) GPS_spi_ss0_MASK);
        #endif /* (GPS_SS1_PIN) */

        #if(GPS_SS1_PIN)
            GPS_spi_ss1_INP_DIS &= (uint32) ~((uint32) GPS_spi_ss1_MASK);
        #endif /* (GPS_SS1_PIN) */

        #if(GPS_SS2_PIN)
            GPS_spi_ss2_INP_DIS &= (uint32) ~((uint32) GPS_spi_ss2_MASK);
        #endif /* (GPS_SS2_PIN) */

        #if(GPS_SS3_PIN)
            GPS_spi_ss3_INP_DIS &= (uint32) ~((uint32) GPS_spi_ss3_MASK);
        #endif /* (GPS_SS3_PIN) */
        }
        else
        /* SPI Slave */
        {
        #if(GPS_MISO_SDA_TX_PIN)
            GPS_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                                (uint32) ~((uint32) GPS_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (GPS_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else if (GPS_SCB_MODE_UART_RUNTM_CFG)
    {
        if(GPS_UART_CTRL_MODE_UART_SMARTCARD != 
                (GPS_UART_CTRL_REG & GPS_UART_CTRL_MODE_MASK))
        /* UART Standard or IrDA */
        {
        #if(GPS_MISO_SDA_TX_PIN)
            GPS_spi_miso_i2c_sda_uart_tx_INP_DIS &= \
                                                (uint32) ~((uint32) GPS_spi_miso_i2c_sda_uart_tx_MASK);
        #endif /* (GPS_MISO_SDA_TX_PIN_PIN) */
        }
    }
    else
    {
        /* Does nothing */
    }
    
#else
        
    /* SCB mode is SPI Master */
    #if(GPS_SPI_MASTER_PINS)
        GPS_sclk_m_INP_DIS &= (uint32) ~((uint32) GPS_sclk_m_MASK);
        GPS_mosi_m_INP_DIS &= (uint32) ~((uint32) GPS_mosi_m_MASK);
        GPS_miso_m_INP_DIS &= (uint32) ~((uint32) GPS_miso_m_MASK);
    #endif /* (GPS_SPI_MASTER_PINS) */

    #if(GPS_SPI_MASTER_SS0_PIN)
        GPS_ss0_m_INP_DIS &= (uint32) ~((uint32) GPS_ss0_m_MASK);
    #endif /* (GPS_SPI_MASTER_SS0_PIN) */

    #if(GPS_SPI_MASTER_SS1_PIN)
        GPS_ss1_m_INP_DIS &= (uint32) ~((uint32) GPS_ss1_m_MASK);
    #endif /* (GPS_SPI_MASTER_SS1_PIN) */

    #if(GPS_SPI_MASTER_SS2_PIN)
        GPS_ss2_m_INP_DIS &= (uint32) ~((uint32) GPS_ss2_m_MASK);
    #endif /* (GPS_SPI_MASTER_SS2_PIN) */

    #if(GPS_SPI_MASTER_SS3_PIN)
        GPS_ss3_m_INP_DIS &= (uint32) ~((uint32) GPS_ss3_m_MASK);
    #endif /* (GPS_SPI_MASTER_SS3_PIN) */
    
    /* SCB mode is SPI Slave */
    #if(GPS_SPI_SLAVE_PINS)
        GPS_miso_s_INP_DIS &= (uint32) ~((uint32) GPS_miso_s_MASK);
    #endif /* (GPS_SPI_SLAVE_PINS) */

    /* SCB mode is UART */
    #if(GPS_UART_TX_PIN)
        GPS_tx_INP_DIS &= (uint32) ~((uint32) GPS_tx_MASK);
    #endif /* (GPS_UART_TX_PIN) */

#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
