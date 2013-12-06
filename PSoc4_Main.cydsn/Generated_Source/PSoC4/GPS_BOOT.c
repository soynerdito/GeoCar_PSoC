/*******************************************************************************
* File Name: GPS_BOOT.c
* Version 1.10
*
* Description:
*  This file provides the source code to the API for the bootloader
*  communication support in SCB Component.
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

#if(GPS_SCB_MODE_I2C_INC)
    #include "GPS_I2C.h"
#endif /* (GPS_SCB_MODE_I2C_INC) */

#if(GPS_SCB_MODE_EZI2C_INC)
    #include "GPS_EZI2C.h"
#endif /* (GPS_SCB_MODE_EZI2C_INC) */

#if(GPS_SCB_MODE_SPI_INC || GPS_SCB_MODE_UART_INC)
    #include "GPS_SPI_UART.h"
#endif /* (GPS_SCB_MODE_SPI_INC || GPS_SCB_MODE_UART_INC) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_GPS) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/*******************************************************************************
* Function Name: GPS_CyBtldrCommStart
********************************************************************************
*
* Summary:
*  Calls Start function fucntion of the bootloader communication component for
*  selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void GPS_CyBtldrCommStart(void)
{
    #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
        if(GPS_SCB_MODE_I2C_RUNTM_CFG)
        {
            GPS_I2CCyBtldrCommStart();
        }
        else if(GPS_SCB_MODE_SPI_RUNTM_CFG)
        {
            GPS_SpiCyBtldrCommStart();
        }
        else if(GPS_SCB_MODE_UART_RUNTM_CFG)
        {
            GPS_UartCyBtldrCommStart();
        }
        else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
        {
             GPS_EzI2CCyBtldrCommStart();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(GPS_SCB_MODE_I2C_CONST_CFG)
        GPS_I2CCyBtldrCommStart();

    #elif(GPS_SCB_MODE_SPI_CONST_CFG)
        GPS_SpiCyBtldrCommStart();

    #elif(GPS_SCB_MODE_UART_CONST_CFG)
        GPS_UartCyBtldrCommStart();

    #elif(GPS_SCB_MODE_EZI2C_CONST_CFG)
        GPS_EzI2CCyBtldrCommStart();

    #else
        /* Do nothing */

    #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_CyBtldrCommStop
********************************************************************************
*
* Summary:
*  Calls Stop function fucntion of the bootloader communication component for
*  selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void GPS_CyBtldrCommStop(void)
{
    #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
        if(GPS_SCB_MODE_I2C_RUNTM_CFG)
        {
            GPS_I2CCyBtldrCommStop();
        }
        else if(GPS_SCB_MODE_SPI_RUNTM_CFG)
        {
            GPS_SpiCyBtldrCommStop();
        }
        else if(GPS_SCB_MODE_UART_RUNTM_CFG)
        {
            GPS_UartCyBtldrCommStop();
        }
        else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            GPS_EzI2CCyBtldrCommStop();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(GPS_SCB_MODE_I2C_CONST_CFG)
        GPS_I2CCyBtldrCommStop();

    #elif(GPS_SCB_MODE_SPI_CONST_CFG)
        GPS_SpiCyBtldrCommStop();

    #elif(GPS_SCB_MODE_UART_CONST_CFG)
        GPS_UartCyBtldrCommStop();

    #elif(GPS_SCB_MODE_EZI2C_CONST_CFG)
        GPS_EzI2CCyBtldrCommStop();

    #else
        /* Do nothing */

    #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_CyBtldrCommReset
********************************************************************************
*
* Summary:
*  Calls reset function fucntion of the bootloader communication component for
*  selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void GPS_CyBtldrCommReset(void)
{
    #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
        if(GPS_SCB_MODE_I2C_RUNTM_CFG)
        {
            GPS_I2CCyBtldrCommReset();
        }
        else if(GPS_SCB_MODE_SPI_RUNTM_CFG)
        {
            GPS_SpiCyBtldrCommReset();
        }
        else if(GPS_SCB_MODE_UART_RUNTM_CFG)
        {
            GPS_UartCyBtldrCommReset();
        }
        else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            GPS_EzI2CCyBtldrCommReset();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(GPS_SCB_MODE_I2C_CONST_CFG)
        GPS_I2CCyBtldrCommReset();

    #elif(GPS_SCB_MODE_SPI_CONST_CFG)
        GPS_SpiCyBtldrCommReset();

    #elif(GPS_SCB_MODE_UART_CONST_CFG)
        GPS_UartCyBtldrCommReset();

    #elif(GPS_SCB_MODE_EZI2C_CONST_CFG)
        GPS_EzI2CCyBtldrCommReset();

    #else
        /* Do nothing */

    #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: GPS_CyBtldrCommRead
********************************************************************************
*
* Summary:
*  Calls read fucntion of the bootloader communication component for selected
*  mode.
*
* Parameters:
*  pData:    Pointer to storage for the block of data to be read from the
*            bootloader host
*  size:     Number of bytes to be read.
*  count:    Pointer to the variable to write the number of bytes actually
*            read.
*  timeOut:  Number of units in 10 ms to wait before returning because of a
*            timeout.
*
* Return:
*  Returns CYRET_SUCCESS if no problem was encountered or returns the value
*  that best describes the problem.
*
*******************************************************************************/
cystatus GPS_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
        if(GPS_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = GPS_I2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(GPS_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = GPS_SpiCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(GPS_SCB_MODE_UART_RUNTM_CFG)
        {
            status = GPS_UartCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = GPS_EzI2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode: return status */
        }

    #elif(GPS_SCB_MODE_I2C_CONST_CFG)
        status = GPS_I2CCyBtldrCommRead(pData, size, count, timeOut);

    #elif(GPS_SCB_MODE_SPI_CONST_CFG)
        status = GPS_SpiCyBtldrCommRead(pData, size, count, timeOut);

    #elif(GPS_SCB_MODE_UART_CONST_CFG)
        status = GPS_UartCyBtldrCommRead(pData, size, count, timeOut);

    #elif(GPS_SCB_MODE_EZI2C_CONST_CFG)
        status = GPS_EzI2CCyBtldrCommRead(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode: return status */

    #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}


/*******************************************************************************
* Function Name: GPS_CyBtldrCommWrite
********************************************************************************
*
* Summary:
*  Calls write fucntion of the bootloader communication component for selected
*  mode.
*
* Parameters:
*  pData:    Pointer to the block of data to be written to the bootloader host.
*  size:     Number of bytes to be written.
*  count:    Pointer to the variable to write the number of bytes actually
*            written.
*  timeOut:  Number of units in 10 ms to wait before returning because of a
*            timeout.
*
* Return:
*  Returns CYRET_SUCCESS if no problem was encountered or returns the value
*  that best describes the problem.
*
*******************************************************************************/
cystatus GPS_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
        if(GPS_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = GPS_I2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(GPS_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = GPS_SpiCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(GPS_SCB_MODE_UART_RUNTM_CFG)
        {
            status = GPS_UartCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(GPS_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = GPS_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode: return status */
        }
    #elif(GPS_SCB_MODE_I2C_CONST_CFG)
        status = GPS_I2CCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(GPS_SCB_MODE_SPI_CONST_CFG)
        status = GPS_SpiCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(GPS_SCB_MODE_UART_CONST_CFG)
        status = GPS_UartCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(GPS_SCB_MODE_EZI2C_CONST_CFG)
        status = GPS_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode: return status */

    #endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_GPS) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */


/* [] END OF FILE */
