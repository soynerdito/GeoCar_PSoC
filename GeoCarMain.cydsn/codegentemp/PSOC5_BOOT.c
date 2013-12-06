/*******************************************************************************
* File Name: PSOC5_BOOT.c
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

#include "PSOC5.h"

#if(PSOC5_SCB_MODE_I2C_INC)
    #include "PSOC5_I2C.h"
#endif /* (PSOC5_SCB_MODE_I2C_INC) */

#if(PSOC5_SCB_MODE_EZI2C_INC)
    #include "PSOC5_EZI2C.h"
#endif /* (PSOC5_SCB_MODE_EZI2C_INC) */

#if(PSOC5_SCB_MODE_SPI_INC || PSOC5_SCB_MODE_UART_INC)
    #include "PSOC5_SPI_UART.h"
#endif /* (PSOC5_SCB_MODE_SPI_INC || PSOC5_SCB_MODE_UART_INC) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/*******************************************************************************
* Function Name: PSOC5_CyBtldrCommStart
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
void PSOC5_CyBtldrCommStart(void)
{
    #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
        {
            PSOC5_I2CCyBtldrCommStart();
        }
        else if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
        {
            PSOC5_SpiCyBtldrCommStart();
        }
        else if(PSOC5_SCB_MODE_UART_RUNTM_CFG)
        {
            PSOC5_UartCyBtldrCommStart();
        }
        else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
        {
             PSOC5_EzI2CCyBtldrCommStart();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(PSOC5_SCB_MODE_I2C_CONST_CFG)
        PSOC5_I2CCyBtldrCommStart();

    #elif(PSOC5_SCB_MODE_SPI_CONST_CFG)
        PSOC5_SpiCyBtldrCommStart();

    #elif(PSOC5_SCB_MODE_UART_CONST_CFG)
        PSOC5_UartCyBtldrCommStart();

    #elif(PSOC5_SCB_MODE_EZI2C_CONST_CFG)
        PSOC5_EzI2CCyBtldrCommStart();

    #else
        /* Do nothing */

    #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_CyBtldrCommStop
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
void PSOC5_CyBtldrCommStop(void)
{
    #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
        {
            PSOC5_I2CCyBtldrCommStop();
        }
        else if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
        {
            PSOC5_SpiCyBtldrCommStop();
        }
        else if(PSOC5_SCB_MODE_UART_RUNTM_CFG)
        {
            PSOC5_UartCyBtldrCommStop();
        }
        else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PSOC5_EzI2CCyBtldrCommStop();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(PSOC5_SCB_MODE_I2C_CONST_CFG)
        PSOC5_I2CCyBtldrCommStop();

    #elif(PSOC5_SCB_MODE_SPI_CONST_CFG)
        PSOC5_SpiCyBtldrCommStop();

    #elif(PSOC5_SCB_MODE_UART_CONST_CFG)
        PSOC5_UartCyBtldrCommStop();

    #elif(PSOC5_SCB_MODE_EZI2C_CONST_CFG)
        PSOC5_EzI2CCyBtldrCommStop();

    #else
        /* Do nothing */

    #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_CyBtldrCommReset
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
void PSOC5_CyBtldrCommReset(void)
{
    #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
        {
            PSOC5_I2CCyBtldrCommReset();
        }
        else if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
        {
            PSOC5_SpiCyBtldrCommReset();
        }
        else if(PSOC5_SCB_MODE_UART_RUNTM_CFG)
        {
            PSOC5_UartCyBtldrCommReset();
        }
        else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            PSOC5_EzI2CCyBtldrCommReset();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(PSOC5_SCB_MODE_I2C_CONST_CFG)
        PSOC5_I2CCyBtldrCommReset();

    #elif(PSOC5_SCB_MODE_SPI_CONST_CFG)
        PSOC5_SpiCyBtldrCommReset();

    #elif(PSOC5_SCB_MODE_UART_CONST_CFG)
        PSOC5_UartCyBtldrCommReset();

    #elif(PSOC5_SCB_MODE_EZI2C_CONST_CFG)
        PSOC5_EzI2CCyBtldrCommReset();

    #else
        /* Do nothing */

    #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: PSOC5_CyBtldrCommRead
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
cystatus PSOC5_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = PSOC5_I2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = PSOC5_SpiCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(PSOC5_SCB_MODE_UART_RUNTM_CFG)
        {
            status = PSOC5_UartCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = PSOC5_EzI2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode: return status */
        }

    #elif(PSOC5_SCB_MODE_I2C_CONST_CFG)
        status = PSOC5_I2CCyBtldrCommRead(pData, size, count, timeOut);

    #elif(PSOC5_SCB_MODE_SPI_CONST_CFG)
        status = PSOC5_SpiCyBtldrCommRead(pData, size, count, timeOut);

    #elif(PSOC5_SCB_MODE_UART_CONST_CFG)
        status = PSOC5_UartCyBtldrCommRead(pData, size, count, timeOut);

    #elif(PSOC5_SCB_MODE_EZI2C_CONST_CFG)
        status = PSOC5_EzI2CCyBtldrCommRead(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode: return status */

    #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}


/*******************************************************************************
* Function Name: PSOC5_CyBtldrCommWrite
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
cystatus PSOC5_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
        if(PSOC5_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = PSOC5_I2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(PSOC5_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = PSOC5_SpiCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(PSOC5_SCB_MODE_UART_RUNTM_CFG)
        {
            status = PSOC5_UartCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(PSOC5_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = PSOC5_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode: return status */
        }
    #elif(PSOC5_SCB_MODE_I2C_CONST_CFG)
        status = PSOC5_I2CCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(PSOC5_SCB_MODE_SPI_CONST_CFG)
        status = PSOC5_SpiCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(PSOC5_SCB_MODE_UART_CONST_CFG)
        status = PSOC5_UartCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(PSOC5_SCB_MODE_EZI2C_CONST_CFG)
        status = PSOC5_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode: return status */

    #endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */


/* [] END OF FILE */
