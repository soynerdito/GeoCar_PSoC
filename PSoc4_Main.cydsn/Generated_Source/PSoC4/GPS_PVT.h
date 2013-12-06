/*******************************************************************************
* File Name: .h
* Version 1.10
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component in I2C mode.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_GPS_H)
#define CY_SCB_PVT_GPS_H

#include "GPS.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define GPS_SetI2CExtClkInterruptMode(interruptMask) GPS_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define GPS_ClearI2CExtClkInterruptSource(interruptMask) GPS_CLEAR_INTR_I2C_EC(interruptMask)
#define GPS_GetI2CExtClkInterruptSource()                (GPS_INTR_I2C_EC_REG)
#define GPS_GetI2CExtClkInterruptMode()                  (GPS_INTR_I2C_EC_MASK_REG)
#define GPS_GetI2CExtClkInterruptSourceMasked()          (GPS_INTR_I2C_EC_MASKED_REG)

/* APIs to service INTR_SPI_EC register */
#define GPS_SetSpiExtClkInterruptMode(interruptMask) GPS_WRITE_INTR_SPI_EC_MASK(interruptMask)
#define GPS_ClearSpiExtClkInterruptSource(interruptMask) GPS_CLEAR_INTR_SPI_EC(interruptMask)
#define GPS_GetExtSpiClkInterruptSource()                 (GPS_INTR_SPI_EC_REG)
#define GPS_GetExtSpiClkInterruptMode()                   (GPS_INTR_SPI_EC_MASK_REG)
#define GPS_GetExtSpiClkInterruptSourceMasked()           (GPS_INTR_SPI_EC_MASKED_REG)

#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void GPS_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx);
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

void GPS_DisableTxPinsInputBuffer(void);
void GPS_EnableTxPinsInputBuffer(void);


/**********************************
*     Vars with External Linkage
**********************************/

extern cyisraddress GPS_customIntrHandler;
extern GPS_BACKUP_STRUCT GPS_backup;

#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    extern uint8 GPS_scbMode;
    extern uint8 GPS_scbEnableWake;
    extern uint8 GPS_scbEnableIntr;

    /* I2C config vars */
    extern uint8 GPS_mode;
    extern uint8 GPS_acceptAddr;

    /* SPI/UART config vars */
    extern volatile uint8 * GPS_rxBuffer;
    extern uint8   GPS_rxDataBits;
    extern uint32  GPS_rxBufferSize;

    extern volatile uint8 * GPS_txBuffer;
    extern uint8   GPS_txDataBits;
    extern uint32  GPS_txBufferSize;

    /* EZI2C config vars */
    extern uint8 GPS_numberOfAddr;
    extern uint8 GPS_subAddrSize;
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

#endif /* (CY_SCB_PVT_GPS_H) */


/* [] END OF FILE */
