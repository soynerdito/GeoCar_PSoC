/*******************************************************************************
* File Name: GPS_SPI_UART.h
* Version 1.10
*
* Description:
*  This file provides constants and parameter values for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_SPI_UART_GPS_H)
#define CY_SCB_SPI_UART_GPS_H

#include "GPS.h"


/***************************************
*   SPI Initial Parameter Constants
****************************************/

#define GPS_SPI_MODE                   (0u)
#define GPS_SPI_SUB_MODE               (0u)
#define GPS_SPI_CLOCK_MODE             (0u)
#define GPS_SPI_OVS_FACTOR             (16u)
#define GPS_SPI_MEDIAN_FILTER_ENABLE   (0u)
#define GPS_SPI_LATE_MISO_SAMPLE_ENABLE (0u)
#define GPS_SPI_RX_DATA_BITS_NUM       (8u)
#define GPS_SPI_TX_DATA_BITS_NUM       (8u)
#define GPS_SPI_WAKE_ENABLE            (0u)
#define GPS_SPI_BITS_ORDER             (1u)
#define GPS_SPI_TRANSFER_SEPARATION    (1u)
#define GPS_SPI_NUMBER_OF_SS_LINES     (1u)
#define GPS_SPI_RX_BUFFER_SIZE         (8u)
#define GPS_SPI_TX_BUFFER_SIZE         (8u)

#define GPS_SPI_INTERRUPT_MODE         (0u)

#define GPS_SPI_INTR_RX_MASK           (0u)
#define GPS_SPI_INTR_TX_MASK           (0u)

#define GPS_SPI_RX_TRIGGER_LEVEL       (7u)
#define GPS_SPI_TX_TRIGGER_LEVEL       (0u)


/***************************************
*   UART Initial Parameter Constants
****************************************/

#define GPS_UART_SUB_MODE              (0u)
#define GPS_UART_DIRECTION             (3u)
#define GPS_UART_DATA_BITS_NUM         (8u)
#define GPS_UART_PARITY_TYPE           (2u)
#define GPS_UART_STOP_BITS_NUM         (2u)
#define GPS_UART_OVS_FACTOR            (12u)
#define GPS_UART_IRDA_LOW_POWER        (0u)
#define GPS_UART_MEDIAN_FILTER_ENABLE  (0u)
#define GPS_UART_RETRY_ON_NACK         (0u)
#define GPS_UART_IRDA_POLARITY         (0u)
#define GPS_UART_DROP_ON_FRAME_ERR     (0u)
#define GPS_UART_DROP_ON_PARITY_ERR    (0u)
#define GPS_UART_WAKE_ENABLE           (0u)
#define GPS_UART_RX_BUFFER_SIZE        (8u)
#define GPS_UART_TX_BUFFER_SIZE        (8u)
#define GPS_UART_MP_MODE_ENABLE        (0u)
#define GPS_UART_MP_ACCEPT_ADDRESS     (0u)
#define GPS_UART_MP_RX_ADDRESS         (2u)
#define GPS_UART_MP_RX_ADDRESS_MASK    (255u)

#define GPS_UART_INTERRUPT_MODE        (0u)

#define GPS_UART_INTR_RX_MASK          (0u)
#define GPS_UART_INTR_TX_MASK          (0u)

#define GPS_UART_RX_TRIGGER_LEVEL      (7u)
#define GPS_UART_TX_TRIGGER_LEVEL      (0u)

/* Sources of RX errors */
#define GPS_INTR_RX_ERR        (GPS_INTR_RX_OVERFLOW    | \
                                             GPS_INTR_RX_UNDERFLOW   | \
                                             GPS_INTR_RX_FRAME_ERROR | \
                                             GPS_INTR_RX_PARITY_ERROR)

/* UART direction enum */
#define GPS_UART_RX    (1u)
#define GPS_UART_TX    (2u)
#define GPS_UART_TX_RX (3u)


/***************************************
*   Conditional Compilation Parameters
****************************************/

#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)

    /* Direction */
    #define GPS_RX_DIRECTION           (1u)
    #define GPS_TX_DIRECTION           (1u)
    #define GPS_UART_RX_DIRECTION      (1u)
    #define GPS_UART_TX_DIRECTION      (1u)

    /* Only external RX and TX buffer for uncofigured mode */
    #define GPS_INTERNAL_RX_SW_BUFFER   (0u)
    #define GPS_INTERNAL_TX_SW_BUFFER   (0u)

    /* Get RX and TX buffer size */
    #define GPS_RX_BUFFER_SIZE (GPS_rxBufferSize)
    #define GPS_TX_BUFFER_SIZE (GPS_txBufferSize)

    /* Return true if buffer is provided */
    #define GPS_CHECK_RX_SW_BUFFER (NULL != GPS_rxBuffer)
    #define GPS_CHECK_TX_SW_BUFFER (NULL != GPS_txBuffer)

    /* Alwasy provde global variables to support RX and TX buffers */
    #define GPS_INTERNAL_RX_SW_BUFFER_CONST    (1u)
    #define GPS_INTERNAL_TX_SW_BUFFER_CONST    (1u)

    /* Get wakeup enable option */
    #define GPS_SPI_WAKE_ENABLE_CONST  (1u)
    #define GPS_CHECK_SPI_WAKE_ENABLE  (0u != GPS_scbEnableWake)
    #define GPS_UART_WAKE_ENABLE_CONST (1u)

#else

    /* SPI internal RX and TX buffers */
    #define GPS_INTERNAL_SPI_RX_SW_BUFFER  (GPS_SPI_RX_BUFFER_SIZE > \
                                                                                            GPS_FIFO_SIZE)
    #define GPS_INTERNAL_SPI_TX_SW_BUFFER  (GPS_SPI_TX_BUFFER_SIZE > \
                                                                                            GPS_FIFO_SIZE)

    /* UART internal RX and TX buffers */
    #define GPS_INTERNAL_UART_RX_SW_BUFFER  (GPS_UART_RX_BUFFER_SIZE > \
                                                                                            GPS_FIFO_SIZE)
    #define GPS_INTERNAL_UART_TX_SW_BUFFER  (GPS_UART_TX_BUFFER_SIZE > \
                                                                                            GPS_FIFO_SIZE)

    /* SPI Direction */
    #define GPS_SPI_RX_DIRECTION (1u)
    #define GPS_SPI_TX_DIRECTION (1u)

    /* UART Direction */
    #define GPS_UART_RX_DIRECTION (0u != (GPS_UART_DIRECTION & GPS_UART_RX))
    #define GPS_UART_TX_DIRECTION (0u != (GPS_UART_DIRECTION & GPS_UART_TX))

    /* Direction */
    #define GPS_RX_DIRECTION ((GPS_SCB_MODE_SPI_CONST_CFG) ? \
                                            (GPS_SPI_RX_DIRECTION) : (GPS_UART_RX_DIRECTION))

    #define GPS_TX_DIRECTION ((GPS_SCB_MODE_SPI_CONST_CFG) ? \
                                            (GPS_SPI_TX_DIRECTION) : (GPS_UART_TX_DIRECTION))

    /* Internal RX and TX buffer: for SPI or UART */
    #if(GPS_SCB_MODE_SPI_CONST_CFG)

        /* Internal SPI RX and TX buffer */
        #define GPS_INTERNAL_RX_SW_BUFFER  (GPS_INTERNAL_SPI_RX_SW_BUFFER)
        #define GPS_INTERNAL_TX_SW_BUFFER  (GPS_INTERNAL_SPI_TX_SW_BUFFER)

        /* Internal SPI RX and TX buffer size */
        #define GPS_RX_BUFFER_SIZE         (GPS_SPI_RX_BUFFER_SIZE + 1u)
        #define GPS_TX_BUFFER_SIZE         (GPS_SPI_TX_BUFFER_SIZE)
        
        /* Get wakeup enable option */
        #define GPS_SPI_WAKE_ENABLE_CONST  (0u != GPS_SPI_WAKE_ENABLE)
        #define GPS_UART_WAKE_ENABLE_CONST (0u)

    #else

        /* Internal UART RX and TX buffer */
        #define GPS_INTERNAL_RX_SW_BUFFER  (GPS_INTERNAL_UART_RX_SW_BUFFER)
        #define GPS_INTERNAL_TX_SW_BUFFER  (GPS_INTERNAL_UART_TX_SW_BUFFER)

        /* Internal UART RX and TX buffer size */
        #define GPS_RX_BUFFER_SIZE         (GPS_UART_RX_BUFFER_SIZE + 1u)
        #define GPS_TX_BUFFER_SIZE         (GPS_UART_TX_BUFFER_SIZE)
        
        /* Get wakeup enable option */
        #define GPS_SPI_WAKE_ENABLE_CONST  (0u)
        #define GPS_UART_WAKE_ENABLE_CONST (0u != GPS_UART_WAKE_ENABLE)
    #endif /* (GPS_SCB_MODE_SPI_CONST_CFG) */

    /* Internal RX and TX buffer: for SPI or UART. Used in conditional compilation check */
    #define GPS_CHECK_RX_SW_BUFFER (GPS_INTERNAL_RX_SW_BUFFER)
    #define GPS_CHECK_TX_SW_BUFFER (GPS_INTERNAL_TX_SW_BUFFER)

    /* Provide global variables to support RX and TX buffers */
    #define GPS_INTERNAL_RX_SW_BUFFER_CONST    (GPS_INTERNAL_RX_SW_BUFFER)
    #define GPS_INTERNAL_TX_SW_BUFFER_CONST    (GPS_INTERNAL_TX_SW_BUFFER)
    
    /* Wakeup for SPI */
    #define GPS_CHECK_SPI_WAKE_ENABLE  (GPS_SPI_WAKE_ENABLE_CONST)

#endif /* End (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Bootloader communication interface enable: NOT supported yet */
#define GPS_SPI_BTLDR_COMM_ENABLED   ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_GPS) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

#define GPS_UART_BTLDR_COMM_ENABLED   ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_GPS) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))


/***************************************
*       Type Definitions
***************************************/

/* GPS_SPI_INIT_STRUCT */
typedef struct
{
    uint32 mode;
    uint32 submode;
    uint32 sclkMode;
    uint32 oversample;
    uint32 enableMedianFilter;
    uint32 enableLateSampling;
    uint32 enableWake;
    uint32 rxDataBits;
    uint32 txDataBits;
    uint32 bitOrder;
    uint32 transferSeperation;
    uint32 rxBufferSize;
    uint8* rxBuffer;
    uint32 txBufferSize;
    uint8* txBuffer;
    uint32 enableInterrupt;
    uint32 rxInterruptMask;
    uint32 rxTriggerLevel;
    uint32 txInterruptMask;
    uint32 txTriggerLevel;
} GPS_SPI_INIT_STRUCT;

/* GPS_UART_INIT_STRUCT */
typedef struct
{
    uint32 mode;
    uint32 direction;
    uint32 dataBits;
    uint32 parity;
    uint32 stopBits;
    uint32 oversample;
    uint32 enableIrdaLowPower;
    uint32 enableMedianFilter;
    uint32 enableRetryNack;
    uint32 enableInvertedRx;
    uint32 dropOnParityErr;
    uint32 dropOnFrameErr;
    uint32 enableWake;
    uint32 rxBufferSize;
    uint8* rxBuffer;
    uint32 txBufferSize;
    uint8* txBuffer;
    uint32 enableMultiproc;
    uint32 multiprocAcceptAddr;
    uint32 multiprocAddr;
    uint32 multiprocAddrMask;
    uint32 enableInterrupt;
    uint32 rxInterruptMask;
    uint32 rxTriggerLevel;
    uint32 txInterruptMask;
    uint32 txTriggerLevel;
} GPS_UART_INIT_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/* SPI specific functions */
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    void GPS_SpiInit(const GPS_SPI_INIT_STRUCT *config);
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(GPS_SCB_MODE_SPI_INC)
    void GPS_SpiSetActiveSlaveSelect(uint32 activeSelect);
#endif /* (GPS_SCB_MODE_SPI_INC) */

/* UART specific functions */
#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    void GPS_UartInit(const GPS_UART_INIT_STRUCT *config);
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(GPS_SCB_MODE_UART_INC)
    void GPS_UartSetRxAddress(uint32 address);
    void GPS_UartSetRxAddressMask(uint32 addressMask);
#endif /* (GPS_SCB_MODE_UART_INC) */

/* UART RX direction APIs */
#if(GPS_UART_RX_DIRECTION)
    uint32 GPS_UartGetChar(void);
    uint32 GPS_UartGetByte(void);
#endif /* (GPS_UART_RX_DIRECTION) */

/* UART TX direction APIs */
#if(GPS_UART_TX_DIRECTION)
    #define GPS_UartPutChar(ch)    GPS_SpiUartWriteTxData((uint32)(ch))
    void GPS_UartPutString(const char8 string[]);
    void GPS_UartPutCRLF(uint32 txDataByte);
#endif /* (GPS_UART_TX_DIRECTION) */

/* Common APIs Rx direction */
#if(GPS_RX_DIRECTION)
    uint32 GPS_SpiUartReadRxData(void);
    uint32 GPS_SpiUartGetRxBufferSize(void);
    void   GPS_SpiUartClearRxBuffer(void);
#endif /* (GPS_RX_DIRECTION) */

/* Common APIs Tx direction */
#if(GPS_TX_DIRECTION)
    void   GPS_SpiUartWriteTxData(uint32 txDataByte);
    void   GPS_SpiUartPutArray(const uint8 wrBuf[], uint32 count);
    void   GPS_SpiUartClearTxBuffer(void);
    uint32 GPS_SpiUartGetTxBufferSize(void);
#endif /* (GPS_TX_DIRECTION) */

CY_ISR_PROTO(GPS_SPI_UART_ISR);

#if(GPS_UART_RX_WAKEUP_IRQ)
    CY_ISR_PROTO(GPS_UART_WAKEUP_ISR);
#endif /* (GPS_UART_RX_WAKEUP_IRQ) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (GPS_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void GPS_SpiCyBtldrCommStart(void);
    void GPS_SpiCyBtldrCommStop (void);
    void GPS_SpiCyBtldrCommReset(void);
    cystatus GPS_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus GPS_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (GPS_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (GPS_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void GPS_UartCyBtldrCommStart(void);
    void GPS_UartCyBtldrCommStop (void);
    void GPS_UartCyBtldrCommReset(void);
    cystatus GPS_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus GPS_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (GPS_UART_BTLDR_COMM_ENABLED) */


/***************************************
*     Buffer Access Macro Definitions
***************************************/

#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    /* RX direction */
    void   GPS_PutWordInRxBuffer  (uint32 idx, uint32 rxDataByte);
    uint32 GPS_GetWordFromRxBuffer(uint32 idx);

    /* TX direction */
    void   GPS_PutWordInTxBuffer  (uint32 idx, uint32 txDataByte);
    uint32 GPS_GetWordFromTxBuffer(uint32 idx);

#else

    /* RX direction */
    #if(GPS_INTERNAL_RX_SW_BUFFER_CONST)
        #define GPS_PutWordInRxBuffer(idx, rxDataByte) \
                do{                                                 \
                    GPS_rxBufferInternal[(idx)] = ((uint8) (rxDataByte)); \
                }while(0)

        #define GPS_GetWordFromRxBuffer(idx) GPS_rxBufferInternal[(idx)]

    #endif /* (GPS_INTERNAL_RX_SW_BUFFER_CONST) */

    /* TX direction */
    #if(GPS_INTERNAL_TX_SW_BUFFER_CONST)
        #define GPS_PutWordInTxBuffer(idx, txDataByte) \
                    do{                                             \
                        GPS_txBufferInternal[(idx)] = ((uint8) (txDataByte)); \
                    }while(0)

        #define GPS_GetWordFromTxBuffer(idx) GPS_txBufferInternal[(idx)]

    #endif /* (GPS_INTERNAL_TX_SW_BUFFER_CONST) */

#endif /* (GPS_TX_SW_BUFFER_ENABLE) */


/***************************************
*         SPI API Constants
***************************************/

/* SPI mode enum */
#define GPS_SPI_SLAVE  (0u)
#define GPS_SPI_MASTER (1u)

/* SPI sub mode enum */
#define GPS_SPI_MODE_MOTOROLA      (0x00u)
#define GPS_SPI_MODE_TI_COINCIDES  (0x01u)
#define GPS_SPI_MODE_TI_PRECEDES   (0x11u)
#define GPS_SPI_MODE_NATIONAL      (0x02u)
#define GPS_SPI_MODE_MASK          (0x03u)
#define GPS_SPI_MODE_TI_PRECEDES_MASK  (0x10u)

/* SPI phase and polarity mode enum */
#define GPS_SPI_SCLK_CPHA0_CPOL0   (0x00u)
#define GPS_SPI_SCLK_CPHA0_CPOL1   (0x02u)
#define GPS_SPI_SCLK_CPHA1_CPOL0   (0x01u)
#define GPS_SPI_SCLK_CPHA1_CPOL1   (0x03u)

/* SPI bits order enum */
#define GPS_BITS_ORDER_LSB_FIRST   (0u)
#define GPS_BITS_ORDER_MSB_FIRST   (1u)

/* SPI transfer separation enum */
#define GPS_SPI_TRANSFER_SEPARATED     (0u)
#define GPS_SPI_TRANSFER_CONTINUOUS    (1u)

/* "activeSS" constants for SpiSetActiveSlaveSelect() function */
#define GPS_SPIM_ACTIVE_SS0    (0x00u)
#define GPS_SPIM_ACTIVE_SS1    (0x01u)
#define GPS_SPIM_ACTIVE_SS2    (0x02u)
#define GPS_SPIM_ACTIVE_SS3    (0x03u)


/***************************************
*         UART API Constants
***************************************/

/* UART sub-modes enum */
#define GPS_UART_MODE_STD          (0u)
#define GPS_UART_MODE_SMARTCARD    (1u)
#define GPS_UART_MODE_IRDA         (2u)

/* UART direction enum */
#define GPS_UART_RX    (1u)
#define GPS_UART_TX    (2u)
#define GPS_UART_TX_RX (3u)

/* UART parity enum */
#define GPS_UART_PARITY_EVEN   (0u)
#define GPS_UART_PARITY_ODD    (1u)
#define GPS_UART_PARITY_NONE   (2u)

/* UART stop bits enum */
#define GPS_UART_STOP_BITS_1   (1u)
#define GPS_UART_STOP_BITS_1_5 (2u)
#define GPS_UART_STOP_BITS_2   (3u)

/* UART IrDA low power OVS enum */
#define GPS_UART_IRDA_LP_OVS16     (16u)
#define GPS_UART_IRDA_LP_OVS32     (32u)
#define GPS_UART_IRDA_LP_OVS48     (48u)
#define GPS_UART_IRDA_LP_OVS96     (96u)
#define GPS_UART_IRDA_LP_OVS192    (192u)
#define GPS_UART_IRDA_LP_OVS768    (768u)
#define GPS_UART_IRDA_LP_OVS1536   (1536u)

/* Uart MP: mark (address) and space (data) bit definitions */
#define GPS_UART_MP_MARK       (0x100u)
#define GPS_UART_MP_SPACE      (0x000u)


/***************************************
*     Vars with External Linkage
***************************************/

#if(GPS_SCB_MODE_UNCONFIG_CONST_CFG)
    extern const GPS_SPI_INIT_STRUCT  GPS_configSpi;
    extern const GPS_UART_INIT_STRUCT GPS_configUart;
#endif /* (GPS_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*    Specific SPI Macro Definitions
***************************************/

#define GPS_GET_SPI_INTR_SLAVE_MASK(sourceMask)  ((sourceMask) & GPS_INTR_SLAVE_SPI_BUS_ERROR)
#define GPS_GET_SPI_INTR_MASTER_MASK(sourceMask) ((sourceMask) & GPS_INTR_MASTER_SPI_DONE)
#define GPS_GET_SPI_INTR_RX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~GPS_INTR_SLAVE_SPI_BUS_ERROR)

#define GPS_GET_SPI_INTR_TX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~GPS_INTR_MASTER_SPI_DONE)


/***************************************
*    Specific UART Macro Definitions
***************************************/

#define GPS_UART_GET_CTRL_OVS_IRDA_LP(oversample) \
        ((GPS_UART_IRDA_LP_OVS16   == (oversample)) ? GPS_CTRL_OVS_IRDA_LP_OVS16 : \
         ((GPS_UART_IRDA_LP_OVS32   == (oversample)) ? GPS_CTRL_OVS_IRDA_LP_OVS32 : \
          ((GPS_UART_IRDA_LP_OVS48   == (oversample)) ? GPS_CTRL_OVS_IRDA_LP_OVS48 : \
           ((GPS_UART_IRDA_LP_OVS96   == (oversample)) ? GPS_CTRL_OVS_IRDA_LP_OVS96 : \
            ((GPS_UART_IRDA_LP_OVS192  == (oversample)) ? GPS_CTRL_OVS_IRDA_LP_OVS192 : \
             ((GPS_UART_IRDA_LP_OVS768  == (oversample)) ? GPS_CTRL_OVS_IRDA_LP_OVS768 : \
              ((GPS_UART_IRDA_LP_OVS1536 == (oversample)) ? GPS_CTRL_OVS_IRDA_LP_OVS1536 : \
                                                                          GPS_CTRL_OVS_IRDA_LP_OVS16)))))))

#define GPS_GET_UART_RX_CTRL_ENABLED(direction) ((0u != (GPS_UART_RX & (direction))) ? \
                                                                    (GPS_RX_CTRL_ENABLED) : (0u))

#define GPS_GET_UART_TX_CTRL_ENABLED(direction) ((0u != (GPS_UART_TX & (direction))) ? \
                                                                    (GPS_TX_CTRL_ENABLED) : (0u))


/***************************************
*        SPI Register Settings
***************************************/

#define GPS_CTRL_SPI      (GPS_CTRL_MODE_SPI)
#define GPS_SPI_RX_CTRL   (GPS_RX_CTRL_ENABLED)
#define GPS_SPI_TX_CTRL   (GPS_TX_CTRL_ENABLED)


/***************************************
*       SPI Init Register Settings
***************************************/

#if(GPS_SCB_MODE_SPI_CONST_CFG)

    /* SPI Configuration */
    #define GPS_SPI_DEFAULT_CTRL \
                    (GPS_GET_CTRL_OVS(GPS_SPI_OVS_FACTOR)         | \
                     GPS_GET_CTRL_EC_AM_MODE(GPS_SPI_WAKE_ENABLE) | \
                     GPS_CTRL_SPI)

    #define GPS_SPI_DEFAULT_SPI_CTRL \
                    (GPS_GET_SPI_CTRL_CONTINUOUS    (GPS_SPI_TRANSFER_SEPARATION)       | \
                     GPS_GET_SPI_CTRL_SELECT_PRECEDE(GPS_SPI_SUB_MODE &                   \
                                                                  GPS_SPI_MODE_TI_PRECEDES_MASK)     | \
                     GPS_GET_SPI_CTRL_SCLK_MODE     (GPS_SPI_CLOCK_MODE)                | \
                     GPS_GET_SPI_CTRL_LATE_MISO_SAMPLE(GPS_SPI_LATE_MISO_SAMPLE_ENABLE) | \
                     GPS_GET_SPI_CTRL_SUB_MODE      (GPS_SPI_SUB_MODE)                  | \
                     GPS_GET_SPI_CTRL_MASTER_MODE   (GPS_SPI_MODE))

    /* RX direction */
    #define GPS_SPI_DEFAULT_RX_CTRL \
                    (GPS_GET_RX_CTRL_DATA_WIDTH(GPS_SPI_RX_DATA_BITS_NUM)     | \
                     GPS_GET_RX_CTRL_BIT_ORDER (GPS_SPI_BITS_ORDER)           | \
                     GPS_GET_RX_CTRL_MEDIAN    (GPS_SPI_MEDIAN_FILTER_ENABLE) | \
                     GPS_SPI_RX_CTRL)

    #define GPS_SPI_DEFAULT_RX_FIFO_CTRL \
                    GPS_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(GPS_SPI_RX_TRIGGER_LEVEL)

    /* TX direction */
    #define GPS_SPI_DEFAULT_TX_CTRL \
                    (GPS_GET_TX_CTRL_DATA_WIDTH(GPS_SPI_TX_DATA_BITS_NUM) | \
                     GPS_GET_TX_CTRL_BIT_ORDER (GPS_SPI_BITS_ORDER)       | \
                     GPS_SPI_TX_CTRL)

    #define GPS_SPI_DEFAULT_TX_FIFO_CTRL \
                    GPS_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(GPS_SPI_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define GPS_SPI_DEFAULT_INTR_SPI_EC_MASK   (GPS_NO_INTR_SOURCES)

    #define GPS_SPI_DEFAULT_INTR_I2C_EC_MASK   (GPS_NO_INTR_SOURCES)
    #define GPS_SPI_DEFAULT_INTR_SLAVE_MASK \
                    (GPS_SPI_INTR_RX_MASK & GPS_INTR_SLAVE_SPI_BUS_ERROR)

    #define GPS_SPI_DEFAULT_INTR_MASTER_MASK \
                    (GPS_SPI_INTR_TX_MASK & GPS_INTR_MASTER_SPI_DONE)

    #define GPS_SPI_DEFAULT_INTR_RX_MASK \
                    (GPS_SPI_INTR_RX_MASK & (uint32) ~GPS_INTR_SLAVE_SPI_BUS_ERROR)

    #define GPS_SPI_DEFAULT_INTR_TX_MASK \
                    (GPS_SPI_INTR_TX_MASK & (uint32) ~GPS_INTR_MASTER_SPI_DONE)

#endif /* (GPS_SCB_MODE_SPI_CONST_CFG) */


/***************************************
*        UART Register Settings
***************************************/

#define GPS_CTRL_UART      (GPS_CTRL_MODE_UART)
#define GPS_UART_RX_CTRL   (GPS_RX_CTRL_LSB_FIRST) /* LSB for UART goes first */
#define GPS_UART_TX_CTRL   (GPS_TX_CTRL_LSB_FIRST) /* LSB for UART goes first */


/***************************************
*      UART Init Register Settings
***************************************/

#if(GPS_SCB_MODE_UART_CONST_CFG)

    /* UART configuration */
    #if(GPS_UART_MODE_IRDA == GPS_UART_SUB_MODE)

        #define GPS_DEFAULT_CTRL_OVS   ((0u != GPS_UART_IRDA_LOW_POWER) ?              \
                                (GPS_UART_GET_CTRL_OVS_IRDA_LP(GPS_UART_OVS_FACTOR)) : \
                                (GPS_CTRL_OVS_IRDA_OVS16))

    #else

        #define GPS_DEFAULT_CTRL_OVS   GPS_GET_CTRL_OVS(GPS_UART_OVS_FACTOR)

    #endif /* (GPS_UART_MODE_IRDA == GPS_UART_SUB_MODE) */

    #define GPS_UART_DEFAULT_CTRL \
                                (GPS_GET_CTRL_ADDR_ACCEPT(GPS_UART_MP_ACCEPT_ADDRESS) | \
                                 GPS_DEFAULT_CTRL_OVS                                              | \
                                 GPS_CTRL_UART)

    #define GPS_UART_DEFAULT_UART_CTRL \
                                    (GPS_GET_UART_CTRL_MODE(GPS_UART_SUB_MODE))

    /* RX direction */
    #define GPS_UART_DEFAULT_RX_CTRL_PARITY \
                                ((GPS_UART_PARITY_NONE != GPS_UART_PARITY_TYPE) ?      \
                                  (GPS_GET_UART_RX_CTRL_PARITY(GPS_UART_PARITY_TYPE) | \
                                   GPS_UART_RX_CTRL_PARITY_ENABLED) : (0u))

    #define GPS_UART_DEFAULT_UART_RX_CTRL \
                    (GPS_GET_UART_RX_CTRL_MODE(GPS_UART_STOP_BITS_NUM)                    | \
                     GPS_GET_UART_RX_CTRL_POLARITY(GPS_UART_IRDA_POLARITY)                | \
                     GPS_GET_UART_RX_CTRL_MP_MODE(GPS_UART_MP_MODE_ENABLE)                | \
                     GPS_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(GPS_UART_DROP_ON_PARITY_ERR) | \
                     GPS_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(GPS_UART_DROP_ON_FRAME_ERR)   | \
                     GPS_UART_DEFAULT_RX_CTRL_PARITY)

    #define GPS_UART_DEFAULT_RX_CTRL \
                                (GPS_GET_RX_CTRL_DATA_WIDTH(GPS_UART_DATA_BITS_NUM)        | \
                                 GPS_GET_RX_CTRL_MEDIAN    (GPS_UART_MEDIAN_FILTER_ENABLE) | \
                                 GPS_GET_UART_RX_CTRL_ENABLED(GPS_UART_DIRECTION))

    #define GPS_UART_DEFAULT_RX_FIFO_CTRL \
                                GPS_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(GPS_UART_RX_TRIGGER_LEVEL)

    #define GPS_UART_DEFAULT_RX_MATCH_REG  ((0u != GPS_UART_MP_MODE_ENABLE) ?          \
                                (GPS_GET_RX_MATCH_ADDR(GPS_UART_MP_RX_ADDRESS) | \
                                 GPS_GET_RX_MATCH_MASK(GPS_UART_MP_RX_ADDRESS_MASK)) : (0u))

    /* TX direction */
    #define GPS_UART_DEFAULT_TX_CTRL_PARITY (GPS_UART_DEFAULT_RX_CTRL_PARITY)

    #define GPS_UART_DEFAULT_UART_TX_CTRL \
                                (GPS_GET_UART_TX_CTRL_MODE(GPS_UART_STOP_BITS_NUM)       | \
                                 GPS_GET_UART_TX_CTRL_RETRY_NACK(GPS_UART_RETRY_ON_NACK) | \
                                 GPS_UART_DEFAULT_TX_CTRL_PARITY)

    #define GPS_UART_DEFAULT_TX_CTRL \
                                (GPS_GET_TX_CTRL_DATA_WIDTH(GPS_UART_DATA_BITS_NUM) | \
                                 GPS_GET_UART_TX_CTRL_ENABLED(GPS_UART_DIRECTION))

    #define GPS_UART_DEFAULT_TX_FIFO_CTRL \
                                GPS_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(GPS_UART_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define GPS_UART_DEFAULT_INTR_I2C_EC_MASK  (GPS_NO_INTR_SOURCES)
    #define GPS_UART_DEFAULT_INTR_SPI_EC_MASK  (GPS_NO_INTR_SOURCES)
    #define GPS_UART_DEFAULT_INTR_SLAVE_MASK   (GPS_NO_INTR_SOURCES)
    #define GPS_UART_DEFAULT_INTR_MASTER_MASK  (GPS_NO_INTR_SOURCES)
    #define GPS_UART_DEFAULT_INTR_RX_MASK      (GPS_UART_INTR_RX_MASK)
    #define GPS_UART_DEFAULT_INTR_TX_MASK      (GPS_UART_INTR_TX_MASK)

#endif /* (GPS_SCB_MODE_UART_CONST_CFG) */

#endif /* CY_SCB_SPI_UART_GPS_H */


/* [] END OF FILE */
