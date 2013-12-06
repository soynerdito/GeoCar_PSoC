/*******************************************************************************
* File Name: PSOC5_SPI_UART.h
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

#if !defined(CY_SCB_SPI_UART_PSOC5_H)
#define CY_SCB_SPI_UART_PSOC5_H

#include "PSOC5.h"


/***************************************
*   SPI Initial Parameter Constants
****************************************/

#define PSOC5_SPI_MODE                   (0u)
#define PSOC5_SPI_SUB_MODE               (0u)
#define PSOC5_SPI_CLOCK_MODE             (0u)
#define PSOC5_SPI_OVS_FACTOR             (16u)
#define PSOC5_SPI_MEDIAN_FILTER_ENABLE   (0u)
#define PSOC5_SPI_LATE_MISO_SAMPLE_ENABLE (0u)
#define PSOC5_SPI_RX_DATA_BITS_NUM       (8u)
#define PSOC5_SPI_TX_DATA_BITS_NUM       (8u)
#define PSOC5_SPI_WAKE_ENABLE            (0u)
#define PSOC5_SPI_BITS_ORDER             (1u)
#define PSOC5_SPI_TRANSFER_SEPARATION    (1u)
#define PSOC5_SPI_NUMBER_OF_SS_LINES     (1u)
#define PSOC5_SPI_RX_BUFFER_SIZE         (8u)
#define PSOC5_SPI_TX_BUFFER_SIZE         (8u)

#define PSOC5_SPI_INTERRUPT_MODE         (0u)

#define PSOC5_SPI_INTR_RX_MASK           (0u)
#define PSOC5_SPI_INTR_TX_MASK           (0u)

#define PSOC5_SPI_RX_TRIGGER_LEVEL       (7u)
#define PSOC5_SPI_TX_TRIGGER_LEVEL       (0u)


/***************************************
*   UART Initial Parameter Constants
****************************************/

#define PSOC5_UART_SUB_MODE              (0u)
#define PSOC5_UART_DIRECTION             (3u)
#define PSOC5_UART_DATA_BITS_NUM         (8u)
#define PSOC5_UART_PARITY_TYPE           (2u)
#define PSOC5_UART_STOP_BITS_NUM         (2u)
#define PSOC5_UART_OVS_FACTOR            (12u)
#define PSOC5_UART_IRDA_LOW_POWER        (0u)
#define PSOC5_UART_MEDIAN_FILTER_ENABLE  (0u)
#define PSOC5_UART_RETRY_ON_NACK         (0u)
#define PSOC5_UART_IRDA_POLARITY         (0u)
#define PSOC5_UART_DROP_ON_FRAME_ERR     (0u)
#define PSOC5_UART_DROP_ON_PARITY_ERR    (0u)
#define PSOC5_UART_WAKE_ENABLE           (0u)
#define PSOC5_UART_RX_BUFFER_SIZE        (8u)
#define PSOC5_UART_TX_BUFFER_SIZE        (8u)
#define PSOC5_UART_MP_MODE_ENABLE        (0u)
#define PSOC5_UART_MP_ACCEPT_ADDRESS     (0u)
#define PSOC5_UART_MP_RX_ADDRESS         (2u)
#define PSOC5_UART_MP_RX_ADDRESS_MASK    (255u)

#define PSOC5_UART_INTERRUPT_MODE        (0u)

#define PSOC5_UART_INTR_RX_MASK          (0u)
#define PSOC5_UART_INTR_TX_MASK          (0u)

#define PSOC5_UART_RX_TRIGGER_LEVEL      (7u)
#define PSOC5_UART_TX_TRIGGER_LEVEL      (0u)

/* Sources of RX errors */
#define PSOC5_INTR_RX_ERR        (PSOC5_INTR_RX_OVERFLOW    | \
                                             PSOC5_INTR_RX_UNDERFLOW   | \
                                             PSOC5_INTR_RX_FRAME_ERROR | \
                                             PSOC5_INTR_RX_PARITY_ERROR)

/* UART direction enum */
#define PSOC5_UART_RX    (1u)
#define PSOC5_UART_TX    (2u)
#define PSOC5_UART_TX_RX (3u)


/***************************************
*   Conditional Compilation Parameters
****************************************/

#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)

    /* Direction */
    #define PSOC5_RX_DIRECTION           (1u)
    #define PSOC5_TX_DIRECTION           (1u)
    #define PSOC5_UART_RX_DIRECTION      (1u)
    #define PSOC5_UART_TX_DIRECTION      (1u)

    /* Only external RX and TX buffer for uncofigured mode */
    #define PSOC5_INTERNAL_RX_SW_BUFFER   (0u)
    #define PSOC5_INTERNAL_TX_SW_BUFFER   (0u)

    /* Get RX and TX buffer size */
    #define PSOC5_RX_BUFFER_SIZE (PSOC5_rxBufferSize)
    #define PSOC5_TX_BUFFER_SIZE (PSOC5_txBufferSize)

    /* Return true if buffer is provided */
    #define PSOC5_CHECK_RX_SW_BUFFER (NULL != PSOC5_rxBuffer)
    #define PSOC5_CHECK_TX_SW_BUFFER (NULL != PSOC5_txBuffer)

    /* Alwasy provde global variables to support RX and TX buffers */
    #define PSOC5_INTERNAL_RX_SW_BUFFER_CONST    (1u)
    #define PSOC5_INTERNAL_TX_SW_BUFFER_CONST    (1u)

    /* Get wakeup enable option */
    #define PSOC5_SPI_WAKE_ENABLE_CONST  (1u)
    #define PSOC5_CHECK_SPI_WAKE_ENABLE  (0u != PSOC5_scbEnableWake)
    #define PSOC5_UART_WAKE_ENABLE_CONST (1u)

#else

    /* SPI internal RX and TX buffers */
    #define PSOC5_INTERNAL_SPI_RX_SW_BUFFER  (PSOC5_SPI_RX_BUFFER_SIZE > \
                                                                                            PSOC5_FIFO_SIZE)
    #define PSOC5_INTERNAL_SPI_TX_SW_BUFFER  (PSOC5_SPI_TX_BUFFER_SIZE > \
                                                                                            PSOC5_FIFO_SIZE)

    /* UART internal RX and TX buffers */
    #define PSOC5_INTERNAL_UART_RX_SW_BUFFER  (PSOC5_UART_RX_BUFFER_SIZE > \
                                                                                            PSOC5_FIFO_SIZE)
    #define PSOC5_INTERNAL_UART_TX_SW_BUFFER  (PSOC5_UART_TX_BUFFER_SIZE > \
                                                                                            PSOC5_FIFO_SIZE)

    /* SPI Direction */
    #define PSOC5_SPI_RX_DIRECTION (1u)
    #define PSOC5_SPI_TX_DIRECTION (1u)

    /* UART Direction */
    #define PSOC5_UART_RX_DIRECTION (0u != (PSOC5_UART_DIRECTION & PSOC5_UART_RX))
    #define PSOC5_UART_TX_DIRECTION (0u != (PSOC5_UART_DIRECTION & PSOC5_UART_TX))

    /* Direction */
    #define PSOC5_RX_DIRECTION ((PSOC5_SCB_MODE_SPI_CONST_CFG) ? \
                                            (PSOC5_SPI_RX_DIRECTION) : (PSOC5_UART_RX_DIRECTION))

    #define PSOC5_TX_DIRECTION ((PSOC5_SCB_MODE_SPI_CONST_CFG) ? \
                                            (PSOC5_SPI_TX_DIRECTION) : (PSOC5_UART_TX_DIRECTION))

    /* Internal RX and TX buffer: for SPI or UART */
    #if(PSOC5_SCB_MODE_SPI_CONST_CFG)

        /* Internal SPI RX and TX buffer */
        #define PSOC5_INTERNAL_RX_SW_BUFFER  (PSOC5_INTERNAL_SPI_RX_SW_BUFFER)
        #define PSOC5_INTERNAL_TX_SW_BUFFER  (PSOC5_INTERNAL_SPI_TX_SW_BUFFER)

        /* Internal SPI RX and TX buffer size */
        #define PSOC5_RX_BUFFER_SIZE         (PSOC5_SPI_RX_BUFFER_SIZE + 1u)
        #define PSOC5_TX_BUFFER_SIZE         (PSOC5_SPI_TX_BUFFER_SIZE)
        
        /* Get wakeup enable option */
        #define PSOC5_SPI_WAKE_ENABLE_CONST  (0u != PSOC5_SPI_WAKE_ENABLE)
        #define PSOC5_UART_WAKE_ENABLE_CONST (0u)

    #else

        /* Internal UART RX and TX buffer */
        #define PSOC5_INTERNAL_RX_SW_BUFFER  (PSOC5_INTERNAL_UART_RX_SW_BUFFER)
        #define PSOC5_INTERNAL_TX_SW_BUFFER  (PSOC5_INTERNAL_UART_TX_SW_BUFFER)

        /* Internal UART RX and TX buffer size */
        #define PSOC5_RX_BUFFER_SIZE         (PSOC5_UART_RX_BUFFER_SIZE + 1u)
        #define PSOC5_TX_BUFFER_SIZE         (PSOC5_UART_TX_BUFFER_SIZE)
        
        /* Get wakeup enable option */
        #define PSOC5_SPI_WAKE_ENABLE_CONST  (0u)
        #define PSOC5_UART_WAKE_ENABLE_CONST (0u != PSOC5_UART_WAKE_ENABLE)
    #endif /* (PSOC5_SCB_MODE_SPI_CONST_CFG) */

    /* Internal RX and TX buffer: for SPI or UART. Used in conditional compilation check */
    #define PSOC5_CHECK_RX_SW_BUFFER (PSOC5_INTERNAL_RX_SW_BUFFER)
    #define PSOC5_CHECK_TX_SW_BUFFER (PSOC5_INTERNAL_TX_SW_BUFFER)

    /* Provide global variables to support RX and TX buffers */
    #define PSOC5_INTERNAL_RX_SW_BUFFER_CONST    (PSOC5_INTERNAL_RX_SW_BUFFER)
    #define PSOC5_INTERNAL_TX_SW_BUFFER_CONST    (PSOC5_INTERNAL_TX_SW_BUFFER)
    
    /* Wakeup for SPI */
    #define PSOC5_CHECK_SPI_WAKE_ENABLE  (PSOC5_SPI_WAKE_ENABLE_CONST)

#endif /* End (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Bootloader communication interface enable: NOT supported yet */
#define PSOC5_SPI_BTLDR_COMM_ENABLED   ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

#define PSOC5_UART_BTLDR_COMM_ENABLED   ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))


/***************************************
*       Type Definitions
***************************************/

/* PSOC5_SPI_INIT_STRUCT */
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
} PSOC5_SPI_INIT_STRUCT;

/* PSOC5_UART_INIT_STRUCT */
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
} PSOC5_UART_INIT_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/* SPI specific functions */
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    void PSOC5_SpiInit(const PSOC5_SPI_INIT_STRUCT *config);
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(PSOC5_SCB_MODE_SPI_INC)
    void PSOC5_SpiSetActiveSlaveSelect(uint32 activeSelect);
#endif /* (PSOC5_SCB_MODE_SPI_INC) */

/* UART specific functions */
#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    void PSOC5_UartInit(const PSOC5_UART_INIT_STRUCT *config);
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(PSOC5_SCB_MODE_UART_INC)
    void PSOC5_UartSetRxAddress(uint32 address);
    void PSOC5_UartSetRxAddressMask(uint32 addressMask);
#endif /* (PSOC5_SCB_MODE_UART_INC) */

/* UART RX direction APIs */
#if(PSOC5_UART_RX_DIRECTION)
    uint32 PSOC5_UartGetChar(void);
    uint32 PSOC5_UartGetByte(void);
#endif /* (PSOC5_UART_RX_DIRECTION) */

/* UART TX direction APIs */
#if(PSOC5_UART_TX_DIRECTION)
    #define PSOC5_UartPutChar(ch)    PSOC5_SpiUartWriteTxData((uint32)(ch))
    void PSOC5_UartPutString(const char8 string[]);
    void PSOC5_UartPutCRLF(uint32 txDataByte);
#endif /* (PSOC5_UART_TX_DIRECTION) */

/* Common APIs Rx direction */
#if(PSOC5_RX_DIRECTION)
    uint32 PSOC5_SpiUartReadRxData(void);
    uint32 PSOC5_SpiUartGetRxBufferSize(void);
    void   PSOC5_SpiUartClearRxBuffer(void);
#endif /* (PSOC5_RX_DIRECTION) */

/* Common APIs Tx direction */
#if(PSOC5_TX_DIRECTION)
    void   PSOC5_SpiUartWriteTxData(uint32 txDataByte);
    void   PSOC5_SpiUartPutArray(const uint8 wrBuf[], uint32 count);
    void   PSOC5_SpiUartClearTxBuffer(void);
    uint32 PSOC5_SpiUartGetTxBufferSize(void);
#endif /* (PSOC5_TX_DIRECTION) */

CY_ISR_PROTO(PSOC5_SPI_UART_ISR);

#if(PSOC5_UART_RX_WAKEUP_IRQ)
    CY_ISR_PROTO(PSOC5_UART_WAKEUP_ISR);
#endif /* (PSOC5_UART_RX_WAKEUP_IRQ) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PSOC5_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void PSOC5_SpiCyBtldrCommStart(void);
    void PSOC5_SpiCyBtldrCommStop (void);
    void PSOC5_SpiCyBtldrCommReset(void);
    cystatus PSOC5_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus PSOC5_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PSOC5_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PSOC5_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void PSOC5_UartCyBtldrCommStart(void);
    void PSOC5_UartCyBtldrCommStop (void);
    void PSOC5_UartCyBtldrCommReset(void);
    cystatus PSOC5_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus PSOC5_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (PSOC5_UART_BTLDR_COMM_ENABLED) */


/***************************************
*     Buffer Access Macro Definitions
***************************************/

#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    /* RX direction */
    void   PSOC5_PutWordInRxBuffer  (uint32 idx, uint32 rxDataByte);
    uint32 PSOC5_GetWordFromRxBuffer(uint32 idx);

    /* TX direction */
    void   PSOC5_PutWordInTxBuffer  (uint32 idx, uint32 txDataByte);
    uint32 PSOC5_GetWordFromTxBuffer(uint32 idx);

#else

    /* RX direction */
    #if(PSOC5_INTERNAL_RX_SW_BUFFER_CONST)
        #define PSOC5_PutWordInRxBuffer(idx, rxDataByte) \
                do{                                                 \
                    PSOC5_rxBufferInternal[(idx)] = ((uint8) (rxDataByte)); \
                }while(0)

        #define PSOC5_GetWordFromRxBuffer(idx) PSOC5_rxBufferInternal[(idx)]

    #endif /* (PSOC5_INTERNAL_RX_SW_BUFFER_CONST) */

    /* TX direction */
    #if(PSOC5_INTERNAL_TX_SW_BUFFER_CONST)
        #define PSOC5_PutWordInTxBuffer(idx, txDataByte) \
                    do{                                             \
                        PSOC5_txBufferInternal[(idx)] = ((uint8) (txDataByte)); \
                    }while(0)

        #define PSOC5_GetWordFromTxBuffer(idx) PSOC5_txBufferInternal[(idx)]

    #endif /* (PSOC5_INTERNAL_TX_SW_BUFFER_CONST) */

#endif /* (PSOC5_TX_SW_BUFFER_ENABLE) */


/***************************************
*         SPI API Constants
***************************************/

/* SPI mode enum */
#define PSOC5_SPI_SLAVE  (0u)
#define PSOC5_SPI_MASTER (1u)

/* SPI sub mode enum */
#define PSOC5_SPI_MODE_MOTOROLA      (0x00u)
#define PSOC5_SPI_MODE_TI_COINCIDES  (0x01u)
#define PSOC5_SPI_MODE_TI_PRECEDES   (0x11u)
#define PSOC5_SPI_MODE_NATIONAL      (0x02u)
#define PSOC5_SPI_MODE_MASK          (0x03u)
#define PSOC5_SPI_MODE_TI_PRECEDES_MASK  (0x10u)

/* SPI phase and polarity mode enum */
#define PSOC5_SPI_SCLK_CPHA0_CPOL0   (0x00u)
#define PSOC5_SPI_SCLK_CPHA0_CPOL1   (0x02u)
#define PSOC5_SPI_SCLK_CPHA1_CPOL0   (0x01u)
#define PSOC5_SPI_SCLK_CPHA1_CPOL1   (0x03u)

/* SPI bits order enum */
#define PSOC5_BITS_ORDER_LSB_FIRST   (0u)
#define PSOC5_BITS_ORDER_MSB_FIRST   (1u)

/* SPI transfer separation enum */
#define PSOC5_SPI_TRANSFER_SEPARATED     (0u)
#define PSOC5_SPI_TRANSFER_CONTINUOUS    (1u)

/* "activeSS" constants for SpiSetActiveSlaveSelect() function */
#define PSOC5_SPIM_ACTIVE_SS0    (0x00u)
#define PSOC5_SPIM_ACTIVE_SS1    (0x01u)
#define PSOC5_SPIM_ACTIVE_SS2    (0x02u)
#define PSOC5_SPIM_ACTIVE_SS3    (0x03u)


/***************************************
*         UART API Constants
***************************************/

/* UART sub-modes enum */
#define PSOC5_UART_MODE_STD          (0u)
#define PSOC5_UART_MODE_SMARTCARD    (1u)
#define PSOC5_UART_MODE_IRDA         (2u)

/* UART direction enum */
#define PSOC5_UART_RX    (1u)
#define PSOC5_UART_TX    (2u)
#define PSOC5_UART_TX_RX (3u)

/* UART parity enum */
#define PSOC5_UART_PARITY_EVEN   (0u)
#define PSOC5_UART_PARITY_ODD    (1u)
#define PSOC5_UART_PARITY_NONE   (2u)

/* UART stop bits enum */
#define PSOC5_UART_STOP_BITS_1   (1u)
#define PSOC5_UART_STOP_BITS_1_5 (2u)
#define PSOC5_UART_STOP_BITS_2   (3u)

/* UART IrDA low power OVS enum */
#define PSOC5_UART_IRDA_LP_OVS16     (16u)
#define PSOC5_UART_IRDA_LP_OVS32     (32u)
#define PSOC5_UART_IRDA_LP_OVS48     (48u)
#define PSOC5_UART_IRDA_LP_OVS96     (96u)
#define PSOC5_UART_IRDA_LP_OVS192    (192u)
#define PSOC5_UART_IRDA_LP_OVS768    (768u)
#define PSOC5_UART_IRDA_LP_OVS1536   (1536u)

/* Uart MP: mark (address) and space (data) bit definitions */
#define PSOC5_UART_MP_MARK       (0x100u)
#define PSOC5_UART_MP_SPACE      (0x000u)


/***************************************
*     Vars with External Linkage
***************************************/

#if(PSOC5_SCB_MODE_UNCONFIG_CONST_CFG)
    extern const PSOC5_SPI_INIT_STRUCT  PSOC5_configSpi;
    extern const PSOC5_UART_INIT_STRUCT PSOC5_configUart;
#endif /* (PSOC5_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*    Specific SPI Macro Definitions
***************************************/

#define PSOC5_GET_SPI_INTR_SLAVE_MASK(sourceMask)  ((sourceMask) & PSOC5_INTR_SLAVE_SPI_BUS_ERROR)
#define PSOC5_GET_SPI_INTR_MASTER_MASK(sourceMask) ((sourceMask) & PSOC5_INTR_MASTER_SPI_DONE)
#define PSOC5_GET_SPI_INTR_RX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~PSOC5_INTR_SLAVE_SPI_BUS_ERROR)

#define PSOC5_GET_SPI_INTR_TX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~PSOC5_INTR_MASTER_SPI_DONE)


/***************************************
*    Specific UART Macro Definitions
***************************************/

#define PSOC5_UART_GET_CTRL_OVS_IRDA_LP(oversample) \
        ((PSOC5_UART_IRDA_LP_OVS16   == (oversample)) ? PSOC5_CTRL_OVS_IRDA_LP_OVS16 : \
         ((PSOC5_UART_IRDA_LP_OVS32   == (oversample)) ? PSOC5_CTRL_OVS_IRDA_LP_OVS32 : \
          ((PSOC5_UART_IRDA_LP_OVS48   == (oversample)) ? PSOC5_CTRL_OVS_IRDA_LP_OVS48 : \
           ((PSOC5_UART_IRDA_LP_OVS96   == (oversample)) ? PSOC5_CTRL_OVS_IRDA_LP_OVS96 : \
            ((PSOC5_UART_IRDA_LP_OVS192  == (oversample)) ? PSOC5_CTRL_OVS_IRDA_LP_OVS192 : \
             ((PSOC5_UART_IRDA_LP_OVS768  == (oversample)) ? PSOC5_CTRL_OVS_IRDA_LP_OVS768 : \
              ((PSOC5_UART_IRDA_LP_OVS1536 == (oversample)) ? PSOC5_CTRL_OVS_IRDA_LP_OVS1536 : \
                                                                          PSOC5_CTRL_OVS_IRDA_LP_OVS16)))))))

#define PSOC5_GET_UART_RX_CTRL_ENABLED(direction) ((0u != (PSOC5_UART_RX & (direction))) ? \
                                                                    (PSOC5_RX_CTRL_ENABLED) : (0u))

#define PSOC5_GET_UART_TX_CTRL_ENABLED(direction) ((0u != (PSOC5_UART_TX & (direction))) ? \
                                                                    (PSOC5_TX_CTRL_ENABLED) : (0u))


/***************************************
*        SPI Register Settings
***************************************/

#define PSOC5_CTRL_SPI      (PSOC5_CTRL_MODE_SPI)
#define PSOC5_SPI_RX_CTRL   (PSOC5_RX_CTRL_ENABLED)
#define PSOC5_SPI_TX_CTRL   (PSOC5_TX_CTRL_ENABLED)


/***************************************
*       SPI Init Register Settings
***************************************/

#if(PSOC5_SCB_MODE_SPI_CONST_CFG)

    /* SPI Configuration */
    #define PSOC5_SPI_DEFAULT_CTRL \
                    (PSOC5_GET_CTRL_OVS(PSOC5_SPI_OVS_FACTOR)         | \
                     PSOC5_GET_CTRL_EC_AM_MODE(PSOC5_SPI_WAKE_ENABLE) | \
                     PSOC5_CTRL_SPI)

    #define PSOC5_SPI_DEFAULT_SPI_CTRL \
                    (PSOC5_GET_SPI_CTRL_CONTINUOUS    (PSOC5_SPI_TRANSFER_SEPARATION)       | \
                     PSOC5_GET_SPI_CTRL_SELECT_PRECEDE(PSOC5_SPI_SUB_MODE &                   \
                                                                  PSOC5_SPI_MODE_TI_PRECEDES_MASK)     | \
                     PSOC5_GET_SPI_CTRL_SCLK_MODE     (PSOC5_SPI_CLOCK_MODE)                | \
                     PSOC5_GET_SPI_CTRL_LATE_MISO_SAMPLE(PSOC5_SPI_LATE_MISO_SAMPLE_ENABLE) | \
                     PSOC5_GET_SPI_CTRL_SUB_MODE      (PSOC5_SPI_SUB_MODE)                  | \
                     PSOC5_GET_SPI_CTRL_MASTER_MODE   (PSOC5_SPI_MODE))

    /* RX direction */
    #define PSOC5_SPI_DEFAULT_RX_CTRL \
                    (PSOC5_GET_RX_CTRL_DATA_WIDTH(PSOC5_SPI_RX_DATA_BITS_NUM)     | \
                     PSOC5_GET_RX_CTRL_BIT_ORDER (PSOC5_SPI_BITS_ORDER)           | \
                     PSOC5_GET_RX_CTRL_MEDIAN    (PSOC5_SPI_MEDIAN_FILTER_ENABLE) | \
                     PSOC5_SPI_RX_CTRL)

    #define PSOC5_SPI_DEFAULT_RX_FIFO_CTRL \
                    PSOC5_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(PSOC5_SPI_RX_TRIGGER_LEVEL)

    /* TX direction */
    #define PSOC5_SPI_DEFAULT_TX_CTRL \
                    (PSOC5_GET_TX_CTRL_DATA_WIDTH(PSOC5_SPI_TX_DATA_BITS_NUM) | \
                     PSOC5_GET_TX_CTRL_BIT_ORDER (PSOC5_SPI_BITS_ORDER)       | \
                     PSOC5_SPI_TX_CTRL)

    #define PSOC5_SPI_DEFAULT_TX_FIFO_CTRL \
                    PSOC5_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(PSOC5_SPI_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define PSOC5_SPI_DEFAULT_INTR_SPI_EC_MASK   (PSOC5_NO_INTR_SOURCES)

    #define PSOC5_SPI_DEFAULT_INTR_I2C_EC_MASK   (PSOC5_NO_INTR_SOURCES)
    #define PSOC5_SPI_DEFAULT_INTR_SLAVE_MASK \
                    (PSOC5_SPI_INTR_RX_MASK & PSOC5_INTR_SLAVE_SPI_BUS_ERROR)

    #define PSOC5_SPI_DEFAULT_INTR_MASTER_MASK \
                    (PSOC5_SPI_INTR_TX_MASK & PSOC5_INTR_MASTER_SPI_DONE)

    #define PSOC5_SPI_DEFAULT_INTR_RX_MASK \
                    (PSOC5_SPI_INTR_RX_MASK & (uint32) ~PSOC5_INTR_SLAVE_SPI_BUS_ERROR)

    #define PSOC5_SPI_DEFAULT_INTR_TX_MASK \
                    (PSOC5_SPI_INTR_TX_MASK & (uint32) ~PSOC5_INTR_MASTER_SPI_DONE)

#endif /* (PSOC5_SCB_MODE_SPI_CONST_CFG) */


/***************************************
*        UART Register Settings
***************************************/

#define PSOC5_CTRL_UART      (PSOC5_CTRL_MODE_UART)
#define PSOC5_UART_RX_CTRL   (PSOC5_RX_CTRL_LSB_FIRST) /* LSB for UART goes first */
#define PSOC5_UART_TX_CTRL   (PSOC5_TX_CTRL_LSB_FIRST) /* LSB for UART goes first */


/***************************************
*      UART Init Register Settings
***************************************/

#if(PSOC5_SCB_MODE_UART_CONST_CFG)

    /* UART configuration */
    #if(PSOC5_UART_MODE_IRDA == PSOC5_UART_SUB_MODE)

        #define PSOC5_DEFAULT_CTRL_OVS   ((0u != PSOC5_UART_IRDA_LOW_POWER) ?              \
                                (PSOC5_UART_GET_CTRL_OVS_IRDA_LP(PSOC5_UART_OVS_FACTOR)) : \
                                (PSOC5_CTRL_OVS_IRDA_OVS16))

    #else

        #define PSOC5_DEFAULT_CTRL_OVS   PSOC5_GET_CTRL_OVS(PSOC5_UART_OVS_FACTOR)

    #endif /* (PSOC5_UART_MODE_IRDA == PSOC5_UART_SUB_MODE) */

    #define PSOC5_UART_DEFAULT_CTRL \
                                (PSOC5_GET_CTRL_ADDR_ACCEPT(PSOC5_UART_MP_ACCEPT_ADDRESS) | \
                                 PSOC5_DEFAULT_CTRL_OVS                                              | \
                                 PSOC5_CTRL_UART)

    #define PSOC5_UART_DEFAULT_UART_CTRL \
                                    (PSOC5_GET_UART_CTRL_MODE(PSOC5_UART_SUB_MODE))

    /* RX direction */
    #define PSOC5_UART_DEFAULT_RX_CTRL_PARITY \
                                ((PSOC5_UART_PARITY_NONE != PSOC5_UART_PARITY_TYPE) ?      \
                                  (PSOC5_GET_UART_RX_CTRL_PARITY(PSOC5_UART_PARITY_TYPE) | \
                                   PSOC5_UART_RX_CTRL_PARITY_ENABLED) : (0u))

    #define PSOC5_UART_DEFAULT_UART_RX_CTRL \
                    (PSOC5_GET_UART_RX_CTRL_MODE(PSOC5_UART_STOP_BITS_NUM)                    | \
                     PSOC5_GET_UART_RX_CTRL_POLARITY(PSOC5_UART_IRDA_POLARITY)                | \
                     PSOC5_GET_UART_RX_CTRL_MP_MODE(PSOC5_UART_MP_MODE_ENABLE)                | \
                     PSOC5_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(PSOC5_UART_DROP_ON_PARITY_ERR) | \
                     PSOC5_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(PSOC5_UART_DROP_ON_FRAME_ERR)   | \
                     PSOC5_UART_DEFAULT_RX_CTRL_PARITY)

    #define PSOC5_UART_DEFAULT_RX_CTRL \
                                (PSOC5_GET_RX_CTRL_DATA_WIDTH(PSOC5_UART_DATA_BITS_NUM)        | \
                                 PSOC5_GET_RX_CTRL_MEDIAN    (PSOC5_UART_MEDIAN_FILTER_ENABLE) | \
                                 PSOC5_GET_UART_RX_CTRL_ENABLED(PSOC5_UART_DIRECTION))

    #define PSOC5_UART_DEFAULT_RX_FIFO_CTRL \
                                PSOC5_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(PSOC5_UART_RX_TRIGGER_LEVEL)

    #define PSOC5_UART_DEFAULT_RX_MATCH_REG  ((0u != PSOC5_UART_MP_MODE_ENABLE) ?          \
                                (PSOC5_GET_RX_MATCH_ADDR(PSOC5_UART_MP_RX_ADDRESS) | \
                                 PSOC5_GET_RX_MATCH_MASK(PSOC5_UART_MP_RX_ADDRESS_MASK)) : (0u))

    /* TX direction */
    #define PSOC5_UART_DEFAULT_TX_CTRL_PARITY (PSOC5_UART_DEFAULT_RX_CTRL_PARITY)

    #define PSOC5_UART_DEFAULT_UART_TX_CTRL \
                                (PSOC5_GET_UART_TX_CTRL_MODE(PSOC5_UART_STOP_BITS_NUM)       | \
                                 PSOC5_GET_UART_TX_CTRL_RETRY_NACK(PSOC5_UART_RETRY_ON_NACK) | \
                                 PSOC5_UART_DEFAULT_TX_CTRL_PARITY)

    #define PSOC5_UART_DEFAULT_TX_CTRL \
                                (PSOC5_GET_TX_CTRL_DATA_WIDTH(PSOC5_UART_DATA_BITS_NUM) | \
                                 PSOC5_GET_UART_TX_CTRL_ENABLED(PSOC5_UART_DIRECTION))

    #define PSOC5_UART_DEFAULT_TX_FIFO_CTRL \
                                PSOC5_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(PSOC5_UART_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define PSOC5_UART_DEFAULT_INTR_I2C_EC_MASK  (PSOC5_NO_INTR_SOURCES)
    #define PSOC5_UART_DEFAULT_INTR_SPI_EC_MASK  (PSOC5_NO_INTR_SOURCES)
    #define PSOC5_UART_DEFAULT_INTR_SLAVE_MASK   (PSOC5_NO_INTR_SOURCES)
    #define PSOC5_UART_DEFAULT_INTR_MASTER_MASK  (PSOC5_NO_INTR_SOURCES)
    #define PSOC5_UART_DEFAULT_INTR_RX_MASK      (PSOC5_UART_INTR_RX_MASK)
    #define PSOC5_UART_DEFAULT_INTR_TX_MASK      (PSOC5_UART_INTR_TX_MASK)

#endif /* (PSOC5_SCB_MODE_UART_CONST_CFG) */

#endif /* CY_SCB_SPI_UART_PSOC5_H */


/* [] END OF FILE */
