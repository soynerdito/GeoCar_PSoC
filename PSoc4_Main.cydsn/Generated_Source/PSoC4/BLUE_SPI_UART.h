/*******************************************************************************
* File Name: BLUE_SPI_UART.h
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

#if !defined(CY_SCB_SPI_UART_BLUE_H)
#define CY_SCB_SPI_UART_BLUE_H

#include "BLUE.h"


/***************************************
*   SPI Initial Parameter Constants
****************************************/

#define BLUE_SPI_MODE                   (0u)
#define BLUE_SPI_SUB_MODE               (0u)
#define BLUE_SPI_CLOCK_MODE             (0u)
#define BLUE_SPI_OVS_FACTOR             (16u)
#define BLUE_SPI_MEDIAN_FILTER_ENABLE   (0u)
#define BLUE_SPI_LATE_MISO_SAMPLE_ENABLE (0u)
#define BLUE_SPI_RX_DATA_BITS_NUM       (8u)
#define BLUE_SPI_TX_DATA_BITS_NUM       (8u)
#define BLUE_SPI_WAKE_ENABLE            (0u)
#define BLUE_SPI_BITS_ORDER             (1u)
#define BLUE_SPI_TRANSFER_SEPARATION    (1u)
#define BLUE_SPI_NUMBER_OF_SS_LINES     (1u)
#define BLUE_SPI_RX_BUFFER_SIZE         (8u)
#define BLUE_SPI_TX_BUFFER_SIZE         (8u)

#define BLUE_SPI_INTERRUPT_MODE         (0u)

#define BLUE_SPI_INTR_RX_MASK           (0u)
#define BLUE_SPI_INTR_TX_MASK           (0u)

#define BLUE_SPI_RX_TRIGGER_LEVEL       (7u)
#define BLUE_SPI_TX_TRIGGER_LEVEL       (0u)


/***************************************
*   UART Initial Parameter Constants
****************************************/

#define BLUE_UART_SUB_MODE              (0u)
#define BLUE_UART_DIRECTION             (3u)
#define BLUE_UART_DATA_BITS_NUM         (8u)
#define BLUE_UART_PARITY_TYPE           (2u)
#define BLUE_UART_STOP_BITS_NUM         (2u)
#define BLUE_UART_OVS_FACTOR            (12u)
#define BLUE_UART_IRDA_LOW_POWER        (0u)
#define BLUE_UART_MEDIAN_FILTER_ENABLE  (0u)
#define BLUE_UART_RETRY_ON_NACK         (0u)
#define BLUE_UART_IRDA_POLARITY         (0u)
#define BLUE_UART_DROP_ON_FRAME_ERR     (0u)
#define BLUE_UART_DROP_ON_PARITY_ERR    (0u)
#define BLUE_UART_WAKE_ENABLE           (0u)
#define BLUE_UART_RX_BUFFER_SIZE        (8u)
#define BLUE_UART_TX_BUFFER_SIZE        (8u)
#define BLUE_UART_MP_MODE_ENABLE        (0u)
#define BLUE_UART_MP_ACCEPT_ADDRESS     (0u)
#define BLUE_UART_MP_RX_ADDRESS         (2u)
#define BLUE_UART_MP_RX_ADDRESS_MASK    (255u)

#define BLUE_UART_INTERRUPT_MODE        (0u)

#define BLUE_UART_INTR_RX_MASK          (0u)
#define BLUE_UART_INTR_TX_MASK          (0u)

#define BLUE_UART_RX_TRIGGER_LEVEL      (7u)
#define BLUE_UART_TX_TRIGGER_LEVEL      (0u)

/* Sources of RX errors */
#define BLUE_INTR_RX_ERR        (BLUE_INTR_RX_OVERFLOW    | \
                                             BLUE_INTR_RX_UNDERFLOW   | \
                                             BLUE_INTR_RX_FRAME_ERROR | \
                                             BLUE_INTR_RX_PARITY_ERROR)

/* UART direction enum */
#define BLUE_UART_RX    (1u)
#define BLUE_UART_TX    (2u)
#define BLUE_UART_TX_RX (3u)


/***************************************
*   Conditional Compilation Parameters
****************************************/

#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)

    /* Direction */
    #define BLUE_RX_DIRECTION           (1u)
    #define BLUE_TX_DIRECTION           (1u)
    #define BLUE_UART_RX_DIRECTION      (1u)
    #define BLUE_UART_TX_DIRECTION      (1u)

    /* Only external RX and TX buffer for uncofigured mode */
    #define BLUE_INTERNAL_RX_SW_BUFFER   (0u)
    #define BLUE_INTERNAL_TX_SW_BUFFER   (0u)

    /* Get RX and TX buffer size */
    #define BLUE_RX_BUFFER_SIZE (BLUE_rxBufferSize)
    #define BLUE_TX_BUFFER_SIZE (BLUE_txBufferSize)

    /* Return true if buffer is provided */
    #define BLUE_CHECK_RX_SW_BUFFER (NULL != BLUE_rxBuffer)
    #define BLUE_CHECK_TX_SW_BUFFER (NULL != BLUE_txBuffer)

    /* Alwasy provde global variables to support RX and TX buffers */
    #define BLUE_INTERNAL_RX_SW_BUFFER_CONST    (1u)
    #define BLUE_INTERNAL_TX_SW_BUFFER_CONST    (1u)

    /* Get wakeup enable option */
    #define BLUE_SPI_WAKE_ENABLE_CONST  (1u)
    #define BLUE_CHECK_SPI_WAKE_ENABLE  (0u != BLUE_scbEnableWake)
    #define BLUE_UART_WAKE_ENABLE_CONST (1u)

#else

    /* SPI internal RX and TX buffers */
    #define BLUE_INTERNAL_SPI_RX_SW_BUFFER  (BLUE_SPI_RX_BUFFER_SIZE > \
                                                                                            BLUE_FIFO_SIZE)
    #define BLUE_INTERNAL_SPI_TX_SW_BUFFER  (BLUE_SPI_TX_BUFFER_SIZE > \
                                                                                            BLUE_FIFO_SIZE)

    /* UART internal RX and TX buffers */
    #define BLUE_INTERNAL_UART_RX_SW_BUFFER  (BLUE_UART_RX_BUFFER_SIZE > \
                                                                                            BLUE_FIFO_SIZE)
    #define BLUE_INTERNAL_UART_TX_SW_BUFFER  (BLUE_UART_TX_BUFFER_SIZE > \
                                                                                            BLUE_FIFO_SIZE)

    /* SPI Direction */
    #define BLUE_SPI_RX_DIRECTION (1u)
    #define BLUE_SPI_TX_DIRECTION (1u)

    /* UART Direction */
    #define BLUE_UART_RX_DIRECTION (0u != (BLUE_UART_DIRECTION & BLUE_UART_RX))
    #define BLUE_UART_TX_DIRECTION (0u != (BLUE_UART_DIRECTION & BLUE_UART_TX))

    /* Direction */
    #define BLUE_RX_DIRECTION ((BLUE_SCB_MODE_SPI_CONST_CFG) ? \
                                            (BLUE_SPI_RX_DIRECTION) : (BLUE_UART_RX_DIRECTION))

    #define BLUE_TX_DIRECTION ((BLUE_SCB_MODE_SPI_CONST_CFG) ? \
                                            (BLUE_SPI_TX_DIRECTION) : (BLUE_UART_TX_DIRECTION))

    /* Internal RX and TX buffer: for SPI or UART */
    #if(BLUE_SCB_MODE_SPI_CONST_CFG)

        /* Internal SPI RX and TX buffer */
        #define BLUE_INTERNAL_RX_SW_BUFFER  (BLUE_INTERNAL_SPI_RX_SW_BUFFER)
        #define BLUE_INTERNAL_TX_SW_BUFFER  (BLUE_INTERNAL_SPI_TX_SW_BUFFER)

        /* Internal SPI RX and TX buffer size */
        #define BLUE_RX_BUFFER_SIZE         (BLUE_SPI_RX_BUFFER_SIZE + 1u)
        #define BLUE_TX_BUFFER_SIZE         (BLUE_SPI_TX_BUFFER_SIZE)
        
        /* Get wakeup enable option */
        #define BLUE_SPI_WAKE_ENABLE_CONST  (0u != BLUE_SPI_WAKE_ENABLE)
        #define BLUE_UART_WAKE_ENABLE_CONST (0u)

    #else

        /* Internal UART RX and TX buffer */
        #define BLUE_INTERNAL_RX_SW_BUFFER  (BLUE_INTERNAL_UART_RX_SW_BUFFER)
        #define BLUE_INTERNAL_TX_SW_BUFFER  (BLUE_INTERNAL_UART_TX_SW_BUFFER)

        /* Internal UART RX and TX buffer size */
        #define BLUE_RX_BUFFER_SIZE         (BLUE_UART_RX_BUFFER_SIZE + 1u)
        #define BLUE_TX_BUFFER_SIZE         (BLUE_UART_TX_BUFFER_SIZE)
        
        /* Get wakeup enable option */
        #define BLUE_SPI_WAKE_ENABLE_CONST  (0u)
        #define BLUE_UART_WAKE_ENABLE_CONST (0u != BLUE_UART_WAKE_ENABLE)
    #endif /* (BLUE_SCB_MODE_SPI_CONST_CFG) */

    /* Internal RX and TX buffer: for SPI or UART. Used in conditional compilation check */
    #define BLUE_CHECK_RX_SW_BUFFER (BLUE_INTERNAL_RX_SW_BUFFER)
    #define BLUE_CHECK_TX_SW_BUFFER (BLUE_INTERNAL_TX_SW_BUFFER)

    /* Provide global variables to support RX and TX buffers */
    #define BLUE_INTERNAL_RX_SW_BUFFER_CONST    (BLUE_INTERNAL_RX_SW_BUFFER)
    #define BLUE_INTERNAL_TX_SW_BUFFER_CONST    (BLUE_INTERNAL_TX_SW_BUFFER)
    
    /* Wakeup for SPI */
    #define BLUE_CHECK_SPI_WAKE_ENABLE  (BLUE_SPI_WAKE_ENABLE_CONST)

#endif /* End (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Bootloader communication interface enable: NOT supported yet */
#define BLUE_SPI_BTLDR_COMM_ENABLED   ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BLUE) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

#define BLUE_UART_BTLDR_COMM_ENABLED   ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BLUE) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))


/***************************************
*       Type Definitions
***************************************/

/* BLUE_SPI_INIT_STRUCT */
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
} BLUE_SPI_INIT_STRUCT;

/* BLUE_UART_INIT_STRUCT */
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
} BLUE_UART_INIT_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/* SPI specific functions */
#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
    void BLUE_SpiInit(const BLUE_SPI_INIT_STRUCT *config);
#endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(BLUE_SCB_MODE_SPI_INC)
    void BLUE_SpiSetActiveSlaveSelect(uint32 activeSelect);
#endif /* (BLUE_SCB_MODE_SPI_INC) */

/* UART specific functions */
#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
    void BLUE_UartInit(const BLUE_UART_INIT_STRUCT *config);
#endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(BLUE_SCB_MODE_UART_INC)
    void BLUE_UartSetRxAddress(uint32 address);
    void BLUE_UartSetRxAddressMask(uint32 addressMask);
#endif /* (BLUE_SCB_MODE_UART_INC) */

/* UART RX direction APIs */
#if(BLUE_UART_RX_DIRECTION)
    uint32 BLUE_UartGetChar(void);
    uint32 BLUE_UartGetByte(void);
#endif /* (BLUE_UART_RX_DIRECTION) */

/* UART TX direction APIs */
#if(BLUE_UART_TX_DIRECTION)
    #define BLUE_UartPutChar(ch)    BLUE_SpiUartWriteTxData((uint32)(ch))
    void BLUE_UartPutString(const char8 string[]);
    void BLUE_UartPutCRLF(uint32 txDataByte);
#endif /* (BLUE_UART_TX_DIRECTION) */

/* Common APIs Rx direction */
#if(BLUE_RX_DIRECTION)
    uint32 BLUE_SpiUartReadRxData(void);
    uint32 BLUE_SpiUartGetRxBufferSize(void);
    void   BLUE_SpiUartClearRxBuffer(void);
#endif /* (BLUE_RX_DIRECTION) */

/* Common APIs Tx direction */
#if(BLUE_TX_DIRECTION)
    void   BLUE_SpiUartWriteTxData(uint32 txDataByte);
    void   BLUE_SpiUartPutArray(const uint8 wrBuf[], uint32 count);
    void   BLUE_SpiUartClearTxBuffer(void);
    uint32 BLUE_SpiUartGetTxBufferSize(void);
#endif /* (BLUE_TX_DIRECTION) */

CY_ISR_PROTO(BLUE_SPI_UART_ISR);

#if(BLUE_UART_RX_WAKEUP_IRQ)
    CY_ISR_PROTO(BLUE_UART_WAKEUP_ISR);
#endif /* (BLUE_UART_RX_WAKEUP_IRQ) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (BLUE_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void BLUE_SpiCyBtldrCommStart(void);
    void BLUE_SpiCyBtldrCommStop (void);
    void BLUE_SpiCyBtldrCommReset(void);
    cystatus BLUE_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus BLUE_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (BLUE_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (BLUE_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void BLUE_UartCyBtldrCommStart(void);
    void BLUE_UartCyBtldrCommStop (void);
    void BLUE_UartCyBtldrCommReset(void);
    cystatus BLUE_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus BLUE_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (BLUE_UART_BTLDR_COMM_ENABLED) */


/***************************************
*     Buffer Access Macro Definitions
***************************************/

#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
    /* RX direction */
    void   BLUE_PutWordInRxBuffer  (uint32 idx, uint32 rxDataByte);
    uint32 BLUE_GetWordFromRxBuffer(uint32 idx);

    /* TX direction */
    void   BLUE_PutWordInTxBuffer  (uint32 idx, uint32 txDataByte);
    uint32 BLUE_GetWordFromTxBuffer(uint32 idx);

#else

    /* RX direction */
    #if(BLUE_INTERNAL_RX_SW_BUFFER_CONST)
        #define BLUE_PutWordInRxBuffer(idx, rxDataByte) \
                do{                                                 \
                    BLUE_rxBufferInternal[(idx)] = ((uint8) (rxDataByte)); \
                }while(0)

        #define BLUE_GetWordFromRxBuffer(idx) BLUE_rxBufferInternal[(idx)]

    #endif /* (BLUE_INTERNAL_RX_SW_BUFFER_CONST) */

    /* TX direction */
    #if(BLUE_INTERNAL_TX_SW_BUFFER_CONST)
        #define BLUE_PutWordInTxBuffer(idx, txDataByte) \
                    do{                                             \
                        BLUE_txBufferInternal[(idx)] = ((uint8) (txDataByte)); \
                    }while(0)

        #define BLUE_GetWordFromTxBuffer(idx) BLUE_txBufferInternal[(idx)]

    #endif /* (BLUE_INTERNAL_TX_SW_BUFFER_CONST) */

#endif /* (BLUE_TX_SW_BUFFER_ENABLE) */


/***************************************
*         SPI API Constants
***************************************/

/* SPI mode enum */
#define BLUE_SPI_SLAVE  (0u)
#define BLUE_SPI_MASTER (1u)

/* SPI sub mode enum */
#define BLUE_SPI_MODE_MOTOROLA      (0x00u)
#define BLUE_SPI_MODE_TI_COINCIDES  (0x01u)
#define BLUE_SPI_MODE_TI_PRECEDES   (0x11u)
#define BLUE_SPI_MODE_NATIONAL      (0x02u)
#define BLUE_SPI_MODE_MASK          (0x03u)
#define BLUE_SPI_MODE_TI_PRECEDES_MASK  (0x10u)

/* SPI phase and polarity mode enum */
#define BLUE_SPI_SCLK_CPHA0_CPOL0   (0x00u)
#define BLUE_SPI_SCLK_CPHA0_CPOL1   (0x02u)
#define BLUE_SPI_SCLK_CPHA1_CPOL0   (0x01u)
#define BLUE_SPI_SCLK_CPHA1_CPOL1   (0x03u)

/* SPI bits order enum */
#define BLUE_BITS_ORDER_LSB_FIRST   (0u)
#define BLUE_BITS_ORDER_MSB_FIRST   (1u)

/* SPI transfer separation enum */
#define BLUE_SPI_TRANSFER_SEPARATED     (0u)
#define BLUE_SPI_TRANSFER_CONTINUOUS    (1u)

/* "activeSS" constants for SpiSetActiveSlaveSelect() function */
#define BLUE_SPIM_ACTIVE_SS0    (0x00u)
#define BLUE_SPIM_ACTIVE_SS1    (0x01u)
#define BLUE_SPIM_ACTIVE_SS2    (0x02u)
#define BLUE_SPIM_ACTIVE_SS3    (0x03u)


/***************************************
*         UART API Constants
***************************************/

/* UART sub-modes enum */
#define BLUE_UART_MODE_STD          (0u)
#define BLUE_UART_MODE_SMARTCARD    (1u)
#define BLUE_UART_MODE_IRDA         (2u)

/* UART direction enum */
#define BLUE_UART_RX    (1u)
#define BLUE_UART_TX    (2u)
#define BLUE_UART_TX_RX (3u)

/* UART parity enum */
#define BLUE_UART_PARITY_EVEN   (0u)
#define BLUE_UART_PARITY_ODD    (1u)
#define BLUE_UART_PARITY_NONE   (2u)

/* UART stop bits enum */
#define BLUE_UART_STOP_BITS_1   (1u)
#define BLUE_UART_STOP_BITS_1_5 (2u)
#define BLUE_UART_STOP_BITS_2   (3u)

/* UART IrDA low power OVS enum */
#define BLUE_UART_IRDA_LP_OVS16     (16u)
#define BLUE_UART_IRDA_LP_OVS32     (32u)
#define BLUE_UART_IRDA_LP_OVS48     (48u)
#define BLUE_UART_IRDA_LP_OVS96     (96u)
#define BLUE_UART_IRDA_LP_OVS192    (192u)
#define BLUE_UART_IRDA_LP_OVS768    (768u)
#define BLUE_UART_IRDA_LP_OVS1536   (1536u)

/* Uart MP: mark (address) and space (data) bit definitions */
#define BLUE_UART_MP_MARK       (0x100u)
#define BLUE_UART_MP_SPACE      (0x000u)


/***************************************
*     Vars with External Linkage
***************************************/

#if(BLUE_SCB_MODE_UNCONFIG_CONST_CFG)
    extern const BLUE_SPI_INIT_STRUCT  BLUE_configSpi;
    extern const BLUE_UART_INIT_STRUCT BLUE_configUart;
#endif /* (BLUE_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*    Specific SPI Macro Definitions
***************************************/

#define BLUE_GET_SPI_INTR_SLAVE_MASK(sourceMask)  ((sourceMask) & BLUE_INTR_SLAVE_SPI_BUS_ERROR)
#define BLUE_GET_SPI_INTR_MASTER_MASK(sourceMask) ((sourceMask) & BLUE_INTR_MASTER_SPI_DONE)
#define BLUE_GET_SPI_INTR_RX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~BLUE_INTR_SLAVE_SPI_BUS_ERROR)

#define BLUE_GET_SPI_INTR_TX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~BLUE_INTR_MASTER_SPI_DONE)


/***************************************
*    Specific UART Macro Definitions
***************************************/

#define BLUE_UART_GET_CTRL_OVS_IRDA_LP(oversample) \
        ((BLUE_UART_IRDA_LP_OVS16   == (oversample)) ? BLUE_CTRL_OVS_IRDA_LP_OVS16 : \
         ((BLUE_UART_IRDA_LP_OVS32   == (oversample)) ? BLUE_CTRL_OVS_IRDA_LP_OVS32 : \
          ((BLUE_UART_IRDA_LP_OVS48   == (oversample)) ? BLUE_CTRL_OVS_IRDA_LP_OVS48 : \
           ((BLUE_UART_IRDA_LP_OVS96   == (oversample)) ? BLUE_CTRL_OVS_IRDA_LP_OVS96 : \
            ((BLUE_UART_IRDA_LP_OVS192  == (oversample)) ? BLUE_CTRL_OVS_IRDA_LP_OVS192 : \
             ((BLUE_UART_IRDA_LP_OVS768  == (oversample)) ? BLUE_CTRL_OVS_IRDA_LP_OVS768 : \
              ((BLUE_UART_IRDA_LP_OVS1536 == (oversample)) ? BLUE_CTRL_OVS_IRDA_LP_OVS1536 : \
                                                                          BLUE_CTRL_OVS_IRDA_LP_OVS16)))))))

#define BLUE_GET_UART_RX_CTRL_ENABLED(direction) ((0u != (BLUE_UART_RX & (direction))) ? \
                                                                    (BLUE_RX_CTRL_ENABLED) : (0u))

#define BLUE_GET_UART_TX_CTRL_ENABLED(direction) ((0u != (BLUE_UART_TX & (direction))) ? \
                                                                    (BLUE_TX_CTRL_ENABLED) : (0u))


/***************************************
*        SPI Register Settings
***************************************/

#define BLUE_CTRL_SPI      (BLUE_CTRL_MODE_SPI)
#define BLUE_SPI_RX_CTRL   (BLUE_RX_CTRL_ENABLED)
#define BLUE_SPI_TX_CTRL   (BLUE_TX_CTRL_ENABLED)


/***************************************
*       SPI Init Register Settings
***************************************/

#if(BLUE_SCB_MODE_SPI_CONST_CFG)

    /* SPI Configuration */
    #define BLUE_SPI_DEFAULT_CTRL \
                    (BLUE_GET_CTRL_OVS(BLUE_SPI_OVS_FACTOR)         | \
                     BLUE_GET_CTRL_EC_AM_MODE(BLUE_SPI_WAKE_ENABLE) | \
                     BLUE_CTRL_SPI)

    #define BLUE_SPI_DEFAULT_SPI_CTRL \
                    (BLUE_GET_SPI_CTRL_CONTINUOUS    (BLUE_SPI_TRANSFER_SEPARATION)       | \
                     BLUE_GET_SPI_CTRL_SELECT_PRECEDE(BLUE_SPI_SUB_MODE &                   \
                                                                  BLUE_SPI_MODE_TI_PRECEDES_MASK)     | \
                     BLUE_GET_SPI_CTRL_SCLK_MODE     (BLUE_SPI_CLOCK_MODE)                | \
                     BLUE_GET_SPI_CTRL_LATE_MISO_SAMPLE(BLUE_SPI_LATE_MISO_SAMPLE_ENABLE) | \
                     BLUE_GET_SPI_CTRL_SUB_MODE      (BLUE_SPI_SUB_MODE)                  | \
                     BLUE_GET_SPI_CTRL_MASTER_MODE   (BLUE_SPI_MODE))

    /* RX direction */
    #define BLUE_SPI_DEFAULT_RX_CTRL \
                    (BLUE_GET_RX_CTRL_DATA_WIDTH(BLUE_SPI_RX_DATA_BITS_NUM)     | \
                     BLUE_GET_RX_CTRL_BIT_ORDER (BLUE_SPI_BITS_ORDER)           | \
                     BLUE_GET_RX_CTRL_MEDIAN    (BLUE_SPI_MEDIAN_FILTER_ENABLE) | \
                     BLUE_SPI_RX_CTRL)

    #define BLUE_SPI_DEFAULT_RX_FIFO_CTRL \
                    BLUE_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(BLUE_SPI_RX_TRIGGER_LEVEL)

    /* TX direction */
    #define BLUE_SPI_DEFAULT_TX_CTRL \
                    (BLUE_GET_TX_CTRL_DATA_WIDTH(BLUE_SPI_TX_DATA_BITS_NUM) | \
                     BLUE_GET_TX_CTRL_BIT_ORDER (BLUE_SPI_BITS_ORDER)       | \
                     BLUE_SPI_TX_CTRL)

    #define BLUE_SPI_DEFAULT_TX_FIFO_CTRL \
                    BLUE_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(BLUE_SPI_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define BLUE_SPI_DEFAULT_INTR_SPI_EC_MASK   (BLUE_NO_INTR_SOURCES)

    #define BLUE_SPI_DEFAULT_INTR_I2C_EC_MASK   (BLUE_NO_INTR_SOURCES)
    #define BLUE_SPI_DEFAULT_INTR_SLAVE_MASK \
                    (BLUE_SPI_INTR_RX_MASK & BLUE_INTR_SLAVE_SPI_BUS_ERROR)

    #define BLUE_SPI_DEFAULT_INTR_MASTER_MASK \
                    (BLUE_SPI_INTR_TX_MASK & BLUE_INTR_MASTER_SPI_DONE)

    #define BLUE_SPI_DEFAULT_INTR_RX_MASK \
                    (BLUE_SPI_INTR_RX_MASK & (uint32) ~BLUE_INTR_SLAVE_SPI_BUS_ERROR)

    #define BLUE_SPI_DEFAULT_INTR_TX_MASK \
                    (BLUE_SPI_INTR_TX_MASK & (uint32) ~BLUE_INTR_MASTER_SPI_DONE)

#endif /* (BLUE_SCB_MODE_SPI_CONST_CFG) */


/***************************************
*        UART Register Settings
***************************************/

#define BLUE_CTRL_UART      (BLUE_CTRL_MODE_UART)
#define BLUE_UART_RX_CTRL   (BLUE_RX_CTRL_LSB_FIRST) /* LSB for UART goes first */
#define BLUE_UART_TX_CTRL   (BLUE_TX_CTRL_LSB_FIRST) /* LSB for UART goes first */


/***************************************
*      UART Init Register Settings
***************************************/

#if(BLUE_SCB_MODE_UART_CONST_CFG)

    /* UART configuration */
    #if(BLUE_UART_MODE_IRDA == BLUE_UART_SUB_MODE)

        #define BLUE_DEFAULT_CTRL_OVS   ((0u != BLUE_UART_IRDA_LOW_POWER) ?              \
                                (BLUE_UART_GET_CTRL_OVS_IRDA_LP(BLUE_UART_OVS_FACTOR)) : \
                                (BLUE_CTRL_OVS_IRDA_OVS16))

    #else

        #define BLUE_DEFAULT_CTRL_OVS   BLUE_GET_CTRL_OVS(BLUE_UART_OVS_FACTOR)

    #endif /* (BLUE_UART_MODE_IRDA == BLUE_UART_SUB_MODE) */

    #define BLUE_UART_DEFAULT_CTRL \
                                (BLUE_GET_CTRL_ADDR_ACCEPT(BLUE_UART_MP_ACCEPT_ADDRESS) | \
                                 BLUE_DEFAULT_CTRL_OVS                                              | \
                                 BLUE_CTRL_UART)

    #define BLUE_UART_DEFAULT_UART_CTRL \
                                    (BLUE_GET_UART_CTRL_MODE(BLUE_UART_SUB_MODE))

    /* RX direction */
    #define BLUE_UART_DEFAULT_RX_CTRL_PARITY \
                                ((BLUE_UART_PARITY_NONE != BLUE_UART_PARITY_TYPE) ?      \
                                  (BLUE_GET_UART_RX_CTRL_PARITY(BLUE_UART_PARITY_TYPE) | \
                                   BLUE_UART_RX_CTRL_PARITY_ENABLED) : (0u))

    #define BLUE_UART_DEFAULT_UART_RX_CTRL \
                    (BLUE_GET_UART_RX_CTRL_MODE(BLUE_UART_STOP_BITS_NUM)                    | \
                     BLUE_GET_UART_RX_CTRL_POLARITY(BLUE_UART_IRDA_POLARITY)                | \
                     BLUE_GET_UART_RX_CTRL_MP_MODE(BLUE_UART_MP_MODE_ENABLE)                | \
                     BLUE_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(BLUE_UART_DROP_ON_PARITY_ERR) | \
                     BLUE_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(BLUE_UART_DROP_ON_FRAME_ERR)   | \
                     BLUE_UART_DEFAULT_RX_CTRL_PARITY)

    #define BLUE_UART_DEFAULT_RX_CTRL \
                                (BLUE_GET_RX_CTRL_DATA_WIDTH(BLUE_UART_DATA_BITS_NUM)        | \
                                 BLUE_GET_RX_CTRL_MEDIAN    (BLUE_UART_MEDIAN_FILTER_ENABLE) | \
                                 BLUE_GET_UART_RX_CTRL_ENABLED(BLUE_UART_DIRECTION))

    #define BLUE_UART_DEFAULT_RX_FIFO_CTRL \
                                BLUE_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(BLUE_UART_RX_TRIGGER_LEVEL)

    #define BLUE_UART_DEFAULT_RX_MATCH_REG  ((0u != BLUE_UART_MP_MODE_ENABLE) ?          \
                                (BLUE_GET_RX_MATCH_ADDR(BLUE_UART_MP_RX_ADDRESS) | \
                                 BLUE_GET_RX_MATCH_MASK(BLUE_UART_MP_RX_ADDRESS_MASK)) : (0u))

    /* TX direction */
    #define BLUE_UART_DEFAULT_TX_CTRL_PARITY (BLUE_UART_DEFAULT_RX_CTRL_PARITY)

    #define BLUE_UART_DEFAULT_UART_TX_CTRL \
                                (BLUE_GET_UART_TX_CTRL_MODE(BLUE_UART_STOP_BITS_NUM)       | \
                                 BLUE_GET_UART_TX_CTRL_RETRY_NACK(BLUE_UART_RETRY_ON_NACK) | \
                                 BLUE_UART_DEFAULT_TX_CTRL_PARITY)

    #define BLUE_UART_DEFAULT_TX_CTRL \
                                (BLUE_GET_TX_CTRL_DATA_WIDTH(BLUE_UART_DATA_BITS_NUM) | \
                                 BLUE_GET_UART_TX_CTRL_ENABLED(BLUE_UART_DIRECTION))

    #define BLUE_UART_DEFAULT_TX_FIFO_CTRL \
                                BLUE_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(BLUE_UART_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define BLUE_UART_DEFAULT_INTR_I2C_EC_MASK  (BLUE_NO_INTR_SOURCES)
    #define BLUE_UART_DEFAULT_INTR_SPI_EC_MASK  (BLUE_NO_INTR_SOURCES)
    #define BLUE_UART_DEFAULT_INTR_SLAVE_MASK   (BLUE_NO_INTR_SOURCES)
    #define BLUE_UART_DEFAULT_INTR_MASTER_MASK  (BLUE_NO_INTR_SOURCES)
    #define BLUE_UART_DEFAULT_INTR_RX_MASK      (BLUE_UART_INTR_RX_MASK)
    #define BLUE_UART_DEFAULT_INTR_TX_MASK      (BLUE_UART_INTR_TX_MASK)

#endif /* (BLUE_SCB_MODE_UART_CONST_CFG) */

#endif /* CY_SCB_SPI_UART_BLUE_H */


/* [] END OF FILE */
