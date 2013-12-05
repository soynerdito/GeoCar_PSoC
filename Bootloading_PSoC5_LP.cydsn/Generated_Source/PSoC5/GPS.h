/*******************************************************************************
* File Name: GPS.h
* Version 2.30
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_GPS_H)
#define CY_UART_GPS_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define GPS_RX_ENABLED                     (1u)
#define GPS_TX_ENABLED                     (0u)
#define GPS_HD_ENABLED                     (0u)
#define GPS_RX_INTERRUPT_ENABLED           (1u)
#define GPS_TX_INTERRUPT_ENABLED           (0u)
#define GPS_INTERNAL_CLOCK_USED            (1u)
#define GPS_RXHW_ADDRESS_ENABLED           (0u)
#define GPS_OVER_SAMPLE_COUNT              (8u)
#define GPS_PARITY_TYPE                    (0u)
#define GPS_PARITY_TYPE_SW                 (0u)
#define GPS_BREAK_DETECT                   (0u)
#define GPS_BREAK_BITS_TX                  (13u)
#define GPS_BREAK_BITS_RX                  (13u)
#define GPS_TXCLKGEN_DP                    (1u)
#define GPS_USE23POLLING                   (1u)
#define GPS_FLOW_CONTROL                   (0u)
#define GPS_CLK_FREQ                       (0u)
#define GPS_TXBUFFERSIZE                   (4u)
#define GPS_RXBUFFERSIZE                   (80u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define GPS_CONTROL_REG_REMOVED            (0u)
#else
    #define GPS_CONTROL_REG_REMOVED            (1u)
#endif /* End GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct GPS_backupStruct_
{
    uint8 enableState;

    #if(GPS_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End GPS_CONTROL_REG_REMOVED */
    #if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (GPS_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End GPS_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (GPS_RX_ENABLED) || (GPS_HD_ENABLED)*/

    #if(GPS_TX_ENABLED)
        #if(GPS_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End GPS_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End GPS_TX_ENABLED */
} GPS_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void GPS_Start(void) ;
void GPS_Stop(void) ;
uint8 GPS_ReadControlRegister(void) ;
void GPS_WriteControlRegister(uint8 control) ;

void GPS_Init(void) ;
void GPS_Enable(void) ;
void GPS_SaveConfig(void) ;
void GPS_RestoreConfig(void) ;
void GPS_Sleep(void) ;
void GPS_Wakeup(void) ;

/* Only if RX is enabled */
#if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )

    #if(GPS_RX_INTERRUPT_ENABLED)
        void  GPS_EnableRxInt(void) ;
        void  GPS_DisableRxInt(void) ;
        CY_ISR_PROTO(GPS_RXISR);
    #endif /* GPS_RX_INTERRUPT_ENABLED */

    void GPS_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void GPS_SetRxAddress1(uint8 address) ;
    void GPS_SetRxAddress2(uint8 address) ;

    void  GPS_SetRxInterruptMode(uint8 intSrc) ;
    uint8 GPS_ReadRxData(void) ;
    uint8 GPS_ReadRxStatus(void) ;
    uint8 GPS_GetChar(void) ;
    uint16 GPS_GetByte(void) ;
    uint8 GPS_GetRxBufferSize(void)
                                                            ;
    void GPS_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define GPS_GetRxInterruptSource   GPS_ReadRxStatus

#endif /* End (GPS_RX_ENABLED) || (GPS_HD_ENABLED) */

/* Only if TX is enabled */
#if(GPS_TX_ENABLED || GPS_HD_ENABLED)

    #if(GPS_TX_INTERRUPT_ENABLED)
        void GPS_EnableTxInt(void) ;
        void GPS_DisableTxInt(void) ;
        CY_ISR_PROTO(GPS_TXISR);
    #endif /* GPS_TX_INTERRUPT_ENABLED */

    void GPS_SetTxInterruptMode(uint8 intSrc) ;
    void GPS_WriteTxData(uint8 txDataByte) ;
    uint8 GPS_ReadTxStatus(void) ;
    void GPS_PutChar(uint8 txDataByte) ;
    void GPS_PutString(const char8 string[]) ;
    void GPS_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void GPS_PutCRLF(uint8 txDataByte) ;
    void GPS_ClearTxBuffer(void) ;
    void GPS_SetTxAddressMode(uint8 addressMode) ;
    void GPS_SendBreak(uint8 retMode) ;
    uint8 GPS_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define GPS_PutStringConst         GPS_PutString
    #define GPS_PutArrayConst          GPS_PutArray
    #define GPS_GetTxInterruptSource   GPS_ReadTxStatus

#endif /* End GPS_TX_ENABLED || GPS_HD_ENABLED */

#if(GPS_HD_ENABLED)
    void GPS_LoadRxConfig(void) ;
    void GPS_LoadTxConfig(void) ;
#endif /* End GPS_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_GPS) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    GPS_CyBtldrCommStart(void) CYSMALL ;
    void    GPS_CyBtldrCommStop(void) CYSMALL ;
    void    GPS_CyBtldrCommReset(void) CYSMALL ;
    cystatus GPS_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus GPS_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_GPS)
        #define CyBtldrCommStart    GPS_CyBtldrCommStart
        #define CyBtldrCommStop     GPS_CyBtldrCommStop
        #define CyBtldrCommReset    GPS_CyBtldrCommReset
        #define CyBtldrCommWrite    GPS_CyBtldrCommWrite
        #define CyBtldrCommRead     GPS_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_GPS) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define GPS_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define GPS_SET_SPACE                              (0x00u)
#define GPS_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (GPS_TX_ENABLED) || (GPS_HD_ENABLED) )
    #if(GPS_TX_INTERRUPT_ENABLED)
        #define GPS_TX_VECT_NUM            (uint8)GPS_TXInternalInterrupt__INTC_NUMBER
        #define GPS_TX_PRIOR_NUM           (uint8)GPS_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* GPS_TX_INTERRUPT_ENABLED */
    #if(GPS_TX_ENABLED)
        #define GPS_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define GPS_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define GPS_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define GPS_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* GPS_TX_ENABLED */
    #if(GPS_HD_ENABLED)
        #define GPS_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define GPS_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define GPS_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define GPS_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* GPS_HD_ENABLED */
    #define GPS_TX_STS_COMPLETE            (uint8)(0x01u << GPS_TX_STS_COMPLETE_SHIFT)
    #define GPS_TX_STS_FIFO_EMPTY          (uint8)(0x01u << GPS_TX_STS_FIFO_EMPTY_SHIFT)
    #define GPS_TX_STS_FIFO_FULL           (uint8)(0x01u << GPS_TX_STS_FIFO_FULL_SHIFT)
    #define GPS_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << GPS_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (GPS_TX_ENABLED) || (GPS_HD_ENABLED)*/

#if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
    #if(GPS_RX_INTERRUPT_ENABLED)
        #define GPS_RX_VECT_NUM            (uint8)GPS_RXInternalInterrupt__INTC_NUMBER
        #define GPS_RX_PRIOR_NUM           (uint8)GPS_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* GPS_RX_INTERRUPT_ENABLED */
    #define GPS_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define GPS_RX_STS_BREAK_SHIFT             (0x01u)
    #define GPS_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define GPS_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define GPS_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define GPS_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define GPS_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define GPS_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define GPS_RX_STS_MRKSPC           (uint8)(0x01u << GPS_RX_STS_MRKSPC_SHIFT)
    #define GPS_RX_STS_BREAK            (uint8)(0x01u << GPS_RX_STS_BREAK_SHIFT)
    #define GPS_RX_STS_PAR_ERROR        (uint8)(0x01u << GPS_RX_STS_PAR_ERROR_SHIFT)
    #define GPS_RX_STS_STOP_ERROR       (uint8)(0x01u << GPS_RX_STS_STOP_ERROR_SHIFT)
    #define GPS_RX_STS_OVERRUN          (uint8)(0x01u << GPS_RX_STS_OVERRUN_SHIFT)
    #define GPS_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << GPS_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define GPS_RX_STS_ADDR_MATCH       (uint8)(0x01u << GPS_RX_STS_ADDR_MATCH_SHIFT)
    #define GPS_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << GPS_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define GPS_RX_HW_MASK                     (0x7Fu)
#endif /* End (GPS_RX_ENABLED) || (GPS_HD_ENABLED) */

/* Control Register definitions */
#define GPS_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define GPS_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define GPS_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define GPS_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define GPS_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define GPS_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define GPS_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define GPS_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define GPS_CTRL_HD_SEND               (uint8)(0x01u << GPS_CTRL_HD_SEND_SHIFT)
#define GPS_CTRL_HD_SEND_BREAK         (uint8)(0x01u << GPS_CTRL_HD_SEND_BREAK_SHIFT)
#define GPS_CTRL_MARK                  (uint8)(0x01u << GPS_CTRL_MARK_SHIFT)
#define GPS_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << GPS_CTRL_PARITY_TYPE0_SHIFT)
#define GPS_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << GPS_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define GPS_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define GPS_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define GPS_SEND_BREAK                         (0x00u)
#define GPS_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define GPS_REINIT                             (0x02u)
#define GPS_SEND_WAIT_REINIT                   (0x03u)

#define GPS_OVER_SAMPLE_8                      (8u)
#define GPS_OVER_SAMPLE_16                     (16u)

#define GPS_BIT_CENTER                         (GPS_OVER_SAMPLE_COUNT - 1u)

#define GPS_FIFO_LENGTH                        (4u)
#define GPS_NUMBER_OF_START_BIT                (1u)
#define GPS_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define GPS_TXBITCTR_BREAKBITS8X   ((GPS_BREAK_BITS_TX * GPS_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define GPS_TXBITCTR_BREAKBITS ((GPS_BREAK_BITS_TX * GPS_OVER_SAMPLE_COUNT) - 1u)

#define GPS_HALF_BIT_COUNT   \
                            (((GPS_OVER_SAMPLE_COUNT / 2u) + (GPS_USE23POLLING * 1u)) - 2u)
#if (GPS_OVER_SAMPLE_COUNT == GPS_OVER_SAMPLE_8)
    #define GPS_HD_TXBITCTR_INIT   (((GPS_BREAK_BITS_TX + \
                            GPS_NUMBER_OF_START_BIT) * GPS_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define GPS_RXBITCTR_INIT  ((((GPS_BREAK_BITS_RX + GPS_NUMBER_OF_START_BIT) \
                            * GPS_OVER_SAMPLE_COUNT) + GPS_HALF_BIT_COUNT) - 1u)


#else /* GPS_OVER_SAMPLE_COUNT == GPS_OVER_SAMPLE_16 */
    #define GPS_HD_TXBITCTR_INIT   ((8u * GPS_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define GPS_RXBITCTR_INIT      (((7u * GPS_OVER_SAMPLE_COUNT) - 1u) + \
                                                      GPS_HALF_BIT_COUNT)
#endif /* End GPS_OVER_SAMPLE_COUNT */
#define GPS_HD_RXBITCTR_INIT                   GPS_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 GPS_initVar;
#if( GPS_TX_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))
    extern volatile uint8 GPS_txBuffer[GPS_TXBUFFERSIZE];
    extern volatile uint8 GPS_txBufferRead;
    extern uint8 GPS_txBufferWrite;
#endif /* End GPS_TX_ENABLED */
#if( ( GPS_RX_ENABLED || GPS_HD_ENABLED ) && \
     (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH) )
    extern volatile uint8 GPS_rxBuffer[GPS_RXBUFFERSIZE];
    extern volatile uint8 GPS_rxBufferRead;
    extern volatile uint8 GPS_rxBufferWrite;
    extern volatile uint8 GPS_rxBufferLoopDetect;
    extern volatile uint8 GPS_rxBufferOverflow;
    #if (GPS_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 GPS_rxAddressMode;
        extern volatile uint8 GPS_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End GPS_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define GPS__B_UART__AM_SW_BYTE_BYTE 1
#define GPS__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define GPS__B_UART__AM_HW_BYTE_BY_BYTE 3
#define GPS__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define GPS__B_UART__AM_NONE 0

#define GPS__B_UART__NONE_REVB 0
#define GPS__B_UART__EVEN_REVB 1
#define GPS__B_UART__ODD_REVB 2
#define GPS__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define GPS_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define GPS_NUMBER_OF_STOP_BITS    (1u)

#if (GPS_RXHW_ADDRESS_ENABLED)
    #define GPS_RXADDRESSMODE      (0u)
    #define GPS_RXHWADDRESS1       (0u)
    #define GPS_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define GPS_RXAddressMode      GPS_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define GPS_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << GPS_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << GPS_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << GPS_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << GPS_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << GPS_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << GPS_RX_STS_BREAK_SHIFT) \
                                        | (0 << GPS_RX_STS_OVERRUN_SHIFT))

#define GPS_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << GPS_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << GPS_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << GPS_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << GPS_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define GPS_CONTROL_REG \
                            (* (reg8 *) GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define GPS_CONTROL_PTR \
                            (  (reg8 *) GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(GPS_TX_ENABLED)
    #define GPS_TXDATA_REG          (* (reg8 *) GPS_BUART_sTX_TxShifter_u0__F0_REG)
    #define GPS_TXDATA_PTR          (  (reg8 *) GPS_BUART_sTX_TxShifter_u0__F0_REG)
    #define GPS_TXDATA_AUX_CTL_REG  (* (reg8 *) GPS_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define GPS_TXDATA_AUX_CTL_PTR  (  (reg8 *) GPS_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define GPS_TXSTATUS_REG        (* (reg8 *) GPS_BUART_sTX_TxSts__STATUS_REG)
    #define GPS_TXSTATUS_PTR        (  (reg8 *) GPS_BUART_sTX_TxSts__STATUS_REG)
    #define GPS_TXSTATUS_MASK_REG   (* (reg8 *) GPS_BUART_sTX_TxSts__MASK_REG)
    #define GPS_TXSTATUS_MASK_PTR   (  (reg8 *) GPS_BUART_sTX_TxSts__MASK_REG)
    #define GPS_TXSTATUS_ACTL_REG   (* (reg8 *) GPS_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define GPS_TXSTATUS_ACTL_PTR   (  (reg8 *) GPS_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(GPS_TXCLKGEN_DP)
        #define GPS_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define GPS_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define GPS_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define GPS_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define GPS_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define GPS_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define GPS_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define GPS_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define GPS_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define GPS_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) GPS_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* GPS_TXCLKGEN_DP */

#endif /* End GPS_TX_ENABLED */

#if(GPS_HD_ENABLED)

    #define GPS_TXDATA_REG             (* (reg8 *) GPS_BUART_sRX_RxShifter_u0__F1_REG )
    #define GPS_TXDATA_PTR             (  (reg8 *) GPS_BUART_sRX_RxShifter_u0__F1_REG )
    #define GPS_TXDATA_AUX_CTL_REG     (* (reg8 *) GPS_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define GPS_TXDATA_AUX_CTL_PTR     (  (reg8 *) GPS_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define GPS_TXSTATUS_REG           (* (reg8 *) GPS_BUART_sRX_RxSts__STATUS_REG )
    #define GPS_TXSTATUS_PTR           (  (reg8 *) GPS_BUART_sRX_RxSts__STATUS_REG )
    #define GPS_TXSTATUS_MASK_REG      (* (reg8 *) GPS_BUART_sRX_RxSts__MASK_REG )
    #define GPS_TXSTATUS_MASK_PTR      (  (reg8 *) GPS_BUART_sRX_RxSts__MASK_REG )
    #define GPS_TXSTATUS_ACTL_REG      (* (reg8 *) GPS_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define GPS_TXSTATUS_ACTL_PTR      (  (reg8 *) GPS_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End GPS_HD_ENABLED */

#if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
    #define GPS_RXDATA_REG             (* (reg8 *) GPS_BUART_sRX_RxShifter_u0__F0_REG )
    #define GPS_RXDATA_PTR             (  (reg8 *) GPS_BUART_sRX_RxShifter_u0__F0_REG )
    #define GPS_RXADDRESS1_REG         (* (reg8 *) GPS_BUART_sRX_RxShifter_u0__D0_REG )
    #define GPS_RXADDRESS1_PTR         (  (reg8 *) GPS_BUART_sRX_RxShifter_u0__D0_REG )
    #define GPS_RXADDRESS2_REG         (* (reg8 *) GPS_BUART_sRX_RxShifter_u0__D1_REG )
    #define GPS_RXADDRESS2_PTR         (  (reg8 *) GPS_BUART_sRX_RxShifter_u0__D1_REG )
    #define GPS_RXDATA_AUX_CTL_REG     (* (reg8 *) GPS_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define GPS_RXBITCTR_PERIOD_REG    (* (reg8 *) GPS_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define GPS_RXBITCTR_PERIOD_PTR    (  (reg8 *) GPS_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define GPS_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) GPS_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define GPS_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) GPS_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define GPS_RXBITCTR_COUNTER_REG   (* (reg8 *) GPS_BUART_sRX_RxBitCounter__COUNT_REG )
    #define GPS_RXBITCTR_COUNTER_PTR   (  (reg8 *) GPS_BUART_sRX_RxBitCounter__COUNT_REG )

    #define GPS_RXSTATUS_REG           (* (reg8 *) GPS_BUART_sRX_RxSts__STATUS_REG )
    #define GPS_RXSTATUS_PTR           (  (reg8 *) GPS_BUART_sRX_RxSts__STATUS_REG )
    #define GPS_RXSTATUS_MASK_REG      (* (reg8 *) GPS_BUART_sRX_RxSts__MASK_REG )
    #define GPS_RXSTATUS_MASK_PTR      (  (reg8 *) GPS_BUART_sRX_RxSts__MASK_REG )
    #define GPS_RXSTATUS_ACTL_REG      (* (reg8 *) GPS_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define GPS_RXSTATUS_ACTL_PTR      (  (reg8 *) GPS_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (GPS_RX_ENABLED) || (GPS_HD_ENABLED) */

#if(GPS_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define GPS_INTCLOCK_CLKEN_REG     (* (reg8 *) GPS_IntClock__PM_ACT_CFG)
    #define GPS_INTCLOCK_CLKEN_PTR     (  (reg8 *) GPS_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define GPS_INTCLOCK_CLKEN_MASK    GPS_IntClock__PM_ACT_MSK
#endif /* End GPS_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(GPS_TX_ENABLED)
    #define GPS_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End GPS_TX_ENABLED */

#if(GPS_HD_ENABLED)
    #define GPS_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End GPS_HD_ENABLED */

#if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
    #define GPS_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (GPS_RX_ENABLED) || (GPS_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define GPS_initvar                    GPS_initVar

#define GPS_RX_Enabled                 GPS_RX_ENABLED
#define GPS_TX_Enabled                 GPS_TX_ENABLED
#define GPS_HD_Enabled                 GPS_HD_ENABLED
#define GPS_RX_IntInterruptEnabled     GPS_RX_INTERRUPT_ENABLED
#define GPS_TX_IntInterruptEnabled     GPS_TX_INTERRUPT_ENABLED
#define GPS_InternalClockUsed          GPS_INTERNAL_CLOCK_USED
#define GPS_RXHW_Address_Enabled       GPS_RXHW_ADDRESS_ENABLED
#define GPS_OverSampleCount            GPS_OVER_SAMPLE_COUNT
#define GPS_ParityType                 GPS_PARITY_TYPE

#if( GPS_TX_ENABLED && (GPS_TXBUFFERSIZE > GPS_FIFO_LENGTH))
    #define GPS_TXBUFFER               GPS_txBuffer
    #define GPS_TXBUFFERREAD           GPS_txBufferRead
    #define GPS_TXBUFFERWRITE          GPS_txBufferWrite
#endif /* End GPS_TX_ENABLED */
#if( ( GPS_RX_ENABLED || GPS_HD_ENABLED ) && \
     (GPS_RXBUFFERSIZE > GPS_FIFO_LENGTH) )
    #define GPS_RXBUFFER               GPS_rxBuffer
    #define GPS_RXBUFFERREAD           GPS_rxBufferRead
    #define GPS_RXBUFFERWRITE          GPS_rxBufferWrite
    #define GPS_RXBUFFERLOOPDETECT     GPS_rxBufferLoopDetect
    #define GPS_RXBUFFER_OVERFLOW      GPS_rxBufferOverflow
#endif /* End GPS_RX_ENABLED */

#ifdef GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define GPS_CONTROL                GPS_CONTROL_REG
#endif /* End GPS_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(GPS_TX_ENABLED)
    #define GPS_TXDATA                 GPS_TXDATA_REG
    #define GPS_TXSTATUS               GPS_TXSTATUS_REG
    #define GPS_TXSTATUS_MASK          GPS_TXSTATUS_MASK_REG
    #define GPS_TXSTATUS_ACTL          GPS_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(GPS_TXCLKGEN_DP)
        #define GPS_TXBITCLKGEN_CTR        GPS_TXBITCLKGEN_CTR_REG
        #define GPS_TXBITCLKTX_COMPLETE    GPS_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define GPS_TXBITCTR_PERIOD        GPS_TXBITCTR_PERIOD_REG
        #define GPS_TXBITCTR_CONTROL       GPS_TXBITCTR_CONTROL_REG
        #define GPS_TXBITCTR_COUNTER       GPS_TXBITCTR_COUNTER_REG
    #endif /* GPS_TXCLKGEN_DP */
#endif /* End GPS_TX_ENABLED */

#if(GPS_HD_ENABLED)
    #define GPS_TXDATA                 GPS_TXDATA_REG
    #define GPS_TXSTATUS               GPS_TXSTATUS_REG
    #define GPS_TXSTATUS_MASK          GPS_TXSTATUS_MASK_REG
    #define GPS_TXSTATUS_ACTL          GPS_TXSTATUS_ACTL_REG
#endif /* End GPS_HD_ENABLED */

#if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
    #define GPS_RXDATA                 GPS_RXDATA_REG
    #define GPS_RXADDRESS1             GPS_RXADDRESS1_REG
    #define GPS_RXADDRESS2             GPS_RXADDRESS2_REG
    #define GPS_RXBITCTR_PERIOD        GPS_RXBITCTR_PERIOD_REG
    #define GPS_RXBITCTR_CONTROL       GPS_RXBITCTR_CONTROL_REG
    #define GPS_RXBITCTR_COUNTER       GPS_RXBITCTR_COUNTER_REG
    #define GPS_RXSTATUS               GPS_RXSTATUS_REG
    #define GPS_RXSTATUS_MASK          GPS_RXSTATUS_MASK_REG
    #define GPS_RXSTATUS_ACTL          GPS_RXSTATUS_ACTL_REG
#endif /* End  (GPS_RX_ENABLED) || (GPS_HD_ENABLED) */

#if(GPS_INTERNAL_CLOCK_USED)
    #define GPS_INTCLOCK_CLKEN         GPS_INTCLOCK_CLKEN_REG
#endif /* End GPS_INTERNAL_CLOCK_USED */

#define GPS_WAIT_FOR_COMLETE_REINIT    GPS_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_GPS_H */


/* [] END OF FILE */
