/*******************************************************************************
* File Name: BLUE.h
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


#if !defined(CY_UART_BLUE_H)
#define CY_UART_BLUE_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define BLUE_RX_ENABLED                     (1u)
#define BLUE_TX_ENABLED                     (1u)
#define BLUE_HD_ENABLED                     (0u)
#define BLUE_RX_INTERRUPT_ENABLED           (0u)
#define BLUE_TX_INTERRUPT_ENABLED           (1u)
#define BLUE_INTERNAL_CLOCK_USED            (1u)
#define BLUE_RXHW_ADDRESS_ENABLED           (0u)
#define BLUE_OVER_SAMPLE_COUNT              (8u)
#define BLUE_PARITY_TYPE                    (0u)
#define BLUE_PARITY_TYPE_SW                 (0u)
#define BLUE_BREAK_DETECT                   (0u)
#define BLUE_BREAK_BITS_TX                  (13u)
#define BLUE_BREAK_BITS_RX                  (13u)
#define BLUE_TXCLKGEN_DP                    (1u)
#define BLUE_USE23POLLING                   (1u)
#define BLUE_FLOW_CONTROL                   (0u)
#define BLUE_CLK_FREQ                       (0u)
#define BLUE_TXBUFFERSIZE                   (50u)
#define BLUE_RXBUFFERSIZE                   (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define BLUE_CONTROL_REG_REMOVED            (0u)
#else
    #define BLUE_CONTROL_REG_REMOVED            (1u)
#endif /* End BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct BLUE_backupStruct_
{
    uint8 enableState;

    #if(BLUE_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End BLUE_CONTROL_REG_REMOVED */
    #if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (BLUE_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End BLUE_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED)*/

    #if(BLUE_TX_ENABLED)
        #if(BLUE_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End BLUE_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End BLUE_TX_ENABLED */
} BLUE_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void BLUE_Start(void) ;
void BLUE_Stop(void) ;
uint8 BLUE_ReadControlRegister(void) ;
void BLUE_WriteControlRegister(uint8 control) ;

void BLUE_Init(void) ;
void BLUE_Enable(void) ;
void BLUE_SaveConfig(void) ;
void BLUE_RestoreConfig(void) ;
void BLUE_Sleep(void) ;
void BLUE_Wakeup(void) ;

/* Only if RX is enabled */
#if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )

    #if(BLUE_RX_INTERRUPT_ENABLED)
        void  BLUE_EnableRxInt(void) ;
        void  BLUE_DisableRxInt(void) ;
        CY_ISR_PROTO(BLUE_RXISR);
    #endif /* BLUE_RX_INTERRUPT_ENABLED */

    void BLUE_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void BLUE_SetRxAddress1(uint8 address) ;
    void BLUE_SetRxAddress2(uint8 address) ;

    void  BLUE_SetRxInterruptMode(uint8 intSrc) ;
    uint8 BLUE_ReadRxData(void) ;
    uint8 BLUE_ReadRxStatus(void) ;
    uint8 BLUE_GetChar(void) ;
    uint16 BLUE_GetByte(void) ;
    uint8 BLUE_GetRxBufferSize(void)
                                                            ;
    void BLUE_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define BLUE_GetRxInterruptSource   BLUE_ReadRxStatus

#endif /* End (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) */

/* Only if TX is enabled */
#if(BLUE_TX_ENABLED || BLUE_HD_ENABLED)

    #if(BLUE_TX_INTERRUPT_ENABLED)
        void BLUE_EnableTxInt(void) ;
        void BLUE_DisableTxInt(void) ;
        CY_ISR_PROTO(BLUE_TXISR);
    #endif /* BLUE_TX_INTERRUPT_ENABLED */

    void BLUE_SetTxInterruptMode(uint8 intSrc) ;
    void BLUE_WriteTxData(uint8 txDataByte) ;
    uint8 BLUE_ReadTxStatus(void) ;
    void BLUE_PutChar(uint8 txDataByte) ;
    void BLUE_PutString(const char8 string[]) ;
    void BLUE_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void BLUE_PutCRLF(uint8 txDataByte) ;
    void BLUE_ClearTxBuffer(void) ;
    void BLUE_SetTxAddressMode(uint8 addressMode) ;
    void BLUE_SendBreak(uint8 retMode) ;
    uint8 BLUE_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define BLUE_PutStringConst         BLUE_PutString
    #define BLUE_PutArrayConst          BLUE_PutArray
    #define BLUE_GetTxInterruptSource   BLUE_ReadTxStatus

#endif /* End BLUE_TX_ENABLED || BLUE_HD_ENABLED */

#if(BLUE_HD_ENABLED)
    void BLUE_LoadRxConfig(void) ;
    void BLUE_LoadTxConfig(void) ;
#endif /* End BLUE_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BLUE) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    BLUE_CyBtldrCommStart(void) CYSMALL ;
    void    BLUE_CyBtldrCommStop(void) CYSMALL ;
    void    BLUE_CyBtldrCommReset(void) CYSMALL ;
    cystatus BLUE_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus BLUE_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BLUE)
        #define CyBtldrCommStart    BLUE_CyBtldrCommStart
        #define CyBtldrCommStop     BLUE_CyBtldrCommStop
        #define CyBtldrCommReset    BLUE_CyBtldrCommReset
        #define CyBtldrCommWrite    BLUE_CyBtldrCommWrite
        #define CyBtldrCommRead     BLUE_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BLUE) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define BLUE_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define BLUE_SET_SPACE                              (0x00u)
#define BLUE_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (BLUE_TX_ENABLED) || (BLUE_HD_ENABLED) )
    #if(BLUE_TX_INTERRUPT_ENABLED)
        #define BLUE_TX_VECT_NUM            (uint8)BLUE_TXInternalInterrupt__INTC_NUMBER
        #define BLUE_TX_PRIOR_NUM           (uint8)BLUE_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* BLUE_TX_INTERRUPT_ENABLED */
    #if(BLUE_TX_ENABLED)
        #define BLUE_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define BLUE_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define BLUE_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define BLUE_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* BLUE_TX_ENABLED */
    #if(BLUE_HD_ENABLED)
        #define BLUE_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define BLUE_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define BLUE_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define BLUE_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* BLUE_HD_ENABLED */
    #define BLUE_TX_STS_COMPLETE            (uint8)(0x01u << BLUE_TX_STS_COMPLETE_SHIFT)
    #define BLUE_TX_STS_FIFO_EMPTY          (uint8)(0x01u << BLUE_TX_STS_FIFO_EMPTY_SHIFT)
    #define BLUE_TX_STS_FIFO_FULL           (uint8)(0x01u << BLUE_TX_STS_FIFO_FULL_SHIFT)
    #define BLUE_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << BLUE_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (BLUE_TX_ENABLED) || (BLUE_HD_ENABLED)*/

#if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
    #if(BLUE_RX_INTERRUPT_ENABLED)
        #define BLUE_RX_VECT_NUM            (uint8)BLUE_RXInternalInterrupt__INTC_NUMBER
        #define BLUE_RX_PRIOR_NUM           (uint8)BLUE_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* BLUE_RX_INTERRUPT_ENABLED */
    #define BLUE_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define BLUE_RX_STS_BREAK_SHIFT             (0x01u)
    #define BLUE_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define BLUE_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define BLUE_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define BLUE_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define BLUE_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define BLUE_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define BLUE_RX_STS_MRKSPC           (uint8)(0x01u << BLUE_RX_STS_MRKSPC_SHIFT)
    #define BLUE_RX_STS_BREAK            (uint8)(0x01u << BLUE_RX_STS_BREAK_SHIFT)
    #define BLUE_RX_STS_PAR_ERROR        (uint8)(0x01u << BLUE_RX_STS_PAR_ERROR_SHIFT)
    #define BLUE_RX_STS_STOP_ERROR       (uint8)(0x01u << BLUE_RX_STS_STOP_ERROR_SHIFT)
    #define BLUE_RX_STS_OVERRUN          (uint8)(0x01u << BLUE_RX_STS_OVERRUN_SHIFT)
    #define BLUE_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << BLUE_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define BLUE_RX_STS_ADDR_MATCH       (uint8)(0x01u << BLUE_RX_STS_ADDR_MATCH_SHIFT)
    #define BLUE_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << BLUE_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define BLUE_RX_HW_MASK                     (0x7Fu)
#endif /* End (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) */

/* Control Register definitions */
#define BLUE_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define BLUE_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define BLUE_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define BLUE_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define BLUE_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define BLUE_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define BLUE_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define BLUE_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define BLUE_CTRL_HD_SEND               (uint8)(0x01u << BLUE_CTRL_HD_SEND_SHIFT)
#define BLUE_CTRL_HD_SEND_BREAK         (uint8)(0x01u << BLUE_CTRL_HD_SEND_BREAK_SHIFT)
#define BLUE_CTRL_MARK                  (uint8)(0x01u << BLUE_CTRL_MARK_SHIFT)
#define BLUE_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << BLUE_CTRL_PARITY_TYPE0_SHIFT)
#define BLUE_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << BLUE_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define BLUE_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define BLUE_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define BLUE_SEND_BREAK                         (0x00u)
#define BLUE_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define BLUE_REINIT                             (0x02u)
#define BLUE_SEND_WAIT_REINIT                   (0x03u)

#define BLUE_OVER_SAMPLE_8                      (8u)
#define BLUE_OVER_SAMPLE_16                     (16u)

#define BLUE_BIT_CENTER                         (BLUE_OVER_SAMPLE_COUNT - 1u)

#define BLUE_FIFO_LENGTH                        (4u)
#define BLUE_NUMBER_OF_START_BIT                (1u)
#define BLUE_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define BLUE_TXBITCTR_BREAKBITS8X   ((BLUE_BREAK_BITS_TX * BLUE_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define BLUE_TXBITCTR_BREAKBITS ((BLUE_BREAK_BITS_TX * BLUE_OVER_SAMPLE_COUNT) - 1u)

#define BLUE_HALF_BIT_COUNT   \
                            (((BLUE_OVER_SAMPLE_COUNT / 2u) + (BLUE_USE23POLLING * 1u)) - 2u)
#if (BLUE_OVER_SAMPLE_COUNT == BLUE_OVER_SAMPLE_8)
    #define BLUE_HD_TXBITCTR_INIT   (((BLUE_BREAK_BITS_TX + \
                            BLUE_NUMBER_OF_START_BIT) * BLUE_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define BLUE_RXBITCTR_INIT  ((((BLUE_BREAK_BITS_RX + BLUE_NUMBER_OF_START_BIT) \
                            * BLUE_OVER_SAMPLE_COUNT) + BLUE_HALF_BIT_COUNT) - 1u)


#else /* BLUE_OVER_SAMPLE_COUNT == BLUE_OVER_SAMPLE_16 */
    #define BLUE_HD_TXBITCTR_INIT   ((8u * BLUE_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define BLUE_RXBITCTR_INIT      (((7u * BLUE_OVER_SAMPLE_COUNT) - 1u) + \
                                                      BLUE_HALF_BIT_COUNT)
#endif /* End BLUE_OVER_SAMPLE_COUNT */
#define BLUE_HD_RXBITCTR_INIT                   BLUE_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 BLUE_initVar;
#if( BLUE_TX_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))
    extern volatile uint8 BLUE_txBuffer[BLUE_TXBUFFERSIZE];
    extern volatile uint8 BLUE_txBufferRead;
    extern uint8 BLUE_txBufferWrite;
#endif /* End BLUE_TX_ENABLED */
#if( ( BLUE_RX_ENABLED || BLUE_HD_ENABLED ) && \
     (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH) )
    extern volatile uint8 BLUE_rxBuffer[BLUE_RXBUFFERSIZE];
    extern volatile uint8 BLUE_rxBufferRead;
    extern volatile uint8 BLUE_rxBufferWrite;
    extern volatile uint8 BLUE_rxBufferLoopDetect;
    extern volatile uint8 BLUE_rxBufferOverflow;
    #if (BLUE_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 BLUE_rxAddressMode;
        extern volatile uint8 BLUE_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End BLUE_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define BLUE__B_UART__AM_SW_BYTE_BYTE 1
#define BLUE__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define BLUE__B_UART__AM_HW_BYTE_BY_BYTE 3
#define BLUE__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define BLUE__B_UART__AM_NONE 0

#define BLUE__B_UART__NONE_REVB 0
#define BLUE__B_UART__EVEN_REVB 1
#define BLUE__B_UART__ODD_REVB 2
#define BLUE__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define BLUE_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define BLUE_NUMBER_OF_STOP_BITS    (1u)

#if (BLUE_RXHW_ADDRESS_ENABLED)
    #define BLUE_RXADDRESSMODE      (0u)
    #define BLUE_RXHWADDRESS1       (0u)
    #define BLUE_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define BLUE_RXAddressMode      BLUE_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define BLUE_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << BLUE_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << BLUE_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << BLUE_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << BLUE_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << BLUE_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << BLUE_RX_STS_BREAK_SHIFT) \
                                        | (0 << BLUE_RX_STS_OVERRUN_SHIFT))

#define BLUE_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << BLUE_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << BLUE_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << BLUE_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << BLUE_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define BLUE_CONTROL_REG \
                            (* (reg8 *) BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define BLUE_CONTROL_PTR \
                            (  (reg8 *) BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(BLUE_TX_ENABLED)
    #define BLUE_TXDATA_REG          (* (reg8 *) BLUE_BUART_sTX_TxShifter_u0__F0_REG)
    #define BLUE_TXDATA_PTR          (  (reg8 *) BLUE_BUART_sTX_TxShifter_u0__F0_REG)
    #define BLUE_TXDATA_AUX_CTL_REG  (* (reg8 *) BLUE_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define BLUE_TXDATA_AUX_CTL_PTR  (  (reg8 *) BLUE_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define BLUE_TXSTATUS_REG        (* (reg8 *) BLUE_BUART_sTX_TxSts__STATUS_REG)
    #define BLUE_TXSTATUS_PTR        (  (reg8 *) BLUE_BUART_sTX_TxSts__STATUS_REG)
    #define BLUE_TXSTATUS_MASK_REG   (* (reg8 *) BLUE_BUART_sTX_TxSts__MASK_REG)
    #define BLUE_TXSTATUS_MASK_PTR   (  (reg8 *) BLUE_BUART_sTX_TxSts__MASK_REG)
    #define BLUE_TXSTATUS_ACTL_REG   (* (reg8 *) BLUE_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define BLUE_TXSTATUS_ACTL_PTR   (  (reg8 *) BLUE_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(BLUE_TXCLKGEN_DP)
        #define BLUE_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define BLUE_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define BLUE_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define BLUE_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define BLUE_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define BLUE_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define BLUE_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define BLUE_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define BLUE_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define BLUE_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) BLUE_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* BLUE_TXCLKGEN_DP */

#endif /* End BLUE_TX_ENABLED */

#if(BLUE_HD_ENABLED)

    #define BLUE_TXDATA_REG             (* (reg8 *) BLUE_BUART_sRX_RxShifter_u0__F1_REG )
    #define BLUE_TXDATA_PTR             (  (reg8 *) BLUE_BUART_sRX_RxShifter_u0__F1_REG )
    #define BLUE_TXDATA_AUX_CTL_REG     (* (reg8 *) BLUE_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define BLUE_TXDATA_AUX_CTL_PTR     (  (reg8 *) BLUE_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define BLUE_TXSTATUS_REG           (* (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_REG )
    #define BLUE_TXSTATUS_PTR           (  (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_REG )
    #define BLUE_TXSTATUS_MASK_REG      (* (reg8 *) BLUE_BUART_sRX_RxSts__MASK_REG )
    #define BLUE_TXSTATUS_MASK_PTR      (  (reg8 *) BLUE_BUART_sRX_RxSts__MASK_REG )
    #define BLUE_TXSTATUS_ACTL_REG      (* (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define BLUE_TXSTATUS_ACTL_PTR      (  (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End BLUE_HD_ENABLED */

#if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
    #define BLUE_RXDATA_REG             (* (reg8 *) BLUE_BUART_sRX_RxShifter_u0__F0_REG )
    #define BLUE_RXDATA_PTR             (  (reg8 *) BLUE_BUART_sRX_RxShifter_u0__F0_REG )
    #define BLUE_RXADDRESS1_REG         (* (reg8 *) BLUE_BUART_sRX_RxShifter_u0__D0_REG )
    #define BLUE_RXADDRESS1_PTR         (  (reg8 *) BLUE_BUART_sRX_RxShifter_u0__D0_REG )
    #define BLUE_RXADDRESS2_REG         (* (reg8 *) BLUE_BUART_sRX_RxShifter_u0__D1_REG )
    #define BLUE_RXADDRESS2_PTR         (  (reg8 *) BLUE_BUART_sRX_RxShifter_u0__D1_REG )
    #define BLUE_RXDATA_AUX_CTL_REG     (* (reg8 *) BLUE_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define BLUE_RXBITCTR_PERIOD_REG    (* (reg8 *) BLUE_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define BLUE_RXBITCTR_PERIOD_PTR    (  (reg8 *) BLUE_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define BLUE_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) BLUE_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define BLUE_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) BLUE_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define BLUE_RXBITCTR_COUNTER_REG   (* (reg8 *) BLUE_BUART_sRX_RxBitCounter__COUNT_REG )
    #define BLUE_RXBITCTR_COUNTER_PTR   (  (reg8 *) BLUE_BUART_sRX_RxBitCounter__COUNT_REG )

    #define BLUE_RXSTATUS_REG           (* (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_REG )
    #define BLUE_RXSTATUS_PTR           (  (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_REG )
    #define BLUE_RXSTATUS_MASK_REG      (* (reg8 *) BLUE_BUART_sRX_RxSts__MASK_REG )
    #define BLUE_RXSTATUS_MASK_PTR      (  (reg8 *) BLUE_BUART_sRX_RxSts__MASK_REG )
    #define BLUE_RXSTATUS_ACTL_REG      (* (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define BLUE_RXSTATUS_ACTL_PTR      (  (reg8 *) BLUE_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) */

#if(BLUE_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define BLUE_INTCLOCK_CLKEN_REG     (* (reg8 *) BLUE_IntClock__PM_ACT_CFG)
    #define BLUE_INTCLOCK_CLKEN_PTR     (  (reg8 *) BLUE_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define BLUE_INTCLOCK_CLKEN_MASK    BLUE_IntClock__PM_ACT_MSK
#endif /* End BLUE_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(BLUE_TX_ENABLED)
    #define BLUE_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End BLUE_TX_ENABLED */

#if(BLUE_HD_ENABLED)
    #define BLUE_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End BLUE_HD_ENABLED */

#if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
    #define BLUE_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define BLUE_initvar                    BLUE_initVar

#define BLUE_RX_Enabled                 BLUE_RX_ENABLED
#define BLUE_TX_Enabled                 BLUE_TX_ENABLED
#define BLUE_HD_Enabled                 BLUE_HD_ENABLED
#define BLUE_RX_IntInterruptEnabled     BLUE_RX_INTERRUPT_ENABLED
#define BLUE_TX_IntInterruptEnabled     BLUE_TX_INTERRUPT_ENABLED
#define BLUE_InternalClockUsed          BLUE_INTERNAL_CLOCK_USED
#define BLUE_RXHW_Address_Enabled       BLUE_RXHW_ADDRESS_ENABLED
#define BLUE_OverSampleCount            BLUE_OVER_SAMPLE_COUNT
#define BLUE_ParityType                 BLUE_PARITY_TYPE

#if( BLUE_TX_ENABLED && (BLUE_TXBUFFERSIZE > BLUE_FIFO_LENGTH))
    #define BLUE_TXBUFFER               BLUE_txBuffer
    #define BLUE_TXBUFFERREAD           BLUE_txBufferRead
    #define BLUE_TXBUFFERWRITE          BLUE_txBufferWrite
#endif /* End BLUE_TX_ENABLED */
#if( ( BLUE_RX_ENABLED || BLUE_HD_ENABLED ) && \
     (BLUE_RXBUFFERSIZE > BLUE_FIFO_LENGTH) )
    #define BLUE_RXBUFFER               BLUE_rxBuffer
    #define BLUE_RXBUFFERREAD           BLUE_rxBufferRead
    #define BLUE_RXBUFFERWRITE          BLUE_rxBufferWrite
    #define BLUE_RXBUFFERLOOPDETECT     BLUE_rxBufferLoopDetect
    #define BLUE_RXBUFFER_OVERFLOW      BLUE_rxBufferOverflow
#endif /* End BLUE_RX_ENABLED */

#ifdef BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define BLUE_CONTROL                BLUE_CONTROL_REG
#endif /* End BLUE_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(BLUE_TX_ENABLED)
    #define BLUE_TXDATA                 BLUE_TXDATA_REG
    #define BLUE_TXSTATUS               BLUE_TXSTATUS_REG
    #define BLUE_TXSTATUS_MASK          BLUE_TXSTATUS_MASK_REG
    #define BLUE_TXSTATUS_ACTL          BLUE_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(BLUE_TXCLKGEN_DP)
        #define BLUE_TXBITCLKGEN_CTR        BLUE_TXBITCLKGEN_CTR_REG
        #define BLUE_TXBITCLKTX_COMPLETE    BLUE_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define BLUE_TXBITCTR_PERIOD        BLUE_TXBITCTR_PERIOD_REG
        #define BLUE_TXBITCTR_CONTROL       BLUE_TXBITCTR_CONTROL_REG
        #define BLUE_TXBITCTR_COUNTER       BLUE_TXBITCTR_COUNTER_REG
    #endif /* BLUE_TXCLKGEN_DP */
#endif /* End BLUE_TX_ENABLED */

#if(BLUE_HD_ENABLED)
    #define BLUE_TXDATA                 BLUE_TXDATA_REG
    #define BLUE_TXSTATUS               BLUE_TXSTATUS_REG
    #define BLUE_TXSTATUS_MASK          BLUE_TXSTATUS_MASK_REG
    #define BLUE_TXSTATUS_ACTL          BLUE_TXSTATUS_ACTL_REG
#endif /* End BLUE_HD_ENABLED */

#if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
    #define BLUE_RXDATA                 BLUE_RXDATA_REG
    #define BLUE_RXADDRESS1             BLUE_RXADDRESS1_REG
    #define BLUE_RXADDRESS2             BLUE_RXADDRESS2_REG
    #define BLUE_RXBITCTR_PERIOD        BLUE_RXBITCTR_PERIOD_REG
    #define BLUE_RXBITCTR_CONTROL       BLUE_RXBITCTR_CONTROL_REG
    #define BLUE_RXBITCTR_COUNTER       BLUE_RXBITCTR_COUNTER_REG
    #define BLUE_RXSTATUS               BLUE_RXSTATUS_REG
    #define BLUE_RXSTATUS_MASK          BLUE_RXSTATUS_MASK_REG
    #define BLUE_RXSTATUS_ACTL          BLUE_RXSTATUS_ACTL_REG
#endif /* End  (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) */

#if(BLUE_INTERNAL_CLOCK_USED)
    #define BLUE_INTCLOCK_CLKEN         BLUE_INTCLOCK_CLKEN_REG
#endif /* End BLUE_INTERNAL_CLOCK_USED */

#define BLUE_WAIT_FOR_COMLETE_REINIT    BLUE_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_BLUE_H */


/* [] END OF FILE */
