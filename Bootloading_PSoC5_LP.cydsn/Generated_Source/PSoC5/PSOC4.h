/*******************************************************************************
* File Name: PSOC4.h
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


#if !defined(CY_UART_PSOC4_H)
#define CY_UART_PSOC4_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define PSOC4_RX_ENABLED                     (1u)
#define PSOC4_TX_ENABLED                     (1u)
#define PSOC4_HD_ENABLED                     (0u)
#define PSOC4_RX_INTERRUPT_ENABLED           (1u)
#define PSOC4_TX_INTERRUPT_ENABLED           (0u)
#define PSOC4_INTERNAL_CLOCK_USED            (1u)
#define PSOC4_RXHW_ADDRESS_ENABLED           (0u)
#define PSOC4_OVER_SAMPLE_COUNT              (8u)
#define PSOC4_PARITY_TYPE                    (0u)
#define PSOC4_PARITY_TYPE_SW                 (0u)
#define PSOC4_BREAK_DETECT                   (0u)
#define PSOC4_BREAK_BITS_TX                  (13u)
#define PSOC4_BREAK_BITS_RX                  (13u)
#define PSOC4_TXCLKGEN_DP                    (1u)
#define PSOC4_USE23POLLING                   (1u)
#define PSOC4_FLOW_CONTROL                   (0u)
#define PSOC4_CLK_FREQ                       (0u)
#define PSOC4_TXBUFFERSIZE                   (4u)
#define PSOC4_RXBUFFERSIZE                   (50u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define PSOC4_CONTROL_REG_REMOVED            (0u)
#else
    #define PSOC4_CONTROL_REG_REMOVED            (1u)
#endif /* End PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct PSOC4_backupStruct_
{
    uint8 enableState;

    #if(PSOC4_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End PSOC4_CONTROL_REG_REMOVED */
    #if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (PSOC4_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End PSOC4_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED)*/

    #if(PSOC4_TX_ENABLED)
        #if(PSOC4_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End PSOC4_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End PSOC4_TX_ENABLED */
} PSOC4_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void PSOC4_Start(void) ;
void PSOC4_Stop(void) ;
uint8 PSOC4_ReadControlRegister(void) ;
void PSOC4_WriteControlRegister(uint8 control) ;

void PSOC4_Init(void) ;
void PSOC4_Enable(void) ;
void PSOC4_SaveConfig(void) ;
void PSOC4_RestoreConfig(void) ;
void PSOC4_Sleep(void) ;
void PSOC4_Wakeup(void) ;

/* Only if RX is enabled */
#if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )

    #if(PSOC4_RX_INTERRUPT_ENABLED)
        void  PSOC4_EnableRxInt(void) ;
        void  PSOC4_DisableRxInt(void) ;
        CY_ISR_PROTO(PSOC4_RXISR);
    #endif /* PSOC4_RX_INTERRUPT_ENABLED */

    void PSOC4_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void PSOC4_SetRxAddress1(uint8 address) ;
    void PSOC4_SetRxAddress2(uint8 address) ;

    void  PSOC4_SetRxInterruptMode(uint8 intSrc) ;
    uint8 PSOC4_ReadRxData(void) ;
    uint8 PSOC4_ReadRxStatus(void) ;
    uint8 PSOC4_GetChar(void) ;
    uint16 PSOC4_GetByte(void) ;
    uint8 PSOC4_GetRxBufferSize(void)
                                                            ;
    void PSOC4_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define PSOC4_GetRxInterruptSource   PSOC4_ReadRxStatus

#endif /* End (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) */

/* Only if TX is enabled */
#if(PSOC4_TX_ENABLED || PSOC4_HD_ENABLED)

    #if(PSOC4_TX_INTERRUPT_ENABLED)
        void PSOC4_EnableTxInt(void) ;
        void PSOC4_DisableTxInt(void) ;
        CY_ISR_PROTO(PSOC4_TXISR);
    #endif /* PSOC4_TX_INTERRUPT_ENABLED */

    void PSOC4_SetTxInterruptMode(uint8 intSrc) ;
    void PSOC4_WriteTxData(uint8 txDataByte) ;
    uint8 PSOC4_ReadTxStatus(void) ;
    void PSOC4_PutChar(uint8 txDataByte) ;
    void PSOC4_PutString(const char8 string[]) ;
    void PSOC4_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void PSOC4_PutCRLF(uint8 txDataByte) ;
    void PSOC4_ClearTxBuffer(void) ;
    void PSOC4_SetTxAddressMode(uint8 addressMode) ;
    void PSOC4_SendBreak(uint8 retMode) ;
    uint8 PSOC4_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define PSOC4_PutStringConst         PSOC4_PutString
    #define PSOC4_PutArrayConst          PSOC4_PutArray
    #define PSOC4_GetTxInterruptSource   PSOC4_ReadTxStatus

#endif /* End PSOC4_TX_ENABLED || PSOC4_HD_ENABLED */

#if(PSOC4_HD_ENABLED)
    void PSOC4_LoadRxConfig(void) ;
    void PSOC4_LoadTxConfig(void) ;
#endif /* End PSOC4_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC4) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    PSOC4_CyBtldrCommStart(void) CYSMALL ;
    void    PSOC4_CyBtldrCommStop(void) CYSMALL ;
    void    PSOC4_CyBtldrCommReset(void) CYSMALL ;
    cystatus PSOC4_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus PSOC4_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC4)
        #define CyBtldrCommStart    PSOC4_CyBtldrCommStart
        #define CyBtldrCommStop     PSOC4_CyBtldrCommStop
        #define CyBtldrCommReset    PSOC4_CyBtldrCommReset
        #define CyBtldrCommWrite    PSOC4_CyBtldrCommWrite
        #define CyBtldrCommRead     PSOC4_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC4) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define PSOC4_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define PSOC4_SET_SPACE                              (0x00u)
#define PSOC4_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (PSOC4_TX_ENABLED) || (PSOC4_HD_ENABLED) )
    #if(PSOC4_TX_INTERRUPT_ENABLED)
        #define PSOC4_TX_VECT_NUM            (uint8)PSOC4_TXInternalInterrupt__INTC_NUMBER
        #define PSOC4_TX_PRIOR_NUM           (uint8)PSOC4_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* PSOC4_TX_INTERRUPT_ENABLED */
    #if(PSOC4_TX_ENABLED)
        #define PSOC4_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define PSOC4_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define PSOC4_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define PSOC4_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* PSOC4_TX_ENABLED */
    #if(PSOC4_HD_ENABLED)
        #define PSOC4_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define PSOC4_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define PSOC4_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define PSOC4_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* PSOC4_HD_ENABLED */
    #define PSOC4_TX_STS_COMPLETE            (uint8)(0x01u << PSOC4_TX_STS_COMPLETE_SHIFT)
    #define PSOC4_TX_STS_FIFO_EMPTY          (uint8)(0x01u << PSOC4_TX_STS_FIFO_EMPTY_SHIFT)
    #define PSOC4_TX_STS_FIFO_FULL           (uint8)(0x01u << PSOC4_TX_STS_FIFO_FULL_SHIFT)
    #define PSOC4_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << PSOC4_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (PSOC4_TX_ENABLED) || (PSOC4_HD_ENABLED)*/

#if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
    #if(PSOC4_RX_INTERRUPT_ENABLED)
        #define PSOC4_RX_VECT_NUM            (uint8)PSOC4_RXInternalInterrupt__INTC_NUMBER
        #define PSOC4_RX_PRIOR_NUM           (uint8)PSOC4_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* PSOC4_RX_INTERRUPT_ENABLED */
    #define PSOC4_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define PSOC4_RX_STS_BREAK_SHIFT             (0x01u)
    #define PSOC4_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define PSOC4_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define PSOC4_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define PSOC4_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define PSOC4_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define PSOC4_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define PSOC4_RX_STS_MRKSPC           (uint8)(0x01u << PSOC4_RX_STS_MRKSPC_SHIFT)
    #define PSOC4_RX_STS_BREAK            (uint8)(0x01u << PSOC4_RX_STS_BREAK_SHIFT)
    #define PSOC4_RX_STS_PAR_ERROR        (uint8)(0x01u << PSOC4_RX_STS_PAR_ERROR_SHIFT)
    #define PSOC4_RX_STS_STOP_ERROR       (uint8)(0x01u << PSOC4_RX_STS_STOP_ERROR_SHIFT)
    #define PSOC4_RX_STS_OVERRUN          (uint8)(0x01u << PSOC4_RX_STS_OVERRUN_SHIFT)
    #define PSOC4_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << PSOC4_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define PSOC4_RX_STS_ADDR_MATCH       (uint8)(0x01u << PSOC4_RX_STS_ADDR_MATCH_SHIFT)
    #define PSOC4_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << PSOC4_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define PSOC4_RX_HW_MASK                     (0x7Fu)
#endif /* End (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) */

/* Control Register definitions */
#define PSOC4_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define PSOC4_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define PSOC4_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define PSOC4_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define PSOC4_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define PSOC4_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define PSOC4_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define PSOC4_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define PSOC4_CTRL_HD_SEND               (uint8)(0x01u << PSOC4_CTRL_HD_SEND_SHIFT)
#define PSOC4_CTRL_HD_SEND_BREAK         (uint8)(0x01u << PSOC4_CTRL_HD_SEND_BREAK_SHIFT)
#define PSOC4_CTRL_MARK                  (uint8)(0x01u << PSOC4_CTRL_MARK_SHIFT)
#define PSOC4_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << PSOC4_CTRL_PARITY_TYPE0_SHIFT)
#define PSOC4_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << PSOC4_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define PSOC4_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define PSOC4_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define PSOC4_SEND_BREAK                         (0x00u)
#define PSOC4_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define PSOC4_REINIT                             (0x02u)
#define PSOC4_SEND_WAIT_REINIT                   (0x03u)

#define PSOC4_OVER_SAMPLE_8                      (8u)
#define PSOC4_OVER_SAMPLE_16                     (16u)

#define PSOC4_BIT_CENTER                         (PSOC4_OVER_SAMPLE_COUNT - 1u)

#define PSOC4_FIFO_LENGTH                        (4u)
#define PSOC4_NUMBER_OF_START_BIT                (1u)
#define PSOC4_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define PSOC4_TXBITCTR_BREAKBITS8X   ((PSOC4_BREAK_BITS_TX * PSOC4_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define PSOC4_TXBITCTR_BREAKBITS ((PSOC4_BREAK_BITS_TX * PSOC4_OVER_SAMPLE_COUNT) - 1u)

#define PSOC4_HALF_BIT_COUNT   \
                            (((PSOC4_OVER_SAMPLE_COUNT / 2u) + (PSOC4_USE23POLLING * 1u)) - 2u)
#if (PSOC4_OVER_SAMPLE_COUNT == PSOC4_OVER_SAMPLE_8)
    #define PSOC4_HD_TXBITCTR_INIT   (((PSOC4_BREAK_BITS_TX + \
                            PSOC4_NUMBER_OF_START_BIT) * PSOC4_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define PSOC4_RXBITCTR_INIT  ((((PSOC4_BREAK_BITS_RX + PSOC4_NUMBER_OF_START_BIT) \
                            * PSOC4_OVER_SAMPLE_COUNT) + PSOC4_HALF_BIT_COUNT) - 1u)


#else /* PSOC4_OVER_SAMPLE_COUNT == PSOC4_OVER_SAMPLE_16 */
    #define PSOC4_HD_TXBITCTR_INIT   ((8u * PSOC4_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define PSOC4_RXBITCTR_INIT      (((7u * PSOC4_OVER_SAMPLE_COUNT) - 1u) + \
                                                      PSOC4_HALF_BIT_COUNT)
#endif /* End PSOC4_OVER_SAMPLE_COUNT */
#define PSOC4_HD_RXBITCTR_INIT                   PSOC4_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 PSOC4_initVar;
#if( PSOC4_TX_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))
    extern volatile uint8 PSOC4_txBuffer[PSOC4_TXBUFFERSIZE];
    extern volatile uint8 PSOC4_txBufferRead;
    extern uint8 PSOC4_txBufferWrite;
#endif /* End PSOC4_TX_ENABLED */
#if( ( PSOC4_RX_ENABLED || PSOC4_HD_ENABLED ) && \
     (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH) )
    extern volatile uint8 PSOC4_rxBuffer[PSOC4_RXBUFFERSIZE];
    extern volatile uint8 PSOC4_rxBufferRead;
    extern volatile uint8 PSOC4_rxBufferWrite;
    extern volatile uint8 PSOC4_rxBufferLoopDetect;
    extern volatile uint8 PSOC4_rxBufferOverflow;
    #if (PSOC4_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 PSOC4_rxAddressMode;
        extern volatile uint8 PSOC4_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End PSOC4_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define PSOC4__B_UART__AM_SW_BYTE_BYTE 1
#define PSOC4__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define PSOC4__B_UART__AM_HW_BYTE_BY_BYTE 3
#define PSOC4__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define PSOC4__B_UART__AM_NONE 0

#define PSOC4__B_UART__NONE_REVB 0
#define PSOC4__B_UART__EVEN_REVB 1
#define PSOC4__B_UART__ODD_REVB 2
#define PSOC4__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define PSOC4_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define PSOC4_NUMBER_OF_STOP_BITS    (1u)

#if (PSOC4_RXHW_ADDRESS_ENABLED)
    #define PSOC4_RXADDRESSMODE      (0u)
    #define PSOC4_RXHWADDRESS1       (0u)
    #define PSOC4_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define PSOC4_RXAddressMode      PSOC4_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define PSOC4_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << PSOC4_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << PSOC4_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << PSOC4_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << PSOC4_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << PSOC4_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << PSOC4_RX_STS_BREAK_SHIFT) \
                                        | (0 << PSOC4_RX_STS_OVERRUN_SHIFT))

#define PSOC4_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << PSOC4_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << PSOC4_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << PSOC4_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << PSOC4_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define PSOC4_CONTROL_REG \
                            (* (reg8 *) PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define PSOC4_CONTROL_PTR \
                            (  (reg8 *) PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(PSOC4_TX_ENABLED)
    #define PSOC4_TXDATA_REG          (* (reg8 *) PSOC4_BUART_sTX_TxShifter_u0__F0_REG)
    #define PSOC4_TXDATA_PTR          (  (reg8 *) PSOC4_BUART_sTX_TxShifter_u0__F0_REG)
    #define PSOC4_TXDATA_AUX_CTL_REG  (* (reg8 *) PSOC4_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define PSOC4_TXDATA_AUX_CTL_PTR  (  (reg8 *) PSOC4_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define PSOC4_TXSTATUS_REG        (* (reg8 *) PSOC4_BUART_sTX_TxSts__STATUS_REG)
    #define PSOC4_TXSTATUS_PTR        (  (reg8 *) PSOC4_BUART_sTX_TxSts__STATUS_REG)
    #define PSOC4_TXSTATUS_MASK_REG   (* (reg8 *) PSOC4_BUART_sTX_TxSts__MASK_REG)
    #define PSOC4_TXSTATUS_MASK_PTR   (  (reg8 *) PSOC4_BUART_sTX_TxSts__MASK_REG)
    #define PSOC4_TXSTATUS_ACTL_REG   (* (reg8 *) PSOC4_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define PSOC4_TXSTATUS_ACTL_PTR   (  (reg8 *) PSOC4_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(PSOC4_TXCLKGEN_DP)
        #define PSOC4_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define PSOC4_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define PSOC4_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define PSOC4_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define PSOC4_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define PSOC4_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define PSOC4_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define PSOC4_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define PSOC4_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define PSOC4_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) PSOC4_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* PSOC4_TXCLKGEN_DP */

#endif /* End PSOC4_TX_ENABLED */

#if(PSOC4_HD_ENABLED)

    #define PSOC4_TXDATA_REG             (* (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__F1_REG )
    #define PSOC4_TXDATA_PTR             (  (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__F1_REG )
    #define PSOC4_TXDATA_AUX_CTL_REG     (* (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define PSOC4_TXDATA_AUX_CTL_PTR     (  (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define PSOC4_TXSTATUS_REG           (* (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_REG )
    #define PSOC4_TXSTATUS_PTR           (  (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_REG )
    #define PSOC4_TXSTATUS_MASK_REG      (* (reg8 *) PSOC4_BUART_sRX_RxSts__MASK_REG )
    #define PSOC4_TXSTATUS_MASK_PTR      (  (reg8 *) PSOC4_BUART_sRX_RxSts__MASK_REG )
    #define PSOC4_TXSTATUS_ACTL_REG      (* (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define PSOC4_TXSTATUS_ACTL_PTR      (  (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End PSOC4_HD_ENABLED */

#if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
    #define PSOC4_RXDATA_REG             (* (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__F0_REG )
    #define PSOC4_RXDATA_PTR             (  (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__F0_REG )
    #define PSOC4_RXADDRESS1_REG         (* (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__D0_REG )
    #define PSOC4_RXADDRESS1_PTR         (  (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__D0_REG )
    #define PSOC4_RXADDRESS2_REG         (* (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__D1_REG )
    #define PSOC4_RXADDRESS2_PTR         (  (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__D1_REG )
    #define PSOC4_RXDATA_AUX_CTL_REG     (* (reg8 *) PSOC4_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define PSOC4_RXBITCTR_PERIOD_REG    (* (reg8 *) PSOC4_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define PSOC4_RXBITCTR_PERIOD_PTR    (  (reg8 *) PSOC4_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define PSOC4_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) PSOC4_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define PSOC4_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) PSOC4_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define PSOC4_RXBITCTR_COUNTER_REG   (* (reg8 *) PSOC4_BUART_sRX_RxBitCounter__COUNT_REG )
    #define PSOC4_RXBITCTR_COUNTER_PTR   (  (reg8 *) PSOC4_BUART_sRX_RxBitCounter__COUNT_REG )

    #define PSOC4_RXSTATUS_REG           (* (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_REG )
    #define PSOC4_RXSTATUS_PTR           (  (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_REG )
    #define PSOC4_RXSTATUS_MASK_REG      (* (reg8 *) PSOC4_BUART_sRX_RxSts__MASK_REG )
    #define PSOC4_RXSTATUS_MASK_PTR      (  (reg8 *) PSOC4_BUART_sRX_RxSts__MASK_REG )
    #define PSOC4_RXSTATUS_ACTL_REG      (* (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define PSOC4_RXSTATUS_ACTL_PTR      (  (reg8 *) PSOC4_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) */

#if(PSOC4_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define PSOC4_INTCLOCK_CLKEN_REG     (* (reg8 *) PSOC4_IntClock__PM_ACT_CFG)
    #define PSOC4_INTCLOCK_CLKEN_PTR     (  (reg8 *) PSOC4_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define PSOC4_INTCLOCK_CLKEN_MASK    PSOC4_IntClock__PM_ACT_MSK
#endif /* End PSOC4_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(PSOC4_TX_ENABLED)
    #define PSOC4_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End PSOC4_TX_ENABLED */

#if(PSOC4_HD_ENABLED)
    #define PSOC4_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End PSOC4_HD_ENABLED */

#if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
    #define PSOC4_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define PSOC4_initvar                    PSOC4_initVar

#define PSOC4_RX_Enabled                 PSOC4_RX_ENABLED
#define PSOC4_TX_Enabled                 PSOC4_TX_ENABLED
#define PSOC4_HD_Enabled                 PSOC4_HD_ENABLED
#define PSOC4_RX_IntInterruptEnabled     PSOC4_RX_INTERRUPT_ENABLED
#define PSOC4_TX_IntInterruptEnabled     PSOC4_TX_INTERRUPT_ENABLED
#define PSOC4_InternalClockUsed          PSOC4_INTERNAL_CLOCK_USED
#define PSOC4_RXHW_Address_Enabled       PSOC4_RXHW_ADDRESS_ENABLED
#define PSOC4_OverSampleCount            PSOC4_OVER_SAMPLE_COUNT
#define PSOC4_ParityType                 PSOC4_PARITY_TYPE

#if( PSOC4_TX_ENABLED && (PSOC4_TXBUFFERSIZE > PSOC4_FIFO_LENGTH))
    #define PSOC4_TXBUFFER               PSOC4_txBuffer
    #define PSOC4_TXBUFFERREAD           PSOC4_txBufferRead
    #define PSOC4_TXBUFFERWRITE          PSOC4_txBufferWrite
#endif /* End PSOC4_TX_ENABLED */
#if( ( PSOC4_RX_ENABLED || PSOC4_HD_ENABLED ) && \
     (PSOC4_RXBUFFERSIZE > PSOC4_FIFO_LENGTH) )
    #define PSOC4_RXBUFFER               PSOC4_rxBuffer
    #define PSOC4_RXBUFFERREAD           PSOC4_rxBufferRead
    #define PSOC4_RXBUFFERWRITE          PSOC4_rxBufferWrite
    #define PSOC4_RXBUFFERLOOPDETECT     PSOC4_rxBufferLoopDetect
    #define PSOC4_RXBUFFER_OVERFLOW      PSOC4_rxBufferOverflow
#endif /* End PSOC4_RX_ENABLED */

#ifdef PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define PSOC4_CONTROL                PSOC4_CONTROL_REG
#endif /* End PSOC4_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(PSOC4_TX_ENABLED)
    #define PSOC4_TXDATA                 PSOC4_TXDATA_REG
    #define PSOC4_TXSTATUS               PSOC4_TXSTATUS_REG
    #define PSOC4_TXSTATUS_MASK          PSOC4_TXSTATUS_MASK_REG
    #define PSOC4_TXSTATUS_ACTL          PSOC4_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(PSOC4_TXCLKGEN_DP)
        #define PSOC4_TXBITCLKGEN_CTR        PSOC4_TXBITCLKGEN_CTR_REG
        #define PSOC4_TXBITCLKTX_COMPLETE    PSOC4_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define PSOC4_TXBITCTR_PERIOD        PSOC4_TXBITCTR_PERIOD_REG
        #define PSOC4_TXBITCTR_CONTROL       PSOC4_TXBITCTR_CONTROL_REG
        #define PSOC4_TXBITCTR_COUNTER       PSOC4_TXBITCTR_COUNTER_REG
    #endif /* PSOC4_TXCLKGEN_DP */
#endif /* End PSOC4_TX_ENABLED */

#if(PSOC4_HD_ENABLED)
    #define PSOC4_TXDATA                 PSOC4_TXDATA_REG
    #define PSOC4_TXSTATUS               PSOC4_TXSTATUS_REG
    #define PSOC4_TXSTATUS_MASK          PSOC4_TXSTATUS_MASK_REG
    #define PSOC4_TXSTATUS_ACTL          PSOC4_TXSTATUS_ACTL_REG
#endif /* End PSOC4_HD_ENABLED */

#if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
    #define PSOC4_RXDATA                 PSOC4_RXDATA_REG
    #define PSOC4_RXADDRESS1             PSOC4_RXADDRESS1_REG
    #define PSOC4_RXADDRESS2             PSOC4_RXADDRESS2_REG
    #define PSOC4_RXBITCTR_PERIOD        PSOC4_RXBITCTR_PERIOD_REG
    #define PSOC4_RXBITCTR_CONTROL       PSOC4_RXBITCTR_CONTROL_REG
    #define PSOC4_RXBITCTR_COUNTER       PSOC4_RXBITCTR_COUNTER_REG
    #define PSOC4_RXSTATUS               PSOC4_RXSTATUS_REG
    #define PSOC4_RXSTATUS_MASK          PSOC4_RXSTATUS_MASK_REG
    #define PSOC4_RXSTATUS_ACTL          PSOC4_RXSTATUS_ACTL_REG
#endif /* End  (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) */

#if(PSOC4_INTERNAL_CLOCK_USED)
    #define PSOC4_INTCLOCK_CLKEN         PSOC4_INTCLOCK_CLKEN_REG
#endif /* End PSOC4_INTERNAL_CLOCK_USED */

#define PSOC4_WAIT_FOR_COMLETE_REINIT    PSOC4_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_PSOC4_H */


/* [] END OF FILE */
