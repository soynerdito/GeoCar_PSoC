/*******************************************************************************
* File Name: PSOC5.h
* Version 1.10
*
* Description:
*  This file provides constants and parameter values for the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PSOC5_H)
#define CY_SCB_PSOC5_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h"


/***************************************
*  Conditional Compilation Parameters
****************************************/

#define PSOC5_SCB_MODE                     (4u)

/* SCB modes enum */
#define PSOC5_SCB_MODE_I2C                 (0x01u)
#define PSOC5_SCB_MODE_SPI                 (0x02u)
#define PSOC5_SCB_MODE_UART                (0x04u)
#define PSOC5_SCB_MODE_EZI2C               (0x08u)
#define PSOC5_SCB_MODE_UNCONFIG            (0xFFu)

/* Define run time operation mode */
#define PSOC5_SCB_MODE_I2C_RUNTM_CFG       (PSOC5_SCB_MODE_I2C       == PSOC5_scbMode)
#define PSOC5_SCB_MODE_SPI_RUNTM_CFG       (PSOC5_SCB_MODE_SPI       == PSOC5_scbMode)
#define PSOC5_SCB_MODE_UART_RUNTM_CFG      (PSOC5_SCB_MODE_UART      == PSOC5_scbMode)
#define PSOC5_SCB_MODE_EZI2C_RUNTM_CFG     (PSOC5_SCB_MODE_EZI2C     == PSOC5_scbMode)
#define PSOC5_SCB_MODE_UNCONFIG_RUNTM_CFG  (PSOC5_SCB_MODE_UNCONFIG  == PSOC5_scbMode)

/* Condition compilation depends on operation mode: unconfigured implies apply to all modes */
#define PSOC5_SCB_MODE_I2C_CONST_CFG       (PSOC5_SCB_MODE_I2C       == PSOC5_SCB_MODE)
#define PSOC5_SCB_MODE_SPI_CONST_CFG       (PSOC5_SCB_MODE_SPI       == PSOC5_SCB_MODE)
#define PSOC5_SCB_MODE_UART_CONST_CFG      (PSOC5_SCB_MODE_UART      == PSOC5_SCB_MODE)
#define PSOC5_SCB_MODE_EZI2C_CONST_CFG     (PSOC5_SCB_MODE_EZI2C     == PSOC5_SCB_MODE)
#define PSOC5_SCB_MODE_UNCONFIG_CONST_CFG  (PSOC5_SCB_MODE_UNCONFIG  == PSOC5_SCB_MODE)

/* Condition compilation for includes */
#define PSOC5_SCB_MODE_I2C_INC       (0u !=(PSOC5_SCB_MODE_I2C       & PSOC5_SCB_MODE))
#define PSOC5_SCB_MODE_SPI_INC       (0u !=(PSOC5_SCB_MODE_SPI       & PSOC5_SCB_MODE))
#define PSOC5_SCB_MODE_UART_INC      (0u !=(PSOC5_SCB_MODE_UART      & PSOC5_SCB_MODE))
#define PSOC5_SCB_MODE_EZI2C_INC     (0u !=(PSOC5_SCB_MODE_EZI2C     & PSOC5_SCB_MODE))

/* Interrupts remove options */
#define PSOC5_REMOVE_SCB_IRQ             (1u)
#define PSOC5_SCB_IRQ_INTERNAL           (0u == PSOC5_REMOVE_SCB_IRQ)

#define PSOC5_REMOVE_UART_RX_WAKEUP_IRQ  (1u)
#define PSOC5_UART_RX_WAKEUP_IRQ         (0u == PSOC5_REMOVE_UART_RX_WAKEUP_IRQ)

/* SCB interrupt enum */
#define PSOC5_SCB_INTR_MODE_NONE     (0u)
#define PSOC5_SCB_INTR_MODE_INTERNAL (1u)
#define PSOC5_SCB_INTR_MODE_EXTERNAL (2u)

/* Bootloader communication interface enable */
#define PSOC5_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))


/***************************************
*       Includes
****************************************/

#include "PSOC5_PINS.h"

#if(PSOC5_SCB_IRQ_INTERNAL)
    #include "PSOC5_SCB_IRQ.h"
#endif /* (PSOC5_SCB_IRQ_INTERNAL) */

#if(PSOC5_UART_RX_WAKEUP_IRQ)
    #include "PSOC5_RX_WAKEUP_IRQ.h"
#endif /* (PSOC5_UART_RX_WAKEUP_IRQ) */


/***************************************
*       Type Definitions
***************************************/

typedef struct
{
    uint8 enableState;
} PSOC5_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/* Start and Stop APIs */
void PSOC5_Init(void);
void PSOC5_Enable(void);
void PSOC5_Start(void);
void PSOC5_Stop(void);

/* Sleep and Wakeup APis */
void PSOC5_Sleep(void);
void PSOC5_Wakeup(void);

/* Customer interrupt handler */
void PSOC5_SetCustomInterruptHandler(cyisraddress func);

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (PSOC5_BTLDR_COMM_ENABLED)

    /* Bootloader Physical layer functions */
    void PSOC5_CyBtldrCommStart(void);
    void PSOC5_CyBtldrCommStop (void);
    void PSOC5_CyBtldrCommReset(void);
    cystatus PSOC5_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus PSOC5_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    #if(CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5)
        #define CyBtldrCommStart    PSOC5_CyBtldrCommStart
        #define CyBtldrCommStop     PSOC5_CyBtldrCommStop
        #define CyBtldrCommReset    PSOC5_CyBtldrCommReset
        #define CyBtldrCommWrite    PSOC5_CyBtldrCommWrite
        #define CyBtldrCommRead     PSOC5_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5) */

#endif /*defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_PSOC5) || \
                                                     (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */

/* Interface to internal interrupt component */
#if(PSOC5_SCB_IRQ_INTERNAL)
    #define PSOC5_EnableInt()    PSOC5_SCB_IRQ_Enable()
    #define PSOC5_DisableInt()   PSOC5_SCB_IRQ_Disable()
#endif /* (PSOC5_SCB_IRQ_INTERNAL) */

/* Get interrupt cause */
#define PSOC5_GetInterruptCause()    (PSOC5_INTR_CAUSE_REG)

/* APIs to service INTR_RX register */
#define PSOC5_SetRxInterruptMode(interruptMask)     PSOC5_WRITE_INTR_RX_MASK(interruptMask)
#define PSOC5_ClearRxInterruptSource(interruptMask) PSOC5_CLEAR_INTR_RX(interruptMask)
#define PSOC5_SetRxInterrupt(interruptMask)         PSOC5_SET_INTR_RX(interruptMask)
#define PSOC5_GetRxInterruptSource()                (PSOC5_INTR_RX_REG)
#define PSOC5_GetRxInterruptMode()                  (PSOC5_INTR_RX_MASK_REG)
#define PSOC5_GetRxInterruptSourceMasked()          (PSOC5_INTR_RX_MASKED_REG)

/* APIs to service INTR_TX register */
#define PSOC5_SetTxInterruptMode(interruptMask)     PSOC5_WRITE_INTR_TX_MASK(interruptMask)
#define PSOC5_ClearTxInterruptSource(interruptMask) PSOC5_CLEAR_INTR_TX(interruptMask)
#define PSOC5_SetTxInterrupt(interruptMask)         PSOC5_SET_INTR_TX(interruptMask)
#define PSOC5_GetTxInterruptSource()                (PSOC5_INTR_TX_REG)
#define PSOC5_GetTxInterruptMode()                  (PSOC5_INTR_TX_MASK_REG)
#define PSOC5_GetTxInterruptSourceMasked()          (PSOC5_INTR_TX_MASKED_REG)

/* APIs to service INTR_MASTER register */
#define PSOC5_SetMasterInterruptMode(interruptMask)    PSOC5_WRITE_INTR_MASTER_MASK(interruptMask)
#define PSOC5_ClearMasterInterruptSource(interruptMask) PSOC5_CLEAR_INTR_MASTER(interruptMask)
#define PSOC5_SetMasterInterrupt(interruptMask)         PSOC5_SET_INTR_MASTER(interruptMask)
#define PSOC5_GetMasterInterruptSource()                (PSOC5_INTR_MASTER_REG)
#define PSOC5_GetMasterInterruptMode()                  (PSOC5_INTR_MASTER_MASK_REG)
#define PSOC5_GetMasterInterruptSourceMasked()          (PSOC5_INTR_MASTER_MASKED_REG)

/* APIs to service INTR_SLAVE register */
#define PSOC5_SetSlaveInterruptMode(interruptMask)     PSOC5_WRITE_INTR_SLAVE_MASK(interruptMask)
#define PSOC5_ClearSlaveInterruptSource(interruptMask) PSOC5_CLEAR_INTR_SLAVE(interruptMask)
#define PSOC5_SetSlaveInterrupt(interruptMask)         PSOC5_SET_INTR_SLAVE(interruptMask)
#define PSOC5_GetSlaveInterruptSource()                (PSOC5_INTR_SLAVE_REG)
#define PSOC5_GetSlaveInterruptMode()                  (PSOC5_INTR_SLAVE_MASK_REG)
#define PSOC5_GetSlaveInterruptSourceMasked()          (PSOC5_INTR_SLAVE_MASKED_REG)


/**********************************
*     Vars with External Linkage
**********************************/

extern uint8 PSOC5_initVar;


/***************************************
*              Registers
***************************************/

#define PSOC5_CTRL_REG               (*(reg32 *) PSOC5_SCB__CTRL)
#define PSOC5_CTRL_PTR               ( (reg32 *) PSOC5_SCB__CTRL)

#define PSOC5_STATUS_REG             (*(reg32 *) PSOC5_SCB__STATUS)
#define PSOC5_STATUS_PTR             ( (reg32 *) PSOC5_SCB__STATUS)

#define PSOC5_SPI_CTRL_REG           (*(reg32 *) PSOC5_SCB__SPI_CTRL)
#define PSOC5_SPI_CTRL_PTR           ( (reg32 *) PSOC5_SCB__SPI_CTRL)

#define PSOC5_SPI_STATUS_REG         (*(reg32 *) PSOC5_SCB__SPI_STATUS)
#define PSOC5_SPI_STATUS_PTR         ( (reg32 *) PSOC5_SCB__SPI_STATUS)

#define PSOC5_UART_CTRL_REG          (*(reg32 *) PSOC5_SCB__UART_CTRL)
#define PSOC5_UART_CTRL_PTR          ( (reg32 *) PSOC5_SCB__UART_CTRL)

#define PSOC5_UART_TX_CTRL_REG       (*(reg32 *) PSOC5_SCB__UART_TX_CTRL)
#define PSOC5_UART_TX_CTRL_PTR       ( (reg32 *) PSOC5_SCB__UART_RX_CTRL)

#define PSOC5_UART_RX_CTRL_REG       (*(reg32 *) PSOC5_SCB__UART_RX_CTRL)
#define PSOC5_UART_RX_CTRL_PTR       ( (reg32 *) PSOC5_SCB__UART_RX_CTRL)

#define PSOC5_UART_RX_STATUS_REG     (*(reg32 *) PSOC5_SCB__UART_RX_STATUS)
#define PSOC5_UART_RX_STATUS_PTR     ( (reg32 *) PSOC5_SCB__UART_RX_STATUS)

#define PSOC5_I2C_CTRL_REG           (*(reg32 *) PSOC5_SCB__I2C_CTRL)
#define PSOC5_I2C_CTRL_PTR           ( (reg32 *) PSOC5_SCB__I2C_CTRL)

#define PSOC5_I2C_STATUS_REG         (*(reg32 *) PSOC5_SCB__I2C_STATUS)
#define PSOC5_I2C_STATUS_PTR         ( (reg32 *) PSOC5_SCB__I2C_STATUS)

#define PSOC5_I2C_MASTER_CMD_REG     (*(reg32 *) PSOC5_SCB__I2C_M_CMD)
#define PSOC5_I2C_MASTER_CMD_PTR     ( (reg32 *) PSOC5_SCB__I2C_M_CMD)

#define PSOC5_I2C_SLAVE_CMD_REG      (*(reg32 *) PSOC5_SCB__I2C_S_CMD)
#define PSOC5_I2C_SLAVE_CMD_PTR      ( (reg32 *) PSOC5_SCB__I2C_S_CMD)

#define PSOC5_I2C_CFG_REG            (*(reg32 *) PSOC5_SCB__I2C_CFG)
#define PSOC5_I2C_CFG_PTR            ( (reg32 *) PSOC5_SCB__I2C_CFG)

#define PSOC5_TX_CTRL_REG            (*(reg32 *) PSOC5_SCB__TX_CTRL)
#define PSOC5_TX_CTRL_PTR            ( (reg32 *) PSOC5_SCB__TX_CTRL)

#define PSOC5_TX_FIFO_CTRL_REG       (*(reg32 *) PSOC5_SCB__TX_FIFO_CTRL)
#define PSOC5_TX_FIFO_CTRL_PTR       ( (reg32 *) PSOC5_SCB__TX_FIFO_CTRL)

#define PSOC5_TX_FIFO_STATUS_REG     (*(reg32 *) PSOC5_SCB__TX_FIFO_STATUS)
#define PSOC5_TX_FIFO_STATUS_PTR     ( (reg32 *) PSOC5_SCB__TX_FIFO_STATUS)

#define PSOC5_TX_FIFO_WR_REG         (*(reg32 *) PSOC5_SCB__TX_FIFO_WR)
#define PSOC5_TX_FIFO_WR_PTR         ( (reg32 *) PSOC5_SCB__TX_FIFO_WR)

#define PSOC5_RX_CTRL_REG            (*(reg32 *) PSOC5_SCB__RX_CTRL)
#define PSOC5_RX_CTRL_PTR            ( (reg32 *) PSOC5_SCB__RX_CTRL)

#define PSOC5_RX_FIFO_CTRL_REG       (*(reg32 *) PSOC5_SCB__RX_FIFO_CTRL)
#define PSOC5_RX_FIFO_CTRL_PTR       ( (reg32 *) PSOC5_SCB__RX_FIFO_CTRL)

#define PSOC5_RX_FIFO_STATUS_REG     (*(reg32 *) PSOC5_SCB__RX_FIFO_STATUS)
#define PSOC5_RX_FIFO_STATUS_PTR     ( (reg32 *) PSOC5_SCB__RX_FIFO_STATUS)

#define PSOC5_RX_MATCH_REG           (*(reg32 *) PSOC5_SCB__RX_MATCH)
#define PSOC5_RX_MATCH_PTR           ( (reg32 *) PSOC5_SCB__RX_MATCH)

#define PSOC5_RX_FIFO_RD_REG         (*(reg32 *) PSOC5_SCB__RX_FIFO_RD)
#define PSOC5_RX_FIFO_RD_PTR         ( (reg32 *) PSOC5_SCB__RX_FIFO_RD)

#define PSOC5_RX_FIFO_RD_SILENT_REG  (*(reg32 *) PSOC5_SCB__RX_FIFO_RD_SILENT)
#define PSOC5_RX_FIFO_RD_SILENT_PTR  ( (reg32 *) PSOC5_SCB__RX_FIFO_RD_SILENT)

#define PSOC5_EZBUF_DATA00_REG       (*(reg32 *) PSOC5_SCB__EZ_DATA00)
#define PSOC5_EZBUF_DATA00_PTR       ( (reg32 *) PSOC5_SCB__EZ_DATA00)

#define PSOC5_INTR_CAUSE_REG         (*(reg32 *) PSOC5_SCB__INTR_CAUSE)
#define PSOC5_INTR_CAUSE_PTR         ( (reg32 *) PSOC5_SCB__INTR_CAUSE)

#define PSOC5_INTR_I2C_EC_REG        (*(reg32 *) PSOC5_SCB__INTR_I2C_EC)
#define PSOC5_INTR_I2C_EC_PTR        ( (reg32 *) PSOC5_SCB__INTR_I2C_EC)

#define PSOC5_INTR_I2C_EC_MASK_REG   (*(reg32 *) PSOC5_SCB__INTR_I2C_EC_MASK)
#define PSOC5_INTR_I2C_EC_MASK_PTR   ( (reg32 *) PSOC5_SCB__INTR_I2C_EC_MASK)

#define PSOC5_INTR_I2C_EC_MASKED_REG (*(reg32 *) PSOC5_SCB__INTR_I2C_EC_MASKED)
#define PSOC5_INTR_I2C_EC_MASKED_PTR ( (reg32 *) PSOC5_SCB__INTR_I2C_EC_MASKED)

#define PSOC5_INTR_SPI_EC_REG        (*(reg32 *) PSOC5_SCB__INTR_SPI_EC)
#define PSOC5_INTR_SPI_EC_PTR        ( (reg32 *) PSOC5_SCB__INTR_SPI_EC)

#define PSOC5_INTR_SPI_EC_MASK_REG   (*(reg32 *) PSOC5_SCB__INTR_SPI_EC_MASK)
#define PSOC5_INTR_SPI_EC_MASK_PTR   ( (reg32 *) PSOC5_SCB__INTR_SPI_EC_MASK)

#define PSOC5_INTR_SPI_EC_MASKED_REG (*(reg32 *) PSOC5_SCB__INTR_SPI_EC_MASKED)
#define PSOC5_INTR_SPI_EC_MASKED_PTR ( (reg32 *) PSOC5_SCB__INTR_SPI_EC_MASKED)

#define PSOC5_INTR_MASTER_REG        (*(reg32 *) PSOC5_SCB__INTR_M)
#define PSOC5_INTR_MASTER_PTR        ( (reg32 *) PSOC5_SCB__INTR_M)

#define PSOC5_INTR_MASTER_SET_REG    (*(reg32 *) PSOC5_SCB__INTR_M_SET)
#define PSOC5_INTR_MASTER_SET_PTR    ( (reg32 *) PSOC5_SCB__INTR_M_SET)

#define PSOC5_INTR_MASTER_MASK_REG   (*(reg32 *) PSOC5_SCB__INTR_M_MASK)
#define PSOC5_INTR_MASTER_MASK_PTR   ( (reg32 *) PSOC5_SCB__INTR_M_MASK)

#define PSOC5_INTR_MASTER_MASKED_REG (*(reg32 *) PSOC5_SCB__INTR_M_MASKED)
#define PSOC5_INTR_MASTER_MASKED_PTR ( (reg32 *) PSOC5_SCB__INTR_M_MASKED)

#define PSOC5_INTR_SLAVE_REG         (*(reg32 *) PSOC5_SCB__INTR_S)
#define PSOC5_INTR_SLAVE_PTR         ( (reg32 *) PSOC5_SCB__INTR_S)

#define PSOC5_INTR_SLAVE_SET_REG     (*(reg32 *) PSOC5_SCB__INTR_S_SET)
#define PSOC5_INTR_SLAVE_SET_PTR     ( (reg32 *) PSOC5_SCB__INTR_S_SET)

#define PSOC5_INTR_SLAVE_MASK_REG    (*(reg32 *) PSOC5_SCB__INTR_S_MASK)
#define PSOC5_INTR_SLAVE_MASK_PTR    ( (reg32 *) PSOC5_SCB__INTR_S_MASK)

#define PSOC5_INTR_SLAVE_MASKED_REG  (*(reg32 *) PSOC5_SCB__INTR_S_MASKED)
#define PSOC5_INTR_SLAVE_MASKED_PTR  ( (reg32 *) PSOC5_SCB__INTR_S_MASKED)

#define PSOC5_INTR_TX_REG            (*(reg32 *) PSOC5_SCB__INTR_TX)
#define PSOC5_INTR_TX_PTR            ( (reg32 *) PSOC5_SCB__INTR_TX)

#define PSOC5_INTR_TX_SET_REG        (*(reg32 *) PSOC5_SCB__INTR_TX_SET)
#define PSOC5_INTR_TX_SET_PTR        ( (reg32 *) PSOC5_SCB__INTR_TX_SET)

#define PSOC5_INTR_TX_MASK_REG       (*(reg32 *) PSOC5_SCB__INTR_TX_MASK)
#define PSOC5_INTR_TX_MASK_PTR       ( (reg32 *) PSOC5_SCB__INTR_TX_MASK)

#define PSOC5_INTR_TX_MASKED_REG     (*(reg32 *) PSOC5_SCB__INTR_TX_MASKED)
#define PSOC5_INTR_TX_MASKED_PTR     ( (reg32 *) PSOC5_SCB__INTR_TX_MASKED)

#define PSOC5_INTR_RX_REG            (*(reg32 *) PSOC5_SCB__INTR_RX)
#define PSOC5_INTR_RX_PTR            ( (reg32 *) PSOC5_SCB__INTR_RX)

#define PSOC5_INTR_RX_SET_REG        (*(reg32 *) PSOC5_SCB__INTR_RX_SET)
#define PSOC5_INTR_RX_SET_PTR        ( (reg32 *) PSOC5_SCB__INTR_RX_SET)

#define PSOC5_INTR_RX_MASK_REG       (*(reg32 *) PSOC5_SCB__INTR_RX_MASK)
#define PSOC5_INTR_RX_MASK_PTR       ( (reg32 *) PSOC5_SCB__INTR_RX_MASK)

#define PSOC5_INTR_RX_MASKED_REG     (*(reg32 *) PSOC5_SCB__INTR_RX_MASKED)
#define PSOC5_INTR_RX_MASKED_PTR     ( (reg32 *) PSOC5_SCB__INTR_RX_MASKED)


/***************************************
*        Registers Constants
***************************************/

/* PSOC5_CTRL */
#define PSOC5_CTRL_OVS_POS           (0u)  /* [3:0]   Oversampling factor                 */
#define PSOC5_CTRL_EC_AM_MODE_POS    (8u)  /* [8]     Externally clocked address match    */
#define PSOC5_CTRL_EC_OP_MODE_POS    (9u)  /* [9]     Externally clocked operation mode   */
#define PSOC5_CTRL_EZBUF_MODE_POS    (10u) /* [10]    EZ buffer is enabled                */
#define PSOC5_CTRL_ADDR_ACCEPT_POS   (16u) /* [16]    Put matched address in RX FIFO      */
#define PSOC5_CTRL_BLOCK_POS         (17u) /* [17]    Ext and Int logic to resolve colide */
#define PSOC5_CTRL_MODE_POS          (24u) /* [25:24] Operation mode                      */
#define PSOC5_CTRL_ENABLED_POS       (31u) /* [31]    Enable SCB block                    */
#define PSOC5_CTRL_OVS_MASK          ((uint32) 0x0Fu)
#define PSOC5_CTRL_EC_AM_MODE        ((uint32) ((uint32) 0x01u << PSOC5_CTRL_EC_AM_MODE_POS))
#define PSOC5_CTRL_EC_OP_MODE        ((uint32) ((uint32) 0x01u << PSOC5_CTRL_EC_OP_MODE_POS))
#define PSOC5_CTRL_EZBUF_MODE        ((uint32) ((uint32) 0x01u << PSOC5_CTRL_EZBUF_MODE_POS))
#define PSOC5_CTRL_ADDR_ACCEPT       ((uint32) ((uint32) 0x01u << PSOC5_CTRL_ADDR_ACCEPT_POS))
#define PSOC5_CTRL_BLOCK             ((uint32) ((uint32) 0x01u << PSOC5_CTRL_BLOCK_POS))
#define PSOC5_CTRL_MODE_MASK         ((uint32) ((uint32) 0x03u << PSOC5_CTRL_MODE_POS))
#define PSOC5_CTRL_MODE_I2C          ((uint32)  0x00u)
#define PSOC5_CTRL_MODE_SPI          ((uint32) ((uint32) 0x01u << PSOC5_CTRL_MODE_POS))
#define PSOC5_CTRL_MODE_UART         ((uint32) ((uint32) 0x02u << PSOC5_CTRL_MODE_POS))
#define PSOC5_CTRL_ENABLED           ((uint32) ((uint32) 0x01u << PSOC5_CTRL_ENABLED_POS))


/* PSOC5_STATUS_REG */
#define PSOC5_STATUS_EC_BUSY_POS     (0u)  /* [0] Bus busy. Externaly clocked loigc access to EZ memory */
#define PSOC5_STATUS_EC_BUSY         ((uint32) 0x0Fu)


/* PSOC5_SPI_CTRL_REG  */
#define PSOC5_SPI_CTRL_CONTINUOUS_POS        (0u)  /* [0]     Continuous or Separated SPI data transfers */
#define PSOC5_SPI_CTRL_SELECT_PRECEDE_POS    (1u)  /* [1]     Precedes or coincides start of data frame  */
#define PSOC5_SPI_CTRL_CPHA_POS              (2u)  /* [2]     SCLK phase                                 */
#define PSOC5_SPI_CTRL_CPOL_POS              (3u)  /* [3]     SCLK polarity                              */
#define PSOC5_SPI_CTRL_LATE_MISO_SAMPLE_POS  (4u)  /* [4]     Late MISO sample enabled                   */
#define PSOC5_SPI_CTRL_LOOPBACK_POS          (16u) /* [16]    Local loopback control enabled             */
#define PSOC5_SPI_CTRL_MODE_POS              (24u) /* [25:24] Submode of SPI operation                   */
#define PSOC5_SPI_CTRL_SLAVE_SELECT_POS      (26u) /* [27:26] Selects SPI SS signal                      */
#define PSOC5_SPI_CTRL_MASTER_MODE_POS       (31u) /* [31]    Master mode enabled                        */
#define PSOC5_SPI_CTRL_CONTINUOUS            ((uint32) 0x01u)
#define PSOC5_SPI_CTRL_SELECT_PRECEDE        ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_SPI_CTRL_SELECT_PRECEDE_POS))
#define PSOC5_SPI_CTRL_SCLK_MODE_MASK        ((uint32) ((uint32) 0x03u << \
                                                                    PSOC5_SPI_CTRL_CPHA_POS))
#define PSOC5_SPI_CTRL_CPHA                  ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_SPI_CTRL_CPHA_POS))
#define PSOC5_SPI_CTRL_CPOL                  ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_SPI_CTRL_CPOL_POS))
#define PSOC5_SPI_CTRL_LATE_MISO_SAMPLE      ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_SPI_CTRL_LATE_MISO_SAMPLE_POS))
#define PSOC5_SPI_CTRL_LOOPBACK              ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_SPI_CTRL_LOOPBACK_POS))
#define PSOC5_SPI_CTRL_MODE_MASK             ((uint32) ((uint32) 0x03u << \
                                                                    PSOC5_SPI_CTRL_MODE_POS))
#define PSOC5_SPI_CTRL_MODE_MOTOROLA         ((uint32) 0x00u)
#define PSOC5_SPI_CTRL_MODE_TI               ((uint32) ((uint32) 0x01u << PSOC5_CTRL_MODE_POS))
#define PSOC5_SPI_CTRL_MODE_NS               ((uint32) ((uint32) 0x02u << PSOC5_CTRL_MODE_POS))
#define PSOC5_SPI_CTRL_SLAVE_SELECT_MASK     ((uint32) ((uint32) 0x03u << \
                                                                    PSOC5_SPI_CTRL_SLAVE_SELECT_POS))
#define PSOC5_SPI_CTRL_SLAVE_SELECT0         ((uint32) 0x00u)
#define PSOC5_SPI_CTRL_SLAVE_SELECT1         ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_SPI_CTRL_SLAVE_SELECT_POS))
#define PSOC5_SPI_CTRL_SLAVE_SELECT2         ((uint32) ((uint32) 0x02u << \
                                                                    PSOC5_SPI_CTRL_SLAVE_SELECT_POS))
#define PSOC5_SPI_CTRL_SLAVE_SELECT3         ((uint32) ((uint32) 0x03u << \
                                                                    PSOC5_SPI_CTRL_SLAVE_SELECT_POS))
#define PSOC5_SPI_CTRL_MASTER                ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_SPI_CTRL_MASTER_MODE_POS))
#define PSOC5_SPI_CTRL_SLAVE                 ((uint32) 0x00u)


/* PSOC5_SPI_STATUS_REG  */
#define PSOC5_SPI_STATUS_BUS_BUSY_POS    (0u)  /* [0]    Bus busy - slave selected */
#define PSOC5_SPI_STATUS_EZBUF_ADDR_POS  (8u)  /* [15:8] EzAddress                 */
#define PSOC5_SPI_STATUS_BUS_BUSY        ((uint32) 0x01u)
#define PSOC5_SPI_STATUS_EZBUF_ADDR_MASK    ((uint32) ((uint32) 0xFFu << \
                                                                    PSOC5_I2C_STATUS_EZBUF_ADDR_POS))


/* PSOC5_UART_CTRL */
#define PSOC5_UART_CTRL_LOOPBACK_POS         (16u) /* [16] Loopback     */
#define PSOC5_UART_CTRL_MODE_POS             (24u) /* [24] UART subMode */
#define PSOC5_UART_CTRL_LOOPBACK             ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_UART_CTRL_LOOPBACK_POS))
#define PSOC5_UART_CTRL_MODE_UART_STD        ((uint32) 0x00u)
#define PSOC5_UART_CTRL_MODE_UART_SMARTCARD  ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_UART_CTRL_MODE_POS))
#define PSOC5_UART_CTRL_MODE_UART_IRDA       ((uint32) ((uint32) 0x02u << \
                                                                        PSOC5_UART_CTRL_MODE_POS))
#define PSOC5_UART_CTRL_MODE_MASK            ((uint32) ((uint32) 0x03u << \
                                                                        PSOC5_UART_CTRL_MODE_POS))


/* PSOC5_UART_TX_CTRL */
#define PSOC5_UART_TX_CTRL_STOP_BITS_POS         (0u)  /* [2:0] Stop bits: (Stop bits + 1) * 0.5 period */
#define PSOC5_UART_TX_CTRL_PARITY_POS            (4u)  /* [4]   Parity bit                              */
#define PSOC5_UART_TX_CTRL_PARITY_ENABLED_POS    (5u)  /* [5]   Parity enable                           */
#define PSOC5_UART_TX_CTRL_RETRY_ON_NACK_POS     (8u)  /* [8]   Smart Card: re-send frame on NACK       */
#define PSOC5_UART_TX_CTRL_ONE_STOP_BIT          ((uint32) 0x01u)
#define PSOC5_UART_TX_CTRL_ONE_HALF_STOP_BITS    ((uint32) 0x02u)
#define PSOC5_UART_TX_CTRL_TWO_STOP_BITS         ((uint32) 0x03u)
#define PSOC5_UART_TX_CTRL_STOP_BITS_MASK        ((uint32) 0x07u)
#define PSOC5_UART_TX_CTRL_PARITY                ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_TX_CTRL_PARITY_POS))
#define PSOC5_UART_TX_CTRL_PARITY_ENABLED        ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_TX_CTRL_PARITY_ENABLED_POS))
#define PSOC5_UART_TX_CTRL_RETRY_ON_NACK         ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_TX_CTRL_RETRY_ON_NACK_POS))


/* PSOC5_UART_RX_CTRL */
#define PSOC5_UART_RX_CTRL_STOP_BITS_POS             (0u)  /* [2:0] Stop bits: (Stop bits + 1) * 0.5 prd   */
#define PSOC5_UART_RX_CTRL_PARITY_POS                (4u)  /* [4]   Parity bit                             */
#define PSOC5_UART_RX_CTRL_PARITY_ENABLED_POS        (5u)  /* [5]   Parity enable                          */
#define PSOC5_UART_RX_CTRL_POLARITY_POS              (6u)  /* [6]   IrDA: inverts polarity of RX signal    */
#define PSOC5_UART_RX_CTRL_DROP_ON_PARITY_ERR_POS    (8u)  /* [8]   Drop and lost RX FIFO on parity error  */
#define PSOC5_UART_RX_CTRL_DROP_ON_FRAME_ERR_POS     (9u)  /* [9]   Drop and lost RX FIFO on frame error   */
#define PSOC5_UART_RX_CTRL_MP_MODE_POS               (10u) /* [10]  Multi-processor mode                   */
#define PSOC5_UART_RX_CTRL_LIN_MODE_POS              (12u) /* [12]  Lin mode: applicable for UART Standart */
#define PSOC5_UART_RX_CTRL_SKIP_START_POS            (13u) /* [13]  Skip start not: only for UART Standart */
#define PSOC5_UART_RX_CTRL_BREAK_WIDTH_POS           (16u) /* [19:16]  Break width: (Break width + 1)      */
#define PSOC5_UART_TX_CTRL_ONE_STOP_BIT              ((uint32) 0x01u)
#define PSOC5_UART_TX_CTRL_ONE_HALF_STOP_BITS        ((uint32) 0x02u)
#define PSOC5_UART_TX_CTRL_TWO_STOP_BITS             ((uint32) 0x03u)
#define PSOC5_UART_RX_CTRL_STOP_BITS_MASK            ((uint32) 0x07u)
#define PSOC5_UART_RX_CTRL_PARITY                    ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_RX_CTRL_PARITY_POS))
#define PSOC5_UART_RX_CTRL_PARITY_ENABLED            ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_RX_CTRL_PARITY_ENABLED_POS))
#define PSOC5_UART_RX_CTRL_POLARITY                  ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_RX_CTRL_POLARITY_POS))
#define PSOC5_UART_RX_CTRL_DROP_ON_PARITY_ERR        ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_UART_RX_CTRL_DROP_ON_PARITY_ERR_POS))
#define PSOC5_UART_RX_CTRL_DROP_ON_FRAME_ERR         ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_UART_RX_CTRL_DROP_ON_FRAME_ERR_POS))
#define PSOC5_UART_RX_CTRL_MP_MODE                   ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_RX_CTRL_MP_MODE_POS))
#define PSOC5_UART_RX_CTRL_LIN_MODE                  ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_UART_RX_CTRL_LIN_MODE_POS))
#define PSOC5_UART_RX_CTRL_SKIP_START                ((uint32) ((uint32) 0x01u << \
                                                                  PSOC5_UART_RX_CTRL_SKIP_START_POS))
#define PSOC5_UART_RX_CTRL_BREAK_WIDTH_MASK          ((uint32) ((uint32) 0x0Fu << \
                                                                  PSOC5_UART_RX_CTRL_BREAK_WIDTH_POS))

/* PSOC5_UART_RX_STATUS_REG */
#define PSOC5_UART_RX_STATUS_BR_COUNTER_POS     (0u)  /* [11:0] Baute Rate counter */
#define PSOC5_UART_RX_STATUS_BR_COUNTER_MASK    ((uint32) 0xFFFu)


/* PSOC5_I2C_CTRL */
#define PSOC5_I2C_CTRL_HIGH_PHASE_OVS_POS           (0u)   /* [3:0] Oversampling factor high: masrer only */
#define PSOC5_I2C_CTRL_LOW_PHASE_OVS_POS            (4u)   /* [7:4] Oversampling factor low:  masrer only */
#define PSOC5_I2C_CTRL_M_READY_DATA_ACK_POS         (8u)   /* [8]   Master ACKs data wgile RX FIFO != FULL*/
#define PSOC5_I2C_CTRL_M_NOT_READY_DATA_NACK_POS    (9u)   /* [9]   Master NACKs data if RX FIFO ==  FULL */
#define PSOC5_I2C_CTRL_S_GENERAL_IGNORE_POS         (11u)  /* [11]  Slave ignores General call            */
#define PSOC5_I2C_CTRL_S_READY_ADDR_ACK_POS         (12u)  /* [12]  Slave ACKs Address if RX FIFO != FULL */
#define PSOC5_I2C_CTRL_S_READY_DATA_ACK_POS         (13u)  /* [13]  Slave ACKs data while RX FIFO == FULL */
#define PSOC5_I2C_CTRL_S_NOT_READY_ADDR_NACK_POS    (14u)  /* [14]  Slave NACKs address if RX FIFO == FULL*/
#define PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK_POS    (15u)  /* [15]  Slave NACKs data if RX FIFO is  FULL  */
#define PSOC5_I2C_CTRL_LOOPBACK_POS                 (16u)  /* [16]  Loopback                              */
#define PSOC5_I2C_CTRL_SLAVE_MODE_POS               (30u)  /* [30]  Slave mode enabled                    */
#define PSOC5_I2C_CTRL_MASTER_MODE_POS              (31u)  /* [31]  Master mode enabled                   */
#define PSOC5_I2C_CTRL_HIGH_PHASE_OVS_MASK  ((uint32) 0x0Fu)
#define PSOC5_I2C_CTRL_LOW_PHASE_OVS_MASK   ((uint32) ((uint32) 0x0Fu << \
                                                                PSOC5_I2C_CTRL_LOW_PHASE_OVS_POS))
#define PSOC5_I2C_CTRL_M_READY_DATA_ACK      ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_M_READY_DATA_ACK_POS))
#define PSOC5_I2C_CTRL_M_NOT_READY_DATA_NACK ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_M_NOT_READY_DATA_NACK_POS))
#define PSOC5_I2C_CTRL_S_GENERAL_IGNORE      ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_S_GENERAL_IGNORE_POS))
#define PSOC5_I2C_CTRL_S_READY_ADDR_ACK      ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_S_READY_ADDR_ACK_POS))
#define PSOC5_I2C_CTRL_S_READY_DATA_ACK      ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_S_READY_DATA_ACK_POS))
#define PSOC5_I2C_CTRL_S_NOT_READY_ADDR_NACK ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_S_NOT_READY_ADDR_NACK_POS))
#define PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK_POS))
#define PSOC5_I2C_CTRL_LOOPBACK              ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_LOOPBACK_POS))
#define PSOC5_I2C_CTRL_SLAVE_MODE            ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_SLAVE_MODE_POS))
#define PSOC5_I2C_CTRL_MASTER_MODE           ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CTRL_MASTER_MODE_POS))
#define PSOC5_I2C_CTRL_SLAVE_MASTER_MODE_MASK    ((uint32) ((uint32) 0x03u << \
                                                                PSOC5_I2C_CTRL_SLAVE_MODE_POS))


/* PSOC5_I2C_STATUS_REG  */
#define PSOC5_I2C_STATUS_BUS_BUSY_POS    (0u)  /* [0]    Bus busy: internally clocked */
#define PSOC5_I2C_STATUS_S_READ_POS      (4u)  /* [4]    Slave is read by master      */
#define PSOC5_I2C_STATUS_M_READ_POS      (5u)  /* [5]    Master reads Slave           */
#define PSOC5_I2C_STATUS_EZBUF_ADDR_POS  (8u)  /* [15:8] EZAddress                    */
#define PSOC5_I2C_STATUS_BUS_BUSY        ((uint32) 0x01u)
#define PSOC5_I2C_STATUS_S_READ          ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_I2C_STATUS_S_READ_POS))
#define PSOC5_I2C_STATUS_M_READ          ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_I2C_STATUS_M_READ_POS))
#define PSOC5_I2C_STATUS_EZBUF_ADDR_MASK ((uint32) ((uint32) 0xFFu << \
                                                                    PSOC5_I2C_STATUS_EZBUF_ADDR_POS))


/* PSOC5_I2C_MASTER_CMD_REG */
#define PSOC5_I2C_MASTER_CMD_M_START_POS             (0u)  /* [0] Master generate Start                */
#define PSOC5_I2C_MASTER_CMD_M_START_ON_IDLE_POS     (1u)  /* [1] Master generate Start if bus is free */
#define PSOC5_I2C_MASTER_CMD_M_ACK_POS               (2u)  /* [2] Master generate ACK                  */
#define PSOC5_I2C_MASTER_CMD_M_NACK_POS              (3u)  /* [3] Master generate NACK                 */
#define PSOC5_I2C_MASTER_CMD_M_STOP_POS              (4u)  /* [4] Master generate Stop                 */
#define PSOC5_I2C_MASTER_CMD_M_START         ((uint32) 0x01u)
#define PSOC5_I2C_MASTER_CMD_M_START_ON_IDLE ((uint32) ((uint32) 0x01u << \
                                                                   PSOC5_I2C_MASTER_CMD_M_START_ON_IDLE_POS))
#define PSOC5_I2C_MASTER_CMD_M_ACK           ((uint32) ((uint32) 0x01u << \
                                                                   PSOC5_I2C_MASTER_CMD_M_ACK_POS))
#define PSOC5_I2C_MASTER_CMD_M_NACK          ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_I2C_MASTER_CMD_M_NACK_POS))
#define PSOC5_I2C_MASTER_CMD_M_STOP          ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_I2C_MASTER_CMD_M_STOP_POS))


/* PSOC5_I2C_SLAVE_CMD_REG  */
#define PSOC5_I2C_SLAVE_CMD_S_ACK_POS    (0u)  /* [0] Slave generate ACK  */
#define PSOC5_I2C_SLAVE_CMD_S_NACK_POS   (1u)  /* [1] Slave generate NACK */
#define PSOC5_I2C_SLAVE_CMD_S_ACK        ((uint32) 0x01u)
#define PSOC5_I2C_SLAVE_CMD_S_NACK       ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_SLAVE_CMD_S_NACK_POS))

#define PSOC5_I2C_SLAVE_CMD_S_ACK_POS    (0u)  /* [0] Slave generate ACK  */
#define PSOC5_I2C_SLAVE_CMD_S_NACK_POS   (1u)  /* [1] Slave generate NACK */
#define PSOC5_I2C_SLAVE_CMD_S_ACK        ((uint32) 0x01u)
#define PSOC5_I2C_SLAVE_CMD_S_NACK       ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_SLAVE_CMD_S_NACK_POS))
/* PSOC5_I2C_CFG  */
#define PSOC5_I2C_CFG_SDA_FILT_HYS_POS           (0u)  /* [1:0]   Trim bits for the I2C SDA filter         */
#define PSOC5_I2C_CFG_SDA_FILT_TRIM_POS          (2u)  /* [3:2]   Trim bits for the I2C SDA filter         */
#define PSOC5_I2C_CFG_SCL_FILT_HYS_POS           (4u)  /* [5:4]   Trim bits for the I2C SCL filter         */
#define PSOC5_I2C_CFG_SCL_FILT_TRIM_POS          (6u)  /* [7:6]   Trim bits for the I2C SCL filter         */
#define PSOC5_I2C_CFG_SDA_FILT_OUT_HYS_POS       (8u)  /* [9:8]   Trim bits for I2C SDA filter output path */
#define PSOC5_I2C_CFG_SDA_FILT_OUT_TRIM_POS      (10u) /* [11:10] Trim bits for I2C SDA filter output path */
#define PSOC5_I2C_CFG_SDA_FILT_HS_POS            (16u) /* [16]    '0': 50 ns filter, '1': 10 ns filter     */
#define PSOC5_I2C_CFG_SDA_FILT_ENABLED_POS       (17u) /* [17]    I2C SDA filter enabled                   */
#define PSOC5_I2C_CFG_SCL_FILT_HS_POS            (24u) /* [24]    '0': 50 ns filter, '1': 10 ns filter     */
#define PSOC5_I2C_CFG_SCL_FILT_ENABLED_POS       (25u) /* [25]    I2C SCL filter enabled                   */
#define PSOC5_I2C_CFG_SDA_FILT_OUT_HS_POS        (26u) /* [26]    '0': 50ns filter, '1': 10 ns filter      */
#define PSOC5_I2C_CFG_SDA_FILT_OUT_ENABLED_POS   (27u) /* [27]    I2C SDA output delay filter enabled      */
#define PSOC5_I2C_CFG_SDA_FILT_HYS_MASK          ((uint32) 0x00u)
#define PSOC5_I2C_CFG_SDA_FILT_TRIM_MASK         ((uint32) ((uint32) 0x03u << \
                                                                PSOC5_I2C_CFG_SDA_FILT_TRIM_POS))
#define PSOC5_I2C_CFG_SCL_FILT_HYS_MASK          ((uint32) ((uint32) 0x03u << \
                                                                PSOC5_I2C_CFG_SCL_FILT_HYS_POS))
#define PSOC5_I2C_CFG_SCL_FILT_TRIM_MASK         ((uint32) ((uint32) 0x03u << \
                                                                PSOC5_I2C_CFG_SCL_FILT_TRIM_POS))
#define PSOC5_I2C_CFG_SDA_FILT_OUT_HYS_MASK      ((uint32) ((uint32) 0x03u << \
                                                                PSOC5_I2C_CFG_SDA_FILT_OUT_HYS_POS))
#define PSOC5_I2C_CFG_SDA_FILT_OUT_TRIM_MASK     ((uint32) ((uint32) 0x03u << \
                                                                PSOC5_I2C_CFG_SDA_FILT_OUT_TRIM_POS))
#define PSOC5_I2C_CFG_SDA_FILT_HS                ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CFG_SDA_FILT_HS_POS))
#define PSOC5_I2C_CFG_SDA_FILT_ENABLED           ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CFG_SDA_FILT_ENABLED_POS))
#define PSOC5_I2C_CFG_SCL_FILT_HS                ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CFG_SCL_FILT_HS_POS))
#define PSOC5_I2C_CFG_SCL_FILT_ENABLED           ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CFG_SCL_FILT_ENABLED_POS))
#define PSOC5_I2C_CFG_SDA_FILT_OUT_HS            ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CFG_SDA_FILT_OUT_HS_POS))
#define PSOC5_I2C_CFG_SDA_FILT_OUT_ENABLED       ((uint32) ((uint32) 0x01u << \
                                                                PSOC5_I2C_CFG_SDA_FILT_OUT_ENABLED_POS))


/* PSOC5_TX_CTRL_REG */
#define PSOC5_TX_CTRL_DATA_WIDTH_POS     (0u)  /* [3:0] Dataframe width: (Data width - 1) */
#define PSOC5_TX_CTRL_MSB_FIRST_POS      (8u)  /* [8]   MSB first shifter-out             */
#define PSOC5_TX_CTRL_ENABLED_POS        (31u) /* [31]  Transmitter enabled               */
#define PSOC5_TX_CTRL_DATA_WIDTH_MASK    ((uint32) 0x0Fu)
#define PSOC5_TX_CTRL_MSB_FIRST          ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_TX_CTRL_MSB_FIRST_POS))
#define PSOC5_TX_CTRL_LSB_FIRST          ((uint32) 0x00u)
#define PSOC5_TX_CTRL_ENABLED            ((uint32) ((uint32) 0x01u << PSOC5_TX_CTRL_ENABLED_POS))


/* PSOC5_TX_CTRL_FIFO_REG */
#define PSOC5_TX_FIFO_CTRL_TRIGGER_LEVEL_POS     (0u)  /* [2:0] Trigger level                              */
#define PSOC5_TX_FIFO_CTRL_CLEAR_POS             (16u) /* [16]  Clear TX FIFO: claared after set           */
#define PSOC5_TX_FIFO_CTRL_FREEZE_POS            (17u) /* [17]  Freeze TX FIFO: HW do not inc read pointer */
#define PSOC5_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK    ((uint32) 0x07u)
#define PSOC5_TX_FIFO_CTRL_CLEAR                 ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_TX_FIFO_CTRL_CLEAR_POS))
#define PSOC5_TX_FIFO_CTRL_FREEZE                ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_TX_FIFO_CTRL_FREEZE_POS))


/* PSOC5_TX_FIFO_STATUS_REG */
#define PSOC5_TX_FIFO_STATUS_USED_POS    (0u)  /* [3:0]   Amount of entries in TX FIFO */
#define PSOC5_TX_FIFO_SR_VALID_POS       (15u) /* [15]    Shifter status of TX FIFO    */
#define PSOC5_TX_FIFO_STATUS_RD_PTR_POS  (16u) /* [18:16] TX FIFO read pointer         */
#define PSOC5_TX_FIFO_STATUS_WR_PTR_POS  (24u) /* [26:24] TX FIFO write pointer        */
#define PSOC5_TX_FIFO_STATUS_USED_MASK   ((uint32) 0x0Fu)
#define PSOC5_TX_FIFO_SR_VALID           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_TX_FIFO_SR_VALID_POS))
#define PSOC5_TX_FIFO_STATUS_RD_PTR_MASK ((uint32) ((uint32) 0x07u << \
                                                                    PSOC5_TX_FIFO_STATUS_RD_PTR_POS))
#define PSOC5_TX_FIFO_STATUS_WR_PTR_MASK ((uint32) ((uint32) 0x07u << \
                                                                    PSOC5_TX_FIFO_STATUS_WR_PTR_POS))


/* PSOC5_TX_FIFO_WR_REG */
#define PSOC5_TX_FIFO_WR_POS    (0u)  /* [15:0] Data written into TX FIFO */
#define PSOC5_TX_FIFO_WR_MASK   ((uint32) 0xFFu)


/* PSOC5_RX_CTRL_REG */
#define PSOC5_RX_CTRL_DATA_WIDTH_POS     (0u)  /* [3:0] Dataframe width: (Data width - 1) */
#define PSOC5_RX_CTRL_MSB_FIRST_POS      (8u)  /* [8]   MSB first shifter-out             */
#define PSOC5_RX_CTRL_MEDIAN_POS         (9u)  /* [9]   Median filter                     */
#define PSOC5_RX_CTRL_ENABLED_POS        (31u) /* [31]  Receiver enabled                  */
#define PSOC5_RX_CTRL_DATA_WIDTH_MASK    ((uint32) 0x0Fu)
#define PSOC5_RX_CTRL_MSB_FIRST          ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_RX_CTRL_MSB_FIRST_POS))
#define PSOC5_RX_CTRL_LSB_FIRST          ((uint32) 0x00u)
#define PSOC5_RX_CTRL_MEDIAN             ((uint32) ((uint32) 0x01u << PSOC5_RX_CTRL_MEDIAN_POS))
#define PSOC5_RX_CTRL_ENABLED            ((uint32) ((uint32) 0x01u << PSOC5_RX_CTRL_ENABLED_POS))


/* PSOC5_RX_FIFO_CTRL_REG */
#define PSOC5_RX_FIFO_CTRL_TRIGGER_LEVEL_POS     (0u)   /* [2:0] Trigger level                            */
#define PSOC5_RX_FIFO_CTRL_CLEAR_POS             (16u)  /* [16]  Clear RX FIFO: claar after set           */
#define PSOC5_RX_FIFO_CTRL_FREEZE_POS            (17u)  /* [17]  Freeze RX FIFO: HW writes has not effect */
#define PSOC5_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK    ((uint32) 0x07u)
#define PSOC5_RX_FIFO_CTRL_CLEAR                 ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_RX_FIFO_CTRL_CLEAR_POS))
#define PSOC5_RX_FIFO_CTRL_FREEZE                ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_RX_FIFO_CTRL_FREEZE_POS))


/* PSOC5_RX_FIFO_STATUS_REG */
#define PSOC5_RX_FIFO_STATUS_USED_POS    (0u)   /* [3:0]   Amount of entries in RX FIFO */
#define PSOC5_RX_FIFO_SR_VALID_POS       (15u)  /* [15]    Shifter status of RX FIFO    */
#define PSOC5_RX_FIFO_STATUS_RD_PTR_POS  (16u)  /* [18:16] RX FIFO read pointer         */
#define PSOC5_RX_FIFO_STATUS_WR_PTR_POS  (24u)  /* [26:24] RX FIFO write pointer        */
#define PSOC5_RX_FIFO_STATUS_USED_MASK   ((uint32) 0x0Fu)
#define PSOC5_RX_FIFO_SR_VALID           ((uint32) ((uint32) 0x01u << \
                                                                  PSOC5_RX_FIFO_SR_VALID_POS))
#define PSOC5_RX_FIFO_STATUS_RD_PTR_MASK ((uint32) ((uint32) 0x07u << \
                                                                  PSOC5_RX_FIFO_STATUS_RD_PTR_POS))
#define PSOC5_RX_FIFO_STATUS_WR_PTR_MASK ((uint32) ((uint32) 0x07u << \
                                                                  PSOC5_RX_FIFO_STATUS_WR_PTR_POS))


/* PSOC5_RX_MATCH_REG */
#define PSOC5_RX_MATCH_ADDR_POS     (0u)  /* [7:0]   Slave address                        */
#define PSOC5_RX_MATCH_MASK_POS     (16u) /* [23:16] Slave address mask: 0 - doesn't care */
#define PSOC5_RX_MATCH_ADDR_MASK    ((uint32) 0xFFu)
#define PSOC5_RX_MATCH_MASK_MASK    ((uint32) ((uint32) 0xFFu << PSOC5_RX_MATCH_MASK_POS))


/* PSOC5_RX_FIFO_WR_REG */
#define PSOC5_RX_FIFO_RD_POS    (0u)  /* [15:0] Data read from RX FIFO */
#define PSOC5_RX_FIFO_RD_MASK   ((uint32) 0xFFu)


/* PSOC5_RX_FIFO_RD_SILENT_REG */
#define PSOC5_RX_FIFO_RD_SILENT_POS     (0u)  /* [15:0] Data read from RX FIFO: not remove data from FIFO */
#define PSOC5_RX_FIFO_RD_SILENT_MASK    ((uint32) 0xFFu)

/* PSOC5_RX_FIFO_RD_SILENT_REG */
#define PSOC5_RX_FIFO_RD_SILENT_POS     (0u)  /* [15:0] Data read from RX FIFO: not remove data from FIFO */
#define PSOC5_RX_FIFO_RD_SILENT_MASK    ((uint32) 0xFFu)

/* PSOC5_EZBUF_DATA_REG */
#define PSOC5_EZBUF_DATA_POS   (0u)  /* [7:0] Data from Ez Memory */
#define PSOC5_EZBUF_DATA_MASK  ((uint32) 0xFFu)

/*  PSOC5_INTR_CAUSE_REG */
#define PSOC5_INTR_CAUSE_MASTER_POS  (0u)  /* [0] Master interrupt active                 */
#define PSOC5_INTR_CAUSE_SLAVE_POS   (1u)  /* [1] Slave interrupt active                  */
#define PSOC5_INTR_CAUSE_TX_POS      (2u)  /* [2] Transmitter interrupt active            */
#define PSOC5_INTR_CAUSE_RX_POS      (3u)  /* [3] Receiver interrupt active               */
#define PSOC5_INTR_CAUSE_I2C_EC_POS  (4u)  /* [4] Externally clock I2C interrupt active   */
#define PSOC5_INTR_CAUSE_SPI_EC_POS  (5u)  /* [5] Externally clocked SPI interrupt active */
#define PSOC5_INTR_CAUSE_MASTER      ((uint32) 0x01u)
#define PSOC5_INTR_CAUSE_SLAVE       ((uint32) ((uint32) 0x01u << PSOC5_INTR_CAUSE_SLAVE_POS))
#define PSOC5_INTR_CAUSE_TX          ((uint32) ((uint32) 0x01u << PSOC5_INTR_CAUSE_TX_POS))
#define PSOC5_INTR_CAUSE_RX          ((uint32) ((uint32) 0x01u << PSOC5_INTR_CAUSE_RX_POS))
#define PSOC5_INTR_CAUSE_I2C_EC      ((uint32) ((uint32) 0x01u << PSOC5_INTR_CAUSE_I2C_EC_POS))
#define PSOC5_INTR_CAUSE_SPI_EC      ((uint32) ((uint32) 0x01u << PSOC5_INTR_CAUSE_SPI_EC_POS))


/* PSOC5_INTR_SPI_EC_REG, PSOC5_INTR_SPI_EC_MASK_REG, PSOC5_INTR_SPI_EC_MASKED_REG */
#define PSOC5_INTR_SPI_EC_WAKE_UP_POS          (0u)  /* [0] Address match: triggers wakeup of chip */
#define PSOC5_INTR_SPI_EC_EZBUF_STOP_POS       (1u)  /* [1] Externally clocked Stop detected       */
#define PSOC5_INTR_SPI_EC_EZBUF_WRITE_STOP_POS (2u)  /* [2] Externally clocked Write Stop detected */
#define PSOC5_INTR_SPI_EC_WAKE_UP              ((uint32) 0x01u)
#define PSOC5_INTR_SPI_EC_EZBUF_STOP           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SPI_EC_EZBUF_STOP_POS))
#define PSOC5_INTR_SPI_EC_EZBUF_WRITE_STOP     ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SPI_EC_EZBUF_WRITE_STOP_POS))


/* PSOC5_INTR_I2C_EC, PSOC5_INTR_I2C_EC_MASK, PSOC5_INTR_I2C_EC_MASKED */
#define PSOC5_INTR_I2C_EC_WAKE_UP_POS          (0u)  /* [0] Address match: triggers wakeup of chip */
#define PSOC5_INTR_I2C_EC_EZBUF_STOP_POS       (1u)  /* [1] Externally clocked Stop detected       */
#define PSOC5_INTR_I2C_EC_EZBUF_WRITE_STOP_POS (2u)  /* [2] Externally clocked Write Stop detected */
#define PSOC5_INTR_I2C_EC_WAKE_UP              ((uint32) 0x01u)
#define PSOC5_INTR_I2C_EC_EZBUF_STOP           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_I2C_EC_EZBUF_STOP_POS))
#define PSOC5_INTR_I2C_EC_EZBUF_WRITE_STOP     ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_I2C_EC_EZBUF_WRITE_STOP_POS))


/* PSOC5_INTR_MASTER, PSOC5_INTR_MASTER_SET,
   PSOC5_INTR_MASTER_MASK, PSOC5_INTR_MASTER_MASKED */
#define PSOC5_INTR_MASTER_I2C_ARB_LOST_POS   (0u)  /* [0] Master lost arbitration                          */
#define PSOC5_INTR_MASTER_I2C_NACK_POS       (1u)  /* [1] Master receives NACK: address or write to slave  */
#define PSOC5_INTR_MASTER_I2C_ACK_POS        (2u)  /* [2] Master receives NACK: address or write to slave  */
#define PSOC5_INTR_MASTER_I2C_STOP_POS       (4u)  /* [4] Master detects the Stop: only self generated Stop*/
#define PSOC5_INTR_MASTER_I2C_BUS_ERROR_POS  (8u)  /* [8] Master detects bus error: misplaced Start or Stop*/
#define PSOC5_INTR_MASTER_SPI_DONE_POS       (9u)  /* [9] Master complete trasfer: Only for SPI            */
#define PSOC5_INTR_MASTER_I2C_ARB_LOST       ((uint32) 0x01u)
#define PSOC5_INTR_MASTER_I2C_NACK           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_MASTER_I2C_NACK_POS))
#define PSOC5_INTR_MASTER_I2C_ACK            ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_MASTER_I2C_ACK_POS))
#define PSOC5_INTR_MASTER_I2C_STOP           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_MASTER_I2C_STOP_POS))
#define PSOC5_INTR_MASTER_I2C_BUS_ERROR      ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_MASTER_I2C_BUS_ERROR_POS))
#define PSOC5_INTR_MASTER_SPI_DONE           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_MASTER_SPI_DONE_POS))

/*
* PSOC5_INTR_SLAVE, PSOC5_INTR_SLAVE_SET,
* PSOC5_INTR_SLAVE_MASK, PSOC5_INTR_SLAVE_MASKED
*/
#define PSOC5_INTR_SLAVE_I2C_ARB_LOST_POS         (0u)  /* [0]  Slave lost arbitration                   */
#define PSOC5_INTR_SLAVE_I2C_NACK_POS             (1u)  /* [1]  Slave receives NACK: master reads data   */
#define PSOC5_INTR_SLAVE_I2C_ACK_POS              (2u)  /* [2]  Slave receives ACK: master reads data    */
#define PSOC5_INTR_SLAVE_I2C_WRITE_STOP_POS       (3u)  /* [3]  Slave detects end of write transaction   */
#define PSOC5_INTR_SLAVE_I2C_STOP_POS             (4u)  /* [4]  Slave detects end of transaction intened */
#define PSOC5_INTR_SLAVE_I2C_START_POS            (5u)  /* [5]  Slave detects Start                      */
#define PSOC5_INTR_SLAVE_I2C_ADDR_MATCH_POS       (6u)  /* [6]  Slave address matches                    */
#define PSOC5_INTR_SLAVE_I2C_GENERAL_POS          (7u)  /* [7]  General call received                    */
#define PSOC5_INTR_SLAVE_I2C_BUS_ERROR_POS        (8u)  /* [8]  Slave detects bus error                  */
#define PSOC5_INTR_SLAVE_SPI_EZBUF_WRITE_STOP_POS (9u)  /* [9]  Slave write complete: Only for SPI       */
#define PSOC5_INTR_SLAVE_SPI_EZBUF_STOP_POS       (10u) /* [10] Slave end of transaciton: Only for SPI   */
#define PSOC5_INTR_SLAVE_SPI_BUS_ERROR_POS        (11u) /* [11] Slave detects bus error: Only for SPI    */
#define PSOC5_INTR_SLAVE_I2C_ARB_LOST             ((uint32) 0x01u)
#define PSOC5_INTR_SLAVE_I2C_NACK                 ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_NACK_POS))
#define PSOC5_INTR_SLAVE_I2C_ACK                  ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_ACK_POS))
#define PSOC5_INTR_SLAVE_I2C_WRITE_STOP           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_WRITE_STOP_POS))
#define PSOC5_INTR_SLAVE_I2C_STOP                 ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_STOP_POS))
#define PSOC5_INTR_SLAVE_I2C_START                ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_START_POS))
#define PSOC5_INTR_SLAVE_I2C_ADDR_MATCH           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_ADDR_MATCH_POS))
#define PSOC5_INTR_SLAVE_I2C_GENERAL              ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_GENERAL_POS))
#define PSOC5_INTR_SLAVE_I2C_BUS_ERROR            ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_I2C_BUS_ERROR_POS))
#define PSOC5_INTR_SLAVE_SPI_EZBUF_WRITE_STOP     ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_SPI_EZBUF_WRITE_STOP_POS))
#define PSOC5_INTR_SLAVE_SPI_EZBUF_STOP           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_SPI_EZBUF_STOP_POS))
#define PSOC5_INTR_SLAVE_SPI_BUS_ERROR           ((uint32) ((uint32) 0x01u << \
                                                                    PSOC5_INTR_SLAVE_SPI_BUS_ERROR_POS))


/*
* PSOC5_INTR_TX, PSOC5_INTR_TX_SET,
* PSOC5_INTR_TX_MASK, PSOC5_INTR_TX_MASKED
*/
#define PSOC5_INTR_TX_TRIGGER_POS        (0u)  /* [0]  Trigger on TX FIFO entires                       */
#define PSOC5_INTR_TX_NOT_FULL_POS       (1u)  /* [1]  TX FIFO is not full                              */
#define PSOC5_INTR_TX_EMPTY_POS          (4u)  /* [4]  TX FIFO is empty                                 */
#define PSOC5_INTR_TX_OVERFLOW_POS       (5u)  /* [5]  Attempt to write to a full TX FIFO               */
#define PSOC5_INTR_TX_UNDERFLOW_POS      (6u)  /* [6]  Attempt to read from an empty TX FIFO            */
#define PSOC5_INTR_TX_BLOCKED_POS        (7u)  /* [7]  No access to the EZ memory                       */
#define PSOC5_INTR_TX_UART_NACK_POS      (8u)  /* [8]  UART transmitter received a NACK: SmartCard mode */
#define PSOC5_INTR_TX_UART_DONE_POS      (9u)  /* [9]  UART transmitter done even                       */
#define PSOC5_INTR_TX_UART_ARB_LOST_POS  (10u) /* [10] UART lost arbitration: LIN or SmartCard          */
#define PSOC5_INTR_TX_TRIGGER            ((uint32) 0x01u)
#define PSOC5_INTR_TX_NOT_FULL           ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_NOT_FULL_POS))
#define PSOC5_INTR_TX_EMPTY              ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_EMPTY_POS))
#define PSOC5_INTR_TX_OVERFLOW           ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_OVERFLOW_POS))
#define PSOC5_INTR_TX_UNDERFLOW          ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_UNDERFLOW_POS))
#define PSOC5_INTR_TX_BLOCKED            ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_BLOCKED_POS))
#define PSOC5_INTR_TX_UART_NACK          ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_UART_NACK_POS))
#define PSOC5_INTR_TX_UART_DONE          ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_UART_DONE_POS))
#define PSOC5_INTR_TX_UART_ARB_LOST      ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_TX_UART_ARB_LOST_POS))


/*
* PSOC5_INTR_RX, PSOC5_INTR_RX_SET,
* PSOC5_INTR_RX_MASK, PSOC5_INTR_RX_MASKED
*/
#define PSOC5_INTR_RX_TRIGGER_POS        (0u)   /* [0]  Trigger on RX FIFO entires            */
#define PSOC5_INTR_RX_NOT_EMPTY_POS      (2u)   /* [2]  RX FIFO is not empty                  */
#define PSOC5_INTR_RX_FULL_POS           (3u)   /* [3]  RX FIFO is full                       */
#define PSOC5_INTR_RX_OVERFLOW_POS       (5u)   /* [5]  Attempt to write to a full RX FIFO    */
#define PSOC5_INTR_RX_UNDERFLOW_POS      (6u)   /* [6]  Attempt to read from an empty RX FIFO */
#define PSOC5_INTR_RX_BLOCKED_POS        (7u)   /* [7]  No access to the EZ memory            */
#define PSOC5_INTR_RX_FRAME_ERROR_POS    (8u)   /* [8]  Frame error in received data frame    */
#define PSOC5_INTR_RX_PARITY_ERROR_POS   (9u)   /* [9]  Parity error in received data frame   */
#define PSOC5_INTR_RX_BAUD_DETECT_POS    (10u)  /* [10] LIN baudrate detection is completed   */
#define PSOC5_INTR_RX_BREAK_DETECT_POS   (11u)  /* [11] Break detection is successful         */
#define PSOC5_INTR_RX_TRIGGER            ((uint32) 0x01u)
#define PSOC5_INTR_RX_NOT_EMPTY          ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_NOT_EMPTY_POS))
#define PSOC5_INTR_RX_FULL               ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_FULL_POS))
#define PSOC5_INTR_RX_OVERFLOW           ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_OVERFLOW_POS))
#define PSOC5_INTR_RX_UNDERFLOW          ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_UNDERFLOW_POS))
#define PSOC5_INTR_RX_BLOCKED            ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_BLOCKED_POS))
#define PSOC5_INTR_RX_FRAME_ERROR        ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_FRAME_ERROR_POS))
#define PSOC5_INTR_RX_PARITY_ERROR       ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_PARITY_ERROR_POS))
#define PSOC5_INTR_RX_BAUD_DETECT        ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_BAUD_DETECT_POS))
#define PSOC5_INTR_RX_BREAK_DETECT       ((uint32) ((uint32) 0x01u << \
                                                                        PSOC5_INTR_RX_BREAK_DETECT_POS))


/* Define all interupt soureces */
#define PSOC5_INTR_I2C_EC_ALL    (PSOC5_INTR_I2C_EC_WAKE_UP    | \
                                             PSOC5_INTR_I2C_EC_EZBUF_STOP | \
                                             PSOC5_INTR_I2C_EC_EZBUF_WRITE_STOP)

#define PSOC5_INTR_SPI_EC_ALL    (PSOC5_INTR_SPI_EC_WAKE_UP    | \
                                             PSOC5_INTR_SPI_EC_EZBUF_STOP | \
                                             PSOC5_INTR_SPI_EC_EZBUF_WRITE_STOP)

#define PSOC5_INTR_MASTER_ALL    (PSOC5_INTR_MASTER_I2C_ARB_LOST  | \
                                             PSOC5_INTR_MASTER_I2C_NACK      | \
                                             PSOC5_INTR_MASTER_I2C_ACK       | \
                                             PSOC5_INTR_MASTER_I2C_STOP      | \
                                             PSOC5_INTR_MASTER_I2C_BUS_ERROR | \
                                             PSOC5_INTR_MASTER_SPI_DONE )

#define PSOC5_INTR_SLAVE_ALL     (PSOC5_INTR_SLAVE_I2C_ARB_LOST      | \
                                             PSOC5_INTR_SLAVE_I2C_NACK          | \
                                             PSOC5_INTR_SLAVE_I2C_ACK           | \
                                             PSOC5_INTR_SLAVE_I2C_WRITE_STOP    | \
                                             PSOC5_INTR_SLAVE_I2C_STOP          | \
                                             PSOC5_INTR_SLAVE_I2C_START         | \
                                             PSOC5_INTR_SLAVE_I2C_ADDR_MATCH    | \
                                             PSOC5_INTR_SLAVE_I2C_GENERAL       | \
                                             PSOC5_INTR_SLAVE_I2C_BUS_ERROR     | \
                                             PSOC5_INTR_SLAVE_SPI_EZBUF_WRITE_STOP | \
                                             PSOC5_INTR_SLAVE_SPI_EZBUF_STOP       | \
                                             PSOC5_INTR_SLAVE_SPI_BUS_ERROR)

#define PSOC5_INTR_TX_ALL        (PSOC5_INTR_TX_TRIGGER   | \
                                             PSOC5_INTR_TX_NOT_FULL  | \
                                             PSOC5_INTR_TX_EMPTY     | \
                                             PSOC5_INTR_TX_OVERFLOW  | \
                                             PSOC5_INTR_TX_UNDERFLOW | \
                                             PSOC5_INTR_TX_BLOCKED   | \
                                             PSOC5_INTR_TX_UART_NACK | \
                                             PSOC5_INTR_TX_UART_DONE | \
                                             PSOC5_INTR_TX_UART_ARB_LOST)

#define PSOC5_INTR_RX_ALL        (PSOC5_INTR_RX_TRIGGER      | \
                                             PSOC5_INTR_RX_NOT_EMPTY    | \
                                             PSOC5_INTR_RX_FULL         | \
                                             PSOC5_INTR_RX_OVERFLOW     | \
                                             PSOC5_INTR_RX_UNDERFLOW    | \
                                             PSOC5_INTR_RX_BLOCKED      | \
                                             PSOC5_INTR_RX_FRAME_ERROR  | \
                                             PSOC5_INTR_RX_PARITY_ERROR | \
                                             PSOC5_INTR_RX_BAUD_DETECT  | \
                                             PSOC5_INTR_RX_BREAK_DETECT)

/* General usage HW definitions */
#define PSOC5_ONE_BYTE_WIDTH (8u)   /* Number of bits in one byte           */
#define PSOC5_FIFO_SIZE      (8u)   /* Size of TX or RX FIFO: defined by HW */
#define PSOC5_EZBUFFER_SIZE  (32u)  /* EZ Buffer size: defined by HW        */

/* I2C and EZI2C slave address defines */
#define PSOC5_I2C_SLAVE_ADDR_POS    (0x01u)    /* 7-bit address shift */
#define PSOC5_I2C_SLAVE_ADDR_MASK   (0xFEu)    /* 8-bit address mask */

/* OVS constants for IrDA Low Power operation */
#define PSOC5_CTRL_OVS_IRDA_LP_OVS16     (0x00u)
#define PSOC5_CTRL_OVS_IRDA_LP_OVS32     (0x01u)
#define PSOC5_CTRL_OVS_IRDA_LP_OVS48     (0x02u)
#define PSOC5_CTRL_OVS_IRDA_LP_OVS96     (0x03u)
#define PSOC5_CTRL_OVS_IRDA_LP_OVS192    (0x04u)
#define PSOC5_CTRL_OVS_IRDA_LP_OVS768    (0x05u)
#define PSOC5_CTRL_OVS_IRDA_LP_OVS1536   (0x06u)

/* OVS constant for IrDA */
#define PSOC5_CTRL_OVS_IRDA_OVS16        (PSOC5_UART_IRDA_LP_OVS16)


/***************************************
*    SCB Common Macro Definitions
***************************************/

/*
* Re-enables SCB IP: this cause partial reset of IP: state, status, TX and RX FIFOs.
* The triggered interrupts remains set.
*/
#define PSOC5_SCB_SW_RESET \
                        do{ \
                            PSOC5_CTRL_REG &= ((uint32) ~PSOC5_CTRL_ENABLED ); \
                            PSOC5_CTRL_REG |= ((uint32)  PSOC5_CTRL_ENABLED ); \
                        }while(0)

/* TX FIFO macro */
#define PSOC5_CLEAR_TX_FIFO \
                            do{        \
                                PSOC5_TX_FIFO_CTRL_REG |= ((uint32)  PSOC5_TX_FIFO_CTRL_CLEAR); \
                                PSOC5_TX_FIFO_CTRL_REG &= ((uint32) ~PSOC5_TX_FIFO_CTRL_CLEAR); \
                            }while(0)

#define PSOC5_GET_TX_FIFO_ENTRIES    (PSOC5_TX_FIFO_STATUS_REG & \
                                                 PSOC5_TX_FIFO_STATUS_USED_MASK)

#define PSOC5_GET_TX_FIFO_SR_VALID   ((0u != (PSOC5_TX_FIFO_STATUS_REG & \
                                                         PSOC5_TX_FIFO_SR_VALID)) ? (1u) : (0u))

/* RX FIFO macro */
#define PSOC5_CLEAR_RX_FIFO \
                            do{        \
                                PSOC5_RX_FIFO_CTRL_REG |= ((uint32)  PSOC5_RX_FIFO_CTRL_CLEAR); \
                                PSOC5_RX_FIFO_CTRL_REG &= ((uint32) ~PSOC5_RX_FIFO_CTRL_CLEAR); \
                            }while(0)

#define PSOC5_GET_RX_FIFO_ENTRIES    (PSOC5_RX_FIFO_STATUS_REG & \
                                                    PSOC5_RX_FIFO_STATUS_USED_MASK)

#define PSOC5_GET_RX_FIFO_SR_VALID   ((0u != (PSOC5_RX_FIFO_STATUS_REG & \
                                                         PSOC5_RX_FIFO_SR_VALID)) ? (1u) : (0u))

/* Writes interrupt source: set sourceMask bits in PSOC5_INTR_X_MASK_REG */
#define PSOC5_WRITE_INTR_I2C_EC_MASK(sourceMask) \
                                                do{         \
                                                    PSOC5_INTR_I2C_EC_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_WRITE_INTR_SPI_EC_MASK(sourceMask) \
                                                do{         \
                                                    PSOC5_INTR_SPI_EC_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_WRITE_INTR_MASTER_MASK(sourceMask) \
                                                do{         \
                                                    PSOC5_INTR_MASTER_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_WRITE_INTR_SLAVE_MASK(sourceMask)  \
                                                do{         \
                                                    PSOC5_INTR_SLAVE_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_WRITE_INTR_TX_MASK(sourceMask)     \
                                                do{         \
                                                    PSOC5_INTR_TX_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_WRITE_INTR_RX_MASK(sourceMask)     \
                                                do{         \
                                                    PSOC5_INTR_RX_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

/* Enables interrupt source: set sourceMask bits in PSOC5_INTR_X_MASK_REG */
#define PSOC5_ENABLE_INTR_I2C_EC(sourceMask) \
                                                do{     \
                                                    PSOC5_INTR_I2C_EC_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_ENABLE_INTR_SPI_EC(sourceMask) \
                                                do{     \
                                                    PSOC5_INTR_SPI_EC_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_ENABLE_INTR_MASTER(sourceMask) \
                                                do{     \
                                                    PSOC5_INTR_MASTER_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_ENABLE_INTR_SLAVE(sourceMask)  \
                                                do{     \
                                                    PSOC5_INTR_SLAVE_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_ENABLE_INTR_TX(sourceMask)     \
                                                do{     \
                                                    PSOC5_INTR_TX_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_ENABLE_INTR_RX(sourceMask)     \
                                                do{     \
                                                    PSOC5_INTR_RX_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

/* Disables interrupt source: clear sourceMask bits in PSOC5_INTR_X_MASK_REG */
#define PSOC5_DISABLE_INTR_I2C_EC(sourceMask) \
                                do{                      \
                                    PSOC5_INTR_I2C_EC_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define PSOC5_DISABLE_INTR_SPI_EC(sourceMask) \
                                do{                      \
                                    PSOC5_INTR_SPI_EC_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                 }while(0)

#define PSOC5_DISABLE_INTR_MASTER(sourceMask) \
                                do{                      \
                                PSOC5_INTR_MASTER_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define PSOC5_DISABLE_INTR_SLAVE(sourceMask) \
                                do{                     \
                                    PSOC5_INTR_SLAVE_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define PSOC5_DISABLE_INTR_TX(sourceMask)    \
                                do{                     \
                                    PSOC5_INTR_TX_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                 }while(0)

#define PSOC5_DISABLE_INTR_RX(sourceMask)    \
                                do{                     \
                                    PSOC5_INTR_RX_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

/* Sets interrupt sources: write sourceMask bits in PSOC5_INTR_X_SET_REG */
#define PSOC5_SET_INTR_MASTER(sourceMask)    \
                                                do{     \
                                                    PSOC5_INTR_MASTER_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_SET_INTR_SLAVE(sourceMask) \
                                                do{ \
                                                    PSOC5_INTR_SLAVE_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_SET_INTR_TX(sourceMask)    \
                                                do{ \
                                                    PSOC5_INTR_TX_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_SET_INTR_RX(sourceMask)    \
                                                do{ \
                                                    PSOC5_INTR_RX_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

/* Clears interrupt sources: write sourceMask bits in PSOC5_INTR_X_REG */
#define PSOC5_CLEAR_INTR_I2C_EC(sourceMask)  \
                                                do{     \
                                                    PSOC5_INTR_I2C_EC_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_CLEAR_INTR_SPI_EC(sourceMask)  \
                                                do{     \
                                                    PSOC5_INTR_SPI_EC_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_CLEAR_INTR_MASTER(sourceMask)  \
                                                do{     \
                                                    PSOC5_INTR_MASTER_REG = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_CLEAR_INTR_SLAVE(sourceMask)   \
                                                do{     \
                                                    PSOC5_INTR_SLAVE_REG  = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_CLEAR_INTR_TX(sourceMask)      \
                                                do{     \
                                                    PSOC5_INTR_TX_REG     = (uint32) (sourceMask); \
                                                }while(0)

#define PSOC5_CLEAR_INTR_RX(sourceMask)      \
                                                do{     \
                                                    PSOC5_INTR_RX_REG     = (uint32) (sourceMask); \
                                                }while(0)

/* Return true if sourceMask is set in PSOC5_INTR_CAUSE_REG */
#define PSOC5_CHECK_CAUSE_INTR(sourceMask)    (0u != (PSOC5_INTR_CAUSE_REG & (sourceMask)))

/* Return true if sourceMask is set in  INTR_X_MASKED_REG */
#define PSOC5_CHECK_INTR_EC_I2C(sourceMask)  (0u != (PSOC5_INTR_I2C_EC_REG & (sourceMask)))
#define PSOC5_CHECK_INTR_EC_SPI(sourceMask)  (0u != (PSOC5_INTR_SPI_EC_REG & (sourceMask)))
#define PSOC5_CHECK_INTR_MASTER(sourceMask)  (0u != (PSOC5_INTR_MASTER_REG & (sourceMask)))
#define PSOC5_CHECK_INTR_SLAVE(sourceMask)   (0u != (PSOC5_INTR_SLAVE_REG  & (sourceMask)))
#define PSOC5_CHECK_INTR_TX(sourceMask)      (0u != (PSOC5_INTR_TX_REG     & (sourceMask)))
#define PSOC5_CHECK_INTR_RX(sourceMask)      (0u != (PSOC5_INTR_RX_REG     & (sourceMask)))

/* Return true if sourceMask is set in PSOC5_INTR_X_MASKED_REG */
#define PSOC5_CHECK_INTR_I2C_EC_MASKED(sourceMask)   (0u != (PSOC5_INTR_I2C_EC_MASKED_REG & \
                                                                       (sourceMask)))
#define PSOC5_CHECK_INTR_SPI_EC_MASKED(sourceMask)   (0u != (PSOC5_INTR_SPI_EC_MASKED_REG & \
                                                                       (sourceMask)))
#define PSOC5_CHECK_INTR_MASTER_MASKED(sourceMask)   (0u != (PSOC5_INTR_MASTER_MASKED_REG & \
                                                                       (sourceMask)))
#define PSOC5_CHECK_INTR_SLAVE_MASKED(sourceMask)    (0u != (PSOC5_INTR_SLAVE_MASKED_REG  & \
                                                                       (sourceMask)))
#define PSOC5_CHECK_INTR_TX_MASKED(sourceMask)       (0u != (PSOC5_INTR_TX_MASKED_REG     & \
                                                                       (sourceMask)))
#define PSOC5_CHECK_INTR_RX_MASKED(sourceMask)       (0u != (PSOC5_INTR_RX_MASKED_REG     & \
                                                                       (sourceMask)))

/* Return true if sourceMask is set in PSOC5_CTRL_REG: generaly is used to check enable bit */
#define PSOC5_GET_CTRL_ENABLED    (0u != (PSOC5_CTRL_REG & PSOC5_CTRL_ENABLED))

#define PSOC5_CHECK_SLAVE_AUTO_ADDR_NACK     (0u != (PSOC5_I2C_CTRL_REG & \
                                                                PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK))


/***************************************
*      I2C Macro Definitions
***************************************/

/* Enable auto ACK/NACK */
#define PSOC5_ENABLE_SLAVE_AUTO_ADDR_NACK \
                            do{                      \
                                PSOC5_I2C_CTRL_REG |= PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define PSOC5_ENABLE_SLAVE_AUTO_DATA_ACK \
                            do{                     \
                                PSOC5_I2C_CTRL_REG |= PSOC5_I2C_CTRL_S_READY_DATA_ACK; \
                            }while(0)

#define PSOC5_ENABLE_SLAVE_AUTO_DATA_NACK \
                            do{                      \
                                PSOC5_I2C_CTRL_REG |= PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define PSOC5_ENABLE_MASTER_AUTO_DATA_ACK \
                            do{                      \
                                PSOC5_I2C_CTRL_REG |= PSOC5_I2C_CTRL_M_READY_DATA_ACK; \
                            }while(0)

#define PSOC5_ENABLE_MASTER_AUTO_DATA_NACK \
                            do{                       \
                                PSOC5_I2C_CTRL_REG |= PSOC5_I2C_CTRL_M_NOT_READY_DATA_NACK; \
                            }while(0)

/* Disable auto ACK/NACK */
#define PSOC5_DISABLE_SLAVE_AUTO_ADDR_NACK \
                            do{                       \
                                PSOC5_I2C_CTRL_REG &= ~PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define PSOC5_DISABLE_SLAVE_AUTO_DATA_ACK \
                            do{                      \
                                PSOC5_I2C_CTRL_REG &= ~PSOC5_I2C_CTRL_S_READY_DATA_ACK; \
                            }while(0)

#define PSOC5_DISABLE_SLAVE_AUTO_DATA_NACK \
                            do{                       \
                                PSOC5_I2C_CTRL_REG &= ~PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define PSOC5_DISABLE_MASTER_AUTO_DATA_ACK \
                            do{                       \
                                PSOC5_I2C_CTRL_REG &= ~PSOC5_I2C_CTRL_M_READY_DATA_ACK; \
                            }while(0)

#define PSOC5_DISABLE_MASTER_AUTO_DATA_NACK \
                            do{                        \
                                PSOC5_I2C_CTRL_REG &= ~PSOC5_I2C_CTRL_M_NOT_READY_DATA_NACK; \
                            }while(0)

/* Enable Slave autoACK/NACK Data */
#define PSOC5_ENABLE_SLAVE_AUTO_DATA \
                            do{                 \
                                PSOC5_I2C_CTRL_REG |= (PSOC5_I2C_CTRL_S_READY_DATA_ACK |      \
                                                                  PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK); \
                            }while(0)

/* Disable Slave autoACK/NACK Data */
#define PSOC5_DISABLE_SLAVE_AUTO_DATA \
                            do{                  \
                                PSOC5_I2C_CTRL_REG &= ((uint32) \
                                                                  ~(PSOC5_I2C_CTRL_S_READY_DATA_ACK |       \
                                                                    PSOC5_I2C_CTRL_S_NOT_READY_DATA_NACK)); \
                            }while(0)

/* Disable Master autoACK/NACK Data */
#define PSOC5_DISABLE_MASTER_AUTO_DATA \
                            do{                   \
                                PSOC5_I2C_CTRL_REG &= ((uint32) \
                                                                  ~(PSOC5_I2C_CTRL_M_READY_DATA_ACK |       \
                                                                    PSOC5_I2C_CTRL_M_NOT_READY_DATA_NACK)); \
                            }while(0)

/* Master commands */
#define PSOC5_I2C_MASTER_GENERATE_START \
                            do{                    \
                                PSOC5_I2C_MASTER_CMD_REG = PSOC5_I2C_MASTER_CMD_M_START_ON_IDLE; \
                            }while(0)

#define PSOC5_I2C_MASTER_CLEAR_START \
                            do{                 \
                                PSOC5_I2C_MASTER_CMD_REG =  ((uint32) 0u); \
                            }while(0)

#define PSOC5_I2C_MASTER_GENERATE_RESTART PSOC5_I2CReStartGeneration()

#define PSOC5_I2C_MASTER_GENERATE_STOP \
                            do{                   \
                                PSOC5_I2C_MASTER_CMD_REG =                                            \
                                    (PSOC5_I2C_MASTER_CMD_M_STOP |                                    \
                                        (PSOC5_CHECK_I2C_STATUS(PSOC5_I2C_STATUS_M_READ) ? \
                                            (PSOC5_I2C_MASTER_CMD_M_NACK) : (0u)));                   \
                            }while(0)

#define PSOC5_I2C_MASTER_GENERATE_ACK \
                            do{                  \
                                PSOC5_I2C_MASTER_CMD_REG = PSOC5_I2C_MASTER_CMD_M_ACK; \
                            }while(0)

#define PSOC5_I2C_MASTER_GENERATE_NACK \
                            do{                   \
                                PSOC5_I2C_MASTER_CMD_REG = PSOC5_I2C_MASTER_CMD_M_NACK; \
                            }while(0)

/* Slave comamnds */
#define PSOC5_I2C_SLAVE_GENERATE_ACK \
                            do{                 \
                                PSOC5_I2C_SLAVE_CMD_REG = PSOC5_I2C_SLAVE_CMD_S_ACK; \
                            }while(0)

#define PSOC5_I2C_SLAVE_GENERATE_NACK \
                            do{                  \
                                PSOC5_I2C_SLAVE_CMD_REG = PSOC5_I2C_SLAVE_CMD_S_NACK; \
                            }while(0)


/* Return 8-bit address. The input address should be 7-bits */
#define PSOC5_GET_I2C_8BIT_ADDRESS(addr) (((uint32) ((uint32) (addr) << \
                                                                    PSOC5_I2C_SLAVE_ADDR_POS)) & \
                                                                        PSOC5_I2C_SLAVE_ADDR_MASK)

#define PSOC5_GET_I2C_7BIT_ADDRESS(addr) ((uint32) (addr) >> PSOC5_I2C_SLAVE_ADDR_POS)


/* Adjust SDA filter Trim settings */
#define PSOC5_DEFAULT_I2C_CFG_SDA_FILT_TRIM  (0x02u)
#define PSOC5_EC_AM_I2C_CFG_SDA_FILT_TRIM    (0x03u)

#define PSOC5_SET_I2C_CFG_SDA_FILT_TRIM(sdaTrim) \
        do{                                                 \
            PSOC5_I2C_CFG_REG =                  \
                            ((PSOC5_I2C_CFG_REG & (uint32) ~PSOC5_I2C_CFG_SDA_FILT_TRIM_MASK) | \
                             ((uint32) ((uint32) (sdaTrim) <<PSOC5_I2C_CFG_SDA_FILT_TRIM_POS)));           \
        }while(0)

/* Returns slave select number in SPI_CTRL */
#define PSOC5_GET_SPI_CTRL_SS(activeSelect) (((uint32) ((uint32) (activeSelect) << \
                                                                    PSOC5_SPI_CTRL_SLAVE_SELECT_POS)) & \
                                                                        PSOC5_SPI_CTRL_SLAVE_SELECT_MASK)

/* Returns true if bit is set in PSOC5_I2C_STATUS_REG */
#define PSOC5_CHECK_I2C_STATUS(sourceMask)   (0u != (PSOC5_I2C_STATUS_REG & (sourceMask)))

/* Returns true if bit is set in PSOC5_SPI_STATUS_REG */
#define PSOC5_CHECK_SPI_STATUS(sourceMask)   (0u != (PSOC5_SPI_STATUS_REG & (sourceMask)))


/***************************************
*       SCB Init Macros Definitions
***************************************/

/* PSOC5_CTRL */
#define PSOC5_GET_CTRL_OVS(oversample)   ((((uint32) (oversample)) - 1u) & PSOC5_CTRL_OVS_MASK)

#define PSOC5_GET_CTRL_EC_OP_MODE(opMode)        ((0u != (opMode)) ? \
                                                                (PSOC5_CTRL_EC_OP_MODE)  : (0u))

#define PSOC5_GET_CTRL_EC_AM_MODE(amMode)        ((0u != (amMode)) ? \
                                                                (PSOC5_CTRL_EC_AM_MODE)  : (0u))

#define PSOC5_GET_CTRL_BLOCK(block)              ((0u != (block))  ? \
                                                                (PSOC5_CTRL_BLOCK)       : (0u))

#define PSOC5_GET_CTRL_ADDR_ACCEPT(acceptAddr)   ((0u != (acceptAddr)) ? \
                                                                (PSOC5_CTRL_ADDR_ACCEPT) : (0u))

/* PSOC5_I2C_CTRL */
#define PSOC5_GET_I2C_CTRL_HIGH_PHASE_OVS(oversampleHigh) (((uint32) (oversampleHigh) - 1u) & \
                                                                        PSOC5_I2C_CTRL_HIGH_PHASE_OVS_MASK)

#define PSOC5_GET_I2C_CTRL_LOW_PHASE_OVS(oversampleLow)   (((uint32) (((uint32) (oversampleLow) - 1u) << \
                                                                    PSOC5_I2C_CTRL_LOW_PHASE_OVS_POS)) &  \
                                                                    PSOC5_I2C_CTRL_LOW_PHASE_OVS_MASK)

#define PSOC5_GET_I2C_CTRL_S_NOT_READY_ADDR_NACK(wakeNack) ((0u != (wakeNack)) ? \
                                                            (PSOC5_I2C_CTRL_S_NOT_READY_ADDR_NACK) : (0u))

#define PSOC5_GET_I2C_CTRL_SL_MSTR_MODE(mode)    ((uint32) ((uint32)(mode) << \
                                                                    PSOC5_I2C_CTRL_SLAVE_MODE_POS))

/* PSOC5_SPI_CTRL */
#define PSOC5_GET_SPI_CTRL_CONTINUOUS(separate)  ((0u != (separate)) ? \
                                                                (PSOC5_SPI_CTRL_CONTINUOUS) : (0u))

#define PSOC5_GET_SPI_CTRL_SELECT_PRECEDE(mode)  ((0u != (mode)) ? \
                                                                      (PSOC5_SPI_CTRL_SELECT_PRECEDE) : (0u))

#define PSOC5_GET_SPI_CTRL_SCLK_MODE(mode)       (((uint32) ((uint32) (mode) << \
                                                                        PSOC5_SPI_CTRL_CPHA_POS)) & \
                                                                        PSOC5_SPI_CTRL_SCLK_MODE_MASK)

#define PSOC5_GET_SPI_CTRL_LATE_MISO_SAMPLE(lateMiso) ((0u != (lateMiso)) ? \
                                                                    (PSOC5_SPI_CTRL_LATE_MISO_SAMPLE) : (0u))

#define PSOC5_GET_SPI_CTRL_SUB_MODE(mode)        (((uint32) (((uint32) (mode)) << \
                                                                        PSOC5_SPI_CTRL_MODE_POS)) & \
                                                                        PSOC5_SPI_CTRL_MODE_MASK)

#define PSOC5_GET_SPI_CTRL_SLAVE_SELECT(select)  (((uint32) ((uint32) (select) << \
                                                                      PSOC5_SPI_CTRL_SLAVE_SELECT_POS)) & \
                                                                      PSOC5_SPI_CTRL_SLAVE_SELECT_MASK)

#define PSOC5_GET_SPI_CTRL_MASTER_MODE(mode)     ((0u != (mode)) ? \
                                                                (PSOC5_SPI_CTRL_MASTER) : (0u))

/* PSOC5_UART_CTRL */
#define PSOC5_GET_UART_CTRL_MODE(mode)           (((uint32) ((uint32) (mode) << \
                                                                            PSOC5_UART_CTRL_MODE_POS)) & \
                                                                                PSOC5_UART_CTRL_MODE_MASK)

/* PSOC5_UART_RX_CTRL */
#define PSOC5_GET_UART_RX_CTRL_MODE(stopBits)    (((uint32) (stopBits) - 1u) & \
                                                                        PSOC5_UART_RX_CTRL_STOP_BITS_MASK)

#define PSOC5_GET_UART_RX_CTRL_PARITY(parity)    ((0u != (parity)) ? \
                                                                    (PSOC5_UART_RX_CTRL_PARITY) : (0u))

#define PSOC5_GET_UART_RX_CTRL_POLARITY(polarity)    ((0u != (polarity)) ? \
                                                                    (PSOC5_UART_RX_CTRL_POLARITY) : (0u))

#define PSOC5_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(dropErr) ((0u != (dropErr)) ? \
                                                        (PSOC5_UART_RX_CTRL_DROP_ON_PARITY_ERR) : (0u))

#define PSOC5_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(dropErr) ((0u != (dropErr)) ? \
                                                        (PSOC5_UART_RX_CTRL_DROP_ON_FRAME_ERR) : (0u))

#define PSOC5_GET_UART_RX_CTRL_MP_MODE(mpMode)   ((0u != (mpMode)) ? \
                                                        (PSOC5_UART_RX_CTRL_MP_MODE) : (0u))

/* PSOC5_UART_TX_CTRL */
#define PSOC5_GET_UART_TX_CTRL_MODE(stopBits)    (((uint32) (stopBits) - 1u) & \
                                                                PSOC5_UART_RX_CTRL_STOP_BITS_MASK)

#define PSOC5_GET_UART_TX_CTRL_PARITY(parity)    ((0u != (parity)) ? \
                                                               (PSOC5_UART_TX_CTRL_PARITY) : (0u))

#define PSOC5_GET_UART_TX_CTRL_RETRY_NACK(nack)  ((0u != (nack)) ? \
                                                               (PSOC5_UART_TX_CTRL_RETRY_ON_NACK) : (0u))

/* PSOC5_RX_CTRL */
#define PSOC5_GET_RX_CTRL_DATA_WIDTH(dataWidth)  (((uint32) (dataWidth) - 1u) & \
                                                                PSOC5_RX_CTRL_DATA_WIDTH_MASK)

#define PSOC5_GET_RX_CTRL_BIT_ORDER(bitOrder)    ((0u != (bitOrder)) ? \
                                                                (PSOC5_RX_CTRL_MSB_FIRST) : (0u))

#define PSOC5_GET_RX_CTRL_MEDIAN(filterEn)       ((0u != (filterEn)) ? \
                                                                (PSOC5_RX_CTRL_MEDIAN) : (0u))

/* PSOC5_RX_MATCH */
#define PSOC5_GET_RX_MATCH_ADDR(addr)    ((uint32) (addr) & PSOC5_RX_MATCH_ADDR_MASK)
#define PSOC5_GET_RX_MATCH_MASK(mask)    (((uint32) ((uint32) (mask) << \
                                                            PSOC5_RX_MATCH_MASK_POS)) & \
                                                            PSOC5_RX_MATCH_MASK_MASK)

/* PSOC5_RX_FIFO_CTRL */
#define PSOC5_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(level)  ((uint32) (level) & \
                                                                    PSOC5_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK)

/* PSOC5_TX_CTRL */
#define PSOC5_GET_TX_CTRL_DATA_WIDTH(dataWidth)  (((uint32) (dataWidth) - 1u) & \
                                                                PSOC5_RX_CTRL_DATA_WIDTH_MASK)

#define PSOC5_GET_TX_CTRL_BIT_ORDER(bitOrder)    ((0u != (bitOrder)) ? \
                                                                (PSOC5_TX_CTRL_MSB_FIRST) : (0u))

/* PSOC5_TX_FIFO_CTRL */
#define PSOC5_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(level)  ((uint32) (level) & \
                                                                    PSOC5_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK)

/* Clears register: config and interrupt mask */
#define PSOC5_CLEAR_REG          ((uint32) (0u))
#define PSOC5_NO_INTR_SOURCES    ((uint32) (0u))
#define PSOC5_DUMMY_PARAM        ((uint32) (0u))
#define PSOC5_SUBMODE_SPI_SLAVE  ((uint32) (0u))

/* Return in case I2C read error */
#define PSOC5_I2C_INVALID_BYTE   ((uint32) 0xFFFFFFFFu)
#define PSOC5_CHECK_VALID_BYTE   ((uint32) 0x80000000u)

#endif /* (CY_SCB_PSOC5_H) */


/* [] END OF FILE */
