/*******************************************************************************
* File Name: PSOC5_PINS.h
* Version 1.10
*
* Description:
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PINS_PSOC5_H)
#define CY_SCB_PINS_PSOC5_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define PSOC5_REMOVE_MOSI_SCL_RX_WAKE_PIN    (1u)
#define PSOC5_REMOVE_MOSI_SCL_RX_PIN         (1u)
#define PSOC5_REMOVE_MISO_SDA_TX_PIN         (1u)
#define PSOC5_REMOVE_SCLK_PIN                (1u)
#define PSOC5_REMOVE_SS0_PIN                 (1u)
#define PSOC5_REMOVE_SS1_PIN                 (1u)
#define PSOC5_REMOVE_SS2_PIN                 (1u)
#define PSOC5_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define PSOC5_REMOVE_I2C_PINS                (1u)
#define PSOC5_REMOVE_SPI_MASTER_PINS         (1u)
#define PSOC5_REMOVE_SPI_SLAVE_PINS          (1u)
#define PSOC5_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define PSOC5_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define PSOC5_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define PSOC5_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define PSOC5_REMOVE_UART_TX_PIN             (0u)
#define PSOC5_REMOVE_UART_RX_TX_PIN          (1u)
#define PSOC5_REMOVE_UART_RX_PIN             (0u)
#define PSOC5_REMOVE_UART_RX_WAKE_PIN        (1u)

/* Unconfigured pins */
#define PSOC5_MOSI_SCL_RX_WAKE_PIN   (0u == PSOC5_REMOVE_MOSI_SCL_RX_WAKE_PIN)
#define PSOC5_MOSI_SCL_RX_PIN        (0u == PSOC5_REMOVE_MOSI_SCL_RX_PIN)
#define PSOC5_MISO_SDA_TX_PIN        (0u == PSOC5_REMOVE_MISO_SDA_TX_PIN)
#define PSOC5_SCLK_PIN               (0u == PSOC5_REMOVE_SCLK_PIN)
#define PSOC5_SS0_PIN                (0u == PSOC5_REMOVE_SS0_PIN)
#define PSOC5_SS1_PIN                (0u == PSOC5_REMOVE_SS1_PIN)
#define PSOC5_SS2_PIN                (0u == PSOC5_REMOVE_SS2_PIN)
#define PSOC5_SS3_PIN                (0u == PSOC5_REMOVE_SS3_PIN)

/* Mode defined pins */
#define PSOC5_I2C_PINS               (0u == PSOC5_REMOVE_I2C_PINS)
#define PSOC5_SPI_MASTER_PINS        (0u == PSOC5_REMOVE_SPI_MASTER_PINS) 
#define PSOC5_SPI_SLAVE_PINS         (0u == PSOC5_REMOVE_SPI_SLAVE_PINS)
#define PSOC5_SPI_MASTER_SS0_PIN     (0u == PSOC5_REMOVE_SPI_MASTER_SS0_PIN)
#define PSOC5_SPI_MASTER_SS1_PIN     (0u == PSOC5_REMOVE_SPI_MASTER_SS1_PIN)
#define PSOC5_SPI_MASTER_SS2_PIN     (0u == PSOC5_REMOVE_SPI_MASTER_SS2_PIN)
#define PSOC5_SPI_MASTER_SS3_PIN     (0u == PSOC5_REMOVE_SPI_MASTER_SS3_PIN)
#define PSOC5_UART_TX_PIN            (0u == PSOC5_REMOVE_UART_TX_PIN)
#define PSOC5_UART_RX_TX_PIN         (0u == PSOC5_REMOVE_UART_RX_TX_PIN)
#define PSOC5_UART_RX_PIN            (0u == PSOC5_REMOVE_UART_RX_PIN)
#define PSOC5_UART_RX_WAKE_PIN       (0u == PSOC5_REMOVE_UART_RX_WAKE_PIN)


/***************************************
*             Includes
****************************************/

#if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
    #include "PSOC5_spi_mosi_i2c_scl_uart_rx_wake.h"
#endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */

#if(PSOC5_MOSI_SCL_RX_PIN)
    #include "PSOC5_spi_mosi_i2c_scl_uart_rx.h"
#endif /* (PSOC5_MOSI_SCL_RX_PIN) */

#if(PSOC5_MISO_SDA_TX_PIN)
    #include "PSOC5_spi_miso_i2c_sda_uart_tx.h"
#endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */

#if(PSOC5_SCLK_PIN)
    #include "PSOC5_spi_sclk.h"
#endif /* (PSOC5_SCLK_PIN) */

#if(PSOC5_SS0_PIN)
    #include "PSOC5_spi_ss0.h"
#endif /* (PSOC5_SS1_PIN) */

#if(PSOC5_SS1_PIN)
    #include "PSOC5_spi_ss1.h"
#endif /* (PSOC5_SS1_PIN) */

#if(PSOC5_SS2_PIN)
    #include "PSOC5_spi_ss2.h"
#endif /* (PSOC5_SS2_PIN) */

#if(PSOC5_SS3_PIN)
    #include "PSOC5_spi_ss3.h"
#endif /* (PSOC5_SS3_PIN) */

#if(PSOC5_I2C_PINS)
    #include "PSOC5_scl.h"
    #include "PSOC5_sda.h"
#endif /* (PSOC5_I2C_PINS) */
    
#if(PSOC5_SPI_MASTER_PINS)
    #include "PSOC5_sclk_m.h"
    #include "PSOC5_mosi_m.h"
    #include "PSOC5_miso_m.h"
#endif /* (PSOC5_SPI_MASTER_PINS) */

#if(PSOC5_SPI_SLAVE_PINS)
    #include "PSOC5_sclk_s.h"
    #include "PSOC5_mosi_s.h"
    #include "PSOC5_miso_s.h"
    #include "PSOC5_ss_s.h"
#endif /* (PSOC5_SPI_SLAVE_PINS) */

#if(PSOC5_SPI_MASTER_SS0_PIN)
    #include "PSOC5_ss0_m.h"
#endif /* (PSOC5_SPI_MASTER_SS0_PIN) */

#if(PSOC5_SPI_MASTER_SS1_PIN)
    #include "PSOC5_ss1_m.h"
#endif /* (PSOC5_SPI_MASTER_SS1_PIN) */

#if(PSOC5_SPI_MASTER_SS2_PIN)
    #include "PSOC5_ss2_m.h"
#endif /* (PSOC5_SPI_MASTER_SS2_PIN) */

#if(PSOC5_SPI_MASTER_SS3_PIN)
    #include "PSOC5_ss3_m.h"
#endif /* (PSOC5_SPI_MASTER_SS3_PIN) */

#if(PSOC5_UART_TX_PIN)
    #include "PSOC5_tx.h"
#endif /* (PSOC5_UART_TX_PIN) */

#if(PSOC5_UART_RX_TX_PIN)
    #include "PSOC5_rx_tx.h"
#endif /* (PSOC5_UART_RX_TX_PIN) */

#if(PSOC5_UART_RX_PIN)
    #include "PSOC5_rx.h"
#endif /* (PSOC5_UART_RX_PIN) */

#if(PSOC5_UART_RX_WAKE_PIN)
    #include "PSOC5_rx_wake.h"
#endif /* (PSOC5_UART_RX_WAKE_PIN) */


/***************************************
*              Registers
***************************************/

#if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
    #define PSOC5_MOSI_SCL_RX_WAKE_HSIOM_REG  \
                                                (*(reg32 *) PSOC5_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define PSOC5_MOSI_SCL_RX_WAKE_HSIOM_PTR  \
                                                ( (reg32 *) PSOC5_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define PSOC5_MOSI_SCL_RX_WAKE_HSIOM_MASK \
                                                (PSOC5_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_MASK)
    #define PSOC5_MOSI_SCL_RX_WAKE_HSIOM_POS  \
                                                (PSOC5_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_SHIFT)

    #define PSOC5_MOSI_SCL_RX_WAKE_INTCFG_REG    (*(reg32 *) \
                                                              PSOC5_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)
    #define PSOC5_MOSI_SCL_RX_WAKE_INTCFG_PTR    ( (reg32 *) \
                                                              PSOC5_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)

    #define PSOC5_INTCFG_TYPE_MASK                  (0x03u)
    #define PSOC5_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS  (PSOC5_spi_mosi_i2c_scl_uart_rx_wake__SHIFT)
    #define PSOC5_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK ((uint32)                                           \
                                                                    ((uint32) PSOC5_INTCFG_TYPE_MASK << \
                                                                    PSOC5_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS))
#endif /* (PSOC5_MOSI_SCL_RX_WAKE_PIN) */

#if(PSOC5_MOSI_SCL_RX_PIN)
    #define PSOC5_MOSI_SCL_RX_HSIOM_REG      (*(reg32 *) PSOC5_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define PSOC5_MOSI_SCL_RX_HSIOM_PTR      ( (reg32 *) PSOC5_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define PSOC5_MOSI_SCL_RX_HSIOM_MASK     (PSOC5_spi_mosi_i2c_scl_uart_rx__0__HSIOM_MASK)
    #define PSOC5_MOSI_SCL_RX_HSIOM_POS      (PSOC5_spi_mosi_i2c_scl_uart_rx__0__HSIOM_SHIFT)
#endif /* (PSOC5_MOSI_SCL_RX_PIN) */

#if(PSOC5_MISO_SDA_TX_PIN)
    #define PSOC5_MISO_SDA_TX_HSIOM_REG      (*(reg32 *) PSOC5_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define PSOC5_MISO_SDA_TX_HSIOM_PTR      ( (reg32 *) PSOC5_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define PSOC5_MISO_SDA_TX_HSIOM_MASK     (PSOC5_spi_miso_i2c_sda_uart_tx__0__HSIOM_MASK)
    #define PSOC5_MISO_SDA_TX_HSIOM_POS      (PSOC5_spi_miso_i2c_sda_uart_tx__0__HSIOM_SHIFT)
#endif /* (PSOC5_MISO_SDA_TX_PIN_PIN) */

#if(PSOC5_SCLK_PIN)
    #define PSOC5_SCLK_HSIOM_REG     (*(reg32 *) PSOC5_spi_sclk__0__HSIOM)
    #define PSOC5_SCLK_HSIOM_PTR     ( (reg32 *) PSOC5_spi_sclk__0__HSIOM)
    #define PSOC5_SCLK_HSIOM_MASK    (PSOC5_spi_sclk__0__HSIOM_MASK)
    #define PSOC5_SCLK_HSIOM_POS     (PSOC5_spi_sclk__0__HSIOM_SHIFT)
#endif /* (PSOC5_SCLK_PIN) */

#if(PSOC5_SS0_PIN)
    #define PSOC5_SS0_HSIOM_REG      (*(reg32 *) PSOC5_spi_ss0__0__HSIOM)
    #define PSOC5_SS0_HSIOM_PTR      ( (reg32 *) PSOC5_spi_ss0__0__HSIOM)
    #define PSOC5_SS0_HSIOM_MASK     (PSOC5_spi_ss0__0__HSIOM_MASK)
    #define PSOC5_SS0_HSIOM_POS      (PSOC5_spi_ss0__0__HSIOM_SHIFT)
#endif /* (PSOC5_SS1_PIN) */

#if(PSOC5_SS1_PIN)
    #define PSOC5_SS1_HSIOM_REG      (*(reg32 *) PSOC5_spi_ss1__0__HSIOM)
    #define PSOC5_SS1_HSIOM_PTR      ( (reg32 *) PSOC5_spi_ss1__0__HSIOM)
    #define PSOC5_SS1_HSIOM_MASK     (PSOC5_spi_ss1__0__HSIOM_MASK)
    #define PSOC5_SS1_HSIOM_POS      (PSOC5_spi_ss1__0__HSIOM_SHIFT)
#endif /* (PSOC5_SS1_PIN) */

#if(PSOC5_SS2_PIN)
    #define PSOC5_SS2_HSIOM_REG     (*(reg32 *) PSOC5_spi_ss2__0__HSIOM)
    #define PSOC5_SS2_HSIOM_PTR     ( (reg32 *) PSOC5_spi_ss2__0__HSIOM)
    #define PSOC5_SS2_HSIOM_MASK    (PSOC5_spi_ss2__0__HSIOM_MASK)
    #define PSOC5_SS2_HSIOM_POS     (PSOC5_spi_ss2__0__HSIOM_SHIFT)
#endif /* (PSOC5_SS2_PIN) */

#if(PSOC5_SS3_PIN)
    #define PSOC5_SS3_HSIOM_REG     (*(reg32 *) PSOC5_spi_ss3__0__HSIOM)
    #define PSOC5_SS3_HSIOM_PTR     ( (reg32 *) PSOC5_spi_ss3__0__HSIOM)
    #define PSOC5_SS3_HSIOM_MASK    (PSOC5_spi_ss3__0__HSIOM_MASK)
    #define PSOC5_SS3_HSIOM_POS     (PSOC5_spi_ss3__0__HSIOM_SHIFT)
#endif /* (PSOC5_SS3_PIN) */

#if(PSOC5_I2C_PINS)
    #define PSOC5_SCL_HSIOM_REG     (*(reg32 *) PSOC5_scl__0__HSIOM)
    #define PSOC5_SCL_HSIOM_PTR     ( (reg32 *) PSOC5_scl__0__HSIOM)
    #define PSOC5_SCL_HSIOM_MASK    (PSOC5_scl__0__HSIOM_MASK)
    #define PSOC5_SCL_HSIOM_POS     (PSOC5_scl__0__HSIOM_SHIFT)

    #define PSOC5_SDA_HSIOM_REG     (*(reg32 *) PSOC5_sda__0__HSIOM)
    #define PSOC5_SDA_HSIOM_PTR     ( (reg32 *) PSOC5_sda__0__HSIOM)
    #define PSOC5_SDA_HSIOM_MASK    (PSOC5_sda__0__HSIOM_MASK)
    #define PSOC5_SDA_HSIOM_POS     (PSOC5_sda__0__HSIOM_SHIFT)
#endif /* (PSOC5_I2C_PINS) */


/***************************************
*        Registers Constants
***************************************/

/* Pins constanst */
#define PSOC5_HSIOM_DEF_SEL      (0x00u)
#define PSOC5_HSIOM_GPIO_SEL     (0x00u)
#define PSOC5_HSIOM_UART_SEL     (0x09u)
#define PSOC5_HSIOM_I2C_SEL      (0x0Eu)
#define PSOC5_HSIOM_SPI_SEL      (0x0Fu)

#define PSOC5_SCB_PINS_NUMBER            (7u)
#define PSOC5_MOSI_SCL_RX_PIN_INDEX      (0u) /* RX pins without interrupt */
#define PSOC5_MOSI_SCL_RX_WAKE_PIN_INDEX (0u) /* RX pin with interrupt     */
#define PSOC5_MISO_SDA_TX_PIN_INDEX      (1u)
#define PSOC5_SCLK_PIN_INDEX             (2u)
#define PSOC5_SS0_PIN_INDEX              (3u)
#define PSOC5_SS1_PIN_INDEX              (4u)
#define PSOC5_SS2_PIN_INDEX              (5u)
#define PSOC5_SS3_PIN_INDEX              (6u)

#define PSOC5_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin DM defines */
#define PSOC5_PIN_DM_ALG_HIZ  (0u)
#define PSOC5_PIN_DM_DIG_HIZ  (1u)
#define PSOC5_PIN_DM_OD_LO    (4u)
#define PSOC5_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

#define PSOC5_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        /* Sets new bits-mask */                 \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

#define PSOC5_SET_HSIOM_SEL(reg, mask, pos, sel) PSOC5_SET_REGISTER_BITS(reg, mask, pos, sel)
#define PSOC5_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        PSOC5_SET_REGISTER_BITS(reg, mask, pos, intType)


/* PSOC5_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  PSOC5_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* Unconfigured SCB: scl singnal */
#if(PSOC5_MOSI_SCL_RX_WAKE_PIN)
    #define PSOC5_SET_I2C_SCL_DR(val) \
                            PSOC5_spi_mosi_i2c_scl_uart_rx_wake_Write(val)

    #define PSOC5_SET_I2C_SCL_HSIOM_SEL(sel) \
                    PSOC5_SET_HSIOM_SEL(PSOC5_MOSI_SCL_RX_WAKE_HSIOM_REG,  \
                                                   PSOC5_MOSI_SCL_RX_WAKE_HSIOM_MASK, \
                                                   PSOC5_MOSI_SCL_RX_WAKE_HSIOM_POS,  \
                                                   (sel))

#elif(PSOC5_MOSI_SCL_RX_PIN)
    #define PSOC5_SET_I2C_SCL_DR(val) \
                            PSOC5_spi_mosi_i2c_scl_uart_rx_Write(val)


    #define PSOC5_SET_I2C_SCL_HSIOM_SEL(sel) \
                            PSOC5_SET_HSIOM_SEL(PSOC5_MOSI_SCL_RX_HSIOM_REG,  \
                                                           PSOC5_MOSI_SCL_RX_HSIOM_MASK, \
                                                           PSOC5_MOSI_SCL_RX_HSIOM_POS,  \
                                                           (sel))
#else
    #if(!PSOC5_I2C_PINS)
        #define PSOC5_SET_I2C_SCL_DR(val) \
                                                 do{ ; }while(0)
        #define PSOC5_SET_I2C_SCL_HSIOM_SEL(sel) \
                                                        do{ ; }while(0)

    #endif /* (!PSOC5_I2C_PINS) */
#endif /* (PSOC5_MOSI_SCL_RX_PIN) */

/* SCB I2C: scl singal */
#if(PSOC5_I2C_PINS)
    #define PSOC5_SET_I2C_SCL_DR(val) PSOC5_scl_Write(val)

    #define PSOC5_SET_I2C_SCL_HSIOM_SEL(sel) \
                          PSOC5_SET_HSIOM_SEL(PSOC5_SCL_HSIOM_REG,  \
                                                         PSOC5_SCL_HSIOM_MASK, \
                                                         PSOC5_SCL_HSIOM_POS,  \
                                                         (sel))

#endif /* (PSOC5_I2C_PINS) */


#endif /* (CY_SCB_PINS_PSOC5_H) */


/* [] END OF FILE */
