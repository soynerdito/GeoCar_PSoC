/*******************************************************************************
* File Name: BLUE_PINS.h
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

#if !defined(CY_SCB_PINS_BLUE_H)
#define CY_SCB_PINS_BLUE_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define BLUE_REMOVE_MOSI_SCL_RX_WAKE_PIN    (1u)
#define BLUE_REMOVE_MOSI_SCL_RX_PIN         (1u)
#define BLUE_REMOVE_MISO_SDA_TX_PIN         (1u)
#define BLUE_REMOVE_SCLK_PIN                (1u)
#define BLUE_REMOVE_SS0_PIN                 (1u)
#define BLUE_REMOVE_SS1_PIN                 (1u)
#define BLUE_REMOVE_SS2_PIN                 (1u)
#define BLUE_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define BLUE_REMOVE_I2C_PINS                (1u)
#define BLUE_REMOVE_SPI_MASTER_PINS         (1u)
#define BLUE_REMOVE_SPI_SLAVE_PINS          (1u)
#define BLUE_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define BLUE_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define BLUE_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define BLUE_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define BLUE_REMOVE_UART_TX_PIN             (0u)
#define BLUE_REMOVE_UART_RX_TX_PIN          (1u)
#define BLUE_REMOVE_UART_RX_PIN             (0u)
#define BLUE_REMOVE_UART_RX_WAKE_PIN        (1u)

/* Unconfigured pins */
#define BLUE_MOSI_SCL_RX_WAKE_PIN   (0u == BLUE_REMOVE_MOSI_SCL_RX_WAKE_PIN)
#define BLUE_MOSI_SCL_RX_PIN        (0u == BLUE_REMOVE_MOSI_SCL_RX_PIN)
#define BLUE_MISO_SDA_TX_PIN        (0u == BLUE_REMOVE_MISO_SDA_TX_PIN)
#define BLUE_SCLK_PIN               (0u == BLUE_REMOVE_SCLK_PIN)
#define BLUE_SS0_PIN                (0u == BLUE_REMOVE_SS0_PIN)
#define BLUE_SS1_PIN                (0u == BLUE_REMOVE_SS1_PIN)
#define BLUE_SS2_PIN                (0u == BLUE_REMOVE_SS2_PIN)
#define BLUE_SS3_PIN                (0u == BLUE_REMOVE_SS3_PIN)

/* Mode defined pins */
#define BLUE_I2C_PINS               (0u == BLUE_REMOVE_I2C_PINS)
#define BLUE_SPI_MASTER_PINS        (0u == BLUE_REMOVE_SPI_MASTER_PINS) 
#define BLUE_SPI_SLAVE_PINS         (0u == BLUE_REMOVE_SPI_SLAVE_PINS)
#define BLUE_SPI_MASTER_SS0_PIN     (0u == BLUE_REMOVE_SPI_MASTER_SS0_PIN)
#define BLUE_SPI_MASTER_SS1_PIN     (0u == BLUE_REMOVE_SPI_MASTER_SS1_PIN)
#define BLUE_SPI_MASTER_SS2_PIN     (0u == BLUE_REMOVE_SPI_MASTER_SS2_PIN)
#define BLUE_SPI_MASTER_SS3_PIN     (0u == BLUE_REMOVE_SPI_MASTER_SS3_PIN)
#define BLUE_UART_TX_PIN            (0u == BLUE_REMOVE_UART_TX_PIN)
#define BLUE_UART_RX_TX_PIN         (0u == BLUE_REMOVE_UART_RX_TX_PIN)
#define BLUE_UART_RX_PIN            (0u == BLUE_REMOVE_UART_RX_PIN)
#define BLUE_UART_RX_WAKE_PIN       (0u == BLUE_REMOVE_UART_RX_WAKE_PIN)


/***************************************
*             Includes
****************************************/

#if(BLUE_MOSI_SCL_RX_WAKE_PIN)
    #include "BLUE_spi_mosi_i2c_scl_uart_rx_wake.h"
#endif /* (BLUE_MOSI_SCL_RX_WAKE_PIN) */

#if(BLUE_MOSI_SCL_RX_PIN)
    #include "BLUE_spi_mosi_i2c_scl_uart_rx.h"
#endif /* (BLUE_MOSI_SCL_RX_PIN) */

#if(BLUE_MISO_SDA_TX_PIN)
    #include "BLUE_spi_miso_i2c_sda_uart_tx.h"
#endif /* (BLUE_MISO_SDA_TX_PIN_PIN) */

#if(BLUE_SCLK_PIN)
    #include "BLUE_spi_sclk.h"
#endif /* (BLUE_SCLK_PIN) */

#if(BLUE_SS0_PIN)
    #include "BLUE_spi_ss0.h"
#endif /* (BLUE_SS1_PIN) */

#if(BLUE_SS1_PIN)
    #include "BLUE_spi_ss1.h"
#endif /* (BLUE_SS1_PIN) */

#if(BLUE_SS2_PIN)
    #include "BLUE_spi_ss2.h"
#endif /* (BLUE_SS2_PIN) */

#if(BLUE_SS3_PIN)
    #include "BLUE_spi_ss3.h"
#endif /* (BLUE_SS3_PIN) */

#if(BLUE_I2C_PINS)
    #include "BLUE_scl.h"
    #include "BLUE_sda.h"
#endif /* (BLUE_I2C_PINS) */
    
#if(BLUE_SPI_MASTER_PINS)
    #include "BLUE_sclk_m.h"
    #include "BLUE_mosi_m.h"
    #include "BLUE_miso_m.h"
#endif /* (BLUE_SPI_MASTER_PINS) */

#if(BLUE_SPI_SLAVE_PINS)
    #include "BLUE_sclk_s.h"
    #include "BLUE_mosi_s.h"
    #include "BLUE_miso_s.h"
    #include "BLUE_ss_s.h"
#endif /* (BLUE_SPI_SLAVE_PINS) */

#if(BLUE_SPI_MASTER_SS0_PIN)
    #include "BLUE_ss0_m.h"
#endif /* (BLUE_SPI_MASTER_SS0_PIN) */

#if(BLUE_SPI_MASTER_SS1_PIN)
    #include "BLUE_ss1_m.h"
#endif /* (BLUE_SPI_MASTER_SS1_PIN) */

#if(BLUE_SPI_MASTER_SS2_PIN)
    #include "BLUE_ss2_m.h"
#endif /* (BLUE_SPI_MASTER_SS2_PIN) */

#if(BLUE_SPI_MASTER_SS3_PIN)
    #include "BLUE_ss3_m.h"
#endif /* (BLUE_SPI_MASTER_SS3_PIN) */

#if(BLUE_UART_TX_PIN)
    #include "BLUE_tx.h"
#endif /* (BLUE_UART_TX_PIN) */

#if(BLUE_UART_RX_TX_PIN)
    #include "BLUE_rx_tx.h"
#endif /* (BLUE_UART_RX_TX_PIN) */

#if(BLUE_UART_RX_PIN)
    #include "BLUE_rx.h"
#endif /* (BLUE_UART_RX_PIN) */

#if(BLUE_UART_RX_WAKE_PIN)
    #include "BLUE_rx_wake.h"
#endif /* (BLUE_UART_RX_WAKE_PIN) */


/***************************************
*              Registers
***************************************/

#if(BLUE_MOSI_SCL_RX_WAKE_PIN)
    #define BLUE_MOSI_SCL_RX_WAKE_HSIOM_REG  \
                                                (*(reg32 *) BLUE_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define BLUE_MOSI_SCL_RX_WAKE_HSIOM_PTR  \
                                                ( (reg32 *) BLUE_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define BLUE_MOSI_SCL_RX_WAKE_HSIOM_MASK \
                                                (BLUE_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_MASK)
    #define BLUE_MOSI_SCL_RX_WAKE_HSIOM_POS  \
                                                (BLUE_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_SHIFT)

    #define BLUE_MOSI_SCL_RX_WAKE_INTCFG_REG    (*(reg32 *) \
                                                              BLUE_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)
    #define BLUE_MOSI_SCL_RX_WAKE_INTCFG_PTR    ( (reg32 *) \
                                                              BLUE_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)

    #define BLUE_INTCFG_TYPE_MASK                  (0x03u)
    #define BLUE_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS  (BLUE_spi_mosi_i2c_scl_uart_rx_wake__SHIFT)
    #define BLUE_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK ((uint32)                                           \
                                                                    ((uint32) BLUE_INTCFG_TYPE_MASK << \
                                                                    BLUE_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS))
#endif /* (BLUE_MOSI_SCL_RX_WAKE_PIN) */

#if(BLUE_MOSI_SCL_RX_PIN)
    #define BLUE_MOSI_SCL_RX_HSIOM_REG      (*(reg32 *) BLUE_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define BLUE_MOSI_SCL_RX_HSIOM_PTR      ( (reg32 *) BLUE_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define BLUE_MOSI_SCL_RX_HSIOM_MASK     (BLUE_spi_mosi_i2c_scl_uart_rx__0__HSIOM_MASK)
    #define BLUE_MOSI_SCL_RX_HSIOM_POS      (BLUE_spi_mosi_i2c_scl_uart_rx__0__HSIOM_SHIFT)
#endif /* (BLUE_MOSI_SCL_RX_PIN) */

#if(BLUE_MISO_SDA_TX_PIN)
    #define BLUE_MISO_SDA_TX_HSIOM_REG      (*(reg32 *) BLUE_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define BLUE_MISO_SDA_TX_HSIOM_PTR      ( (reg32 *) BLUE_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define BLUE_MISO_SDA_TX_HSIOM_MASK     (BLUE_spi_miso_i2c_sda_uart_tx__0__HSIOM_MASK)
    #define BLUE_MISO_SDA_TX_HSIOM_POS      (BLUE_spi_miso_i2c_sda_uart_tx__0__HSIOM_SHIFT)
#endif /* (BLUE_MISO_SDA_TX_PIN_PIN) */

#if(BLUE_SCLK_PIN)
    #define BLUE_SCLK_HSIOM_REG     (*(reg32 *) BLUE_spi_sclk__0__HSIOM)
    #define BLUE_SCLK_HSIOM_PTR     ( (reg32 *) BLUE_spi_sclk__0__HSIOM)
    #define BLUE_SCLK_HSIOM_MASK    (BLUE_spi_sclk__0__HSIOM_MASK)
    #define BLUE_SCLK_HSIOM_POS     (BLUE_spi_sclk__0__HSIOM_SHIFT)
#endif /* (BLUE_SCLK_PIN) */

#if(BLUE_SS0_PIN)
    #define BLUE_SS0_HSIOM_REG      (*(reg32 *) BLUE_spi_ss0__0__HSIOM)
    #define BLUE_SS0_HSIOM_PTR      ( (reg32 *) BLUE_spi_ss0__0__HSIOM)
    #define BLUE_SS0_HSIOM_MASK     (BLUE_spi_ss0__0__HSIOM_MASK)
    #define BLUE_SS0_HSIOM_POS      (BLUE_spi_ss0__0__HSIOM_SHIFT)
#endif /* (BLUE_SS1_PIN) */

#if(BLUE_SS1_PIN)
    #define BLUE_SS1_HSIOM_REG      (*(reg32 *) BLUE_spi_ss1__0__HSIOM)
    #define BLUE_SS1_HSIOM_PTR      ( (reg32 *) BLUE_spi_ss1__0__HSIOM)
    #define BLUE_SS1_HSIOM_MASK     (BLUE_spi_ss1__0__HSIOM_MASK)
    #define BLUE_SS1_HSIOM_POS      (BLUE_spi_ss1__0__HSIOM_SHIFT)
#endif /* (BLUE_SS1_PIN) */

#if(BLUE_SS2_PIN)
    #define BLUE_SS2_HSIOM_REG     (*(reg32 *) BLUE_spi_ss2__0__HSIOM)
    #define BLUE_SS2_HSIOM_PTR     ( (reg32 *) BLUE_spi_ss2__0__HSIOM)
    #define BLUE_SS2_HSIOM_MASK    (BLUE_spi_ss2__0__HSIOM_MASK)
    #define BLUE_SS2_HSIOM_POS     (BLUE_spi_ss2__0__HSIOM_SHIFT)
#endif /* (BLUE_SS2_PIN) */

#if(BLUE_SS3_PIN)
    #define BLUE_SS3_HSIOM_REG     (*(reg32 *) BLUE_spi_ss3__0__HSIOM)
    #define BLUE_SS3_HSIOM_PTR     ( (reg32 *) BLUE_spi_ss3__0__HSIOM)
    #define BLUE_SS3_HSIOM_MASK    (BLUE_spi_ss3__0__HSIOM_MASK)
    #define BLUE_SS3_HSIOM_POS     (BLUE_spi_ss3__0__HSIOM_SHIFT)
#endif /* (BLUE_SS3_PIN) */

#if(BLUE_I2C_PINS)
    #define BLUE_SCL_HSIOM_REG     (*(reg32 *) BLUE_scl__0__HSIOM)
    #define BLUE_SCL_HSIOM_PTR     ( (reg32 *) BLUE_scl__0__HSIOM)
    #define BLUE_SCL_HSIOM_MASK    (BLUE_scl__0__HSIOM_MASK)
    #define BLUE_SCL_HSIOM_POS     (BLUE_scl__0__HSIOM_SHIFT)

    #define BLUE_SDA_HSIOM_REG     (*(reg32 *) BLUE_sda__0__HSIOM)
    #define BLUE_SDA_HSIOM_PTR     ( (reg32 *) BLUE_sda__0__HSIOM)
    #define BLUE_SDA_HSIOM_MASK    (BLUE_sda__0__HSIOM_MASK)
    #define BLUE_SDA_HSIOM_POS     (BLUE_sda__0__HSIOM_SHIFT)
#endif /* (BLUE_I2C_PINS) */


/***************************************
*        Registers Constants
***************************************/

/* Pins constanst */
#define BLUE_HSIOM_DEF_SEL      (0x00u)
#define BLUE_HSIOM_GPIO_SEL     (0x00u)
#define BLUE_HSIOM_UART_SEL     (0x09u)
#define BLUE_HSIOM_I2C_SEL      (0x0Eu)
#define BLUE_HSIOM_SPI_SEL      (0x0Fu)

#define BLUE_SCB_PINS_NUMBER            (7u)
#define BLUE_MOSI_SCL_RX_PIN_INDEX      (0u) /* RX pins without interrupt */
#define BLUE_MOSI_SCL_RX_WAKE_PIN_INDEX (0u) /* RX pin with interrupt     */
#define BLUE_MISO_SDA_TX_PIN_INDEX      (1u)
#define BLUE_SCLK_PIN_INDEX             (2u)
#define BLUE_SS0_PIN_INDEX              (3u)
#define BLUE_SS1_PIN_INDEX              (4u)
#define BLUE_SS2_PIN_INDEX              (5u)
#define BLUE_SS3_PIN_INDEX              (6u)

#define BLUE_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin DM defines */
#define BLUE_PIN_DM_ALG_HIZ  (0u)
#define BLUE_PIN_DM_DIG_HIZ  (1u)
#define BLUE_PIN_DM_OD_LO    (4u)
#define BLUE_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

#define BLUE_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        /* Sets new bits-mask */                 \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

#define BLUE_SET_HSIOM_SEL(reg, mask, pos, sel) BLUE_SET_REGISTER_BITS(reg, mask, pos, sel)
#define BLUE_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        BLUE_SET_REGISTER_BITS(reg, mask, pos, intType)


/* BLUE_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  BLUE_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* Unconfigured SCB: scl singnal */
#if(BLUE_MOSI_SCL_RX_WAKE_PIN)
    #define BLUE_SET_I2C_SCL_DR(val) \
                            BLUE_spi_mosi_i2c_scl_uart_rx_wake_Write(val)

    #define BLUE_SET_I2C_SCL_HSIOM_SEL(sel) \
                    BLUE_SET_HSIOM_SEL(BLUE_MOSI_SCL_RX_WAKE_HSIOM_REG,  \
                                                   BLUE_MOSI_SCL_RX_WAKE_HSIOM_MASK, \
                                                   BLUE_MOSI_SCL_RX_WAKE_HSIOM_POS,  \
                                                   (sel))

#elif(BLUE_MOSI_SCL_RX_PIN)
    #define BLUE_SET_I2C_SCL_DR(val) \
                            BLUE_spi_mosi_i2c_scl_uart_rx_Write(val)


    #define BLUE_SET_I2C_SCL_HSIOM_SEL(sel) \
                            BLUE_SET_HSIOM_SEL(BLUE_MOSI_SCL_RX_HSIOM_REG,  \
                                                           BLUE_MOSI_SCL_RX_HSIOM_MASK, \
                                                           BLUE_MOSI_SCL_RX_HSIOM_POS,  \
                                                           (sel))
#else
    #if(!BLUE_I2C_PINS)
        #define BLUE_SET_I2C_SCL_DR(val) \
                                                 do{ ; }while(0)
        #define BLUE_SET_I2C_SCL_HSIOM_SEL(sel) \
                                                        do{ ; }while(0)

    #endif /* (!BLUE_I2C_PINS) */
#endif /* (BLUE_MOSI_SCL_RX_PIN) */

/* SCB I2C: scl singal */
#if(BLUE_I2C_PINS)
    #define BLUE_SET_I2C_SCL_DR(val) BLUE_scl_Write(val)

    #define BLUE_SET_I2C_SCL_HSIOM_SEL(sel) \
                          BLUE_SET_HSIOM_SEL(BLUE_SCL_HSIOM_REG,  \
                                                         BLUE_SCL_HSIOM_MASK, \
                                                         BLUE_SCL_HSIOM_POS,  \
                                                         (sel))

#endif /* (BLUE_I2C_PINS) */


#endif /* (CY_SCB_PINS_BLUE_H) */


/* [] END OF FILE */
