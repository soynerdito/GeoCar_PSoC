/*******************************************************************************
* File Name: BLUE_PM.c
* Version 2.30
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BLUE.h"


/***************************************
* Local data allocation
***************************************/

static BLUE_BACKUP_STRUCT  BLUE_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: BLUE_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  BLUE_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void BLUE_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(BLUE_CONTROL_REG_REMOVED == 0u)
            BLUE_backup.cr = BLUE_CONTROL_REG;
        #endif /* End BLUE_CONTROL_REG_REMOVED */

        #if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
            BLUE_backup.rx_period = BLUE_RXBITCTR_PERIOD_REG;
            BLUE_backup.rx_mask = BLUE_RXSTATUS_MASK_REG;
            #if (BLUE_RXHW_ADDRESS_ENABLED)
                BLUE_backup.rx_addr1 = BLUE_RXADDRESS1_REG;
                BLUE_backup.rx_addr2 = BLUE_RXADDRESS2_REG;
            #endif /* End BLUE_RXHW_ADDRESS_ENABLED */
        #endif /* End BLUE_RX_ENABLED | BLUE_HD_ENABLED*/

        #if(BLUE_TX_ENABLED)
            #if(BLUE_TXCLKGEN_DP)
                BLUE_backup.tx_clk_ctr = BLUE_TXBITCLKGEN_CTR_REG;
                BLUE_backup.tx_clk_compl = BLUE_TXBITCLKTX_COMPLETE_REG;
            #else
                BLUE_backup.tx_period = BLUE_TXBITCTR_PERIOD_REG;
            #endif /*End BLUE_TXCLKGEN_DP */
            BLUE_backup.tx_mask = BLUE_TXSTATUS_MASK_REG;
        #endif /*End BLUE_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(BLUE_CONTROL_REG_REMOVED == 0u)
            BLUE_backup.cr = BLUE_CONTROL_REG;
        #endif /* End BLUE_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: BLUE_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  BLUE_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void BLUE_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(BLUE_CONTROL_REG_REMOVED == 0u)
            BLUE_CONTROL_REG = BLUE_backup.cr;
        #endif /* End BLUE_CONTROL_REG_REMOVED */

        #if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
            BLUE_RXBITCTR_PERIOD_REG = BLUE_backup.rx_period;
            BLUE_RXSTATUS_MASK_REG = BLUE_backup.rx_mask;
            #if (BLUE_RXHW_ADDRESS_ENABLED)
                BLUE_RXADDRESS1_REG = BLUE_backup.rx_addr1;
                BLUE_RXADDRESS2_REG = BLUE_backup.rx_addr2;
            #endif /* End BLUE_RXHW_ADDRESS_ENABLED */
        #endif  /* End (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) */

        #if(BLUE_TX_ENABLED)
            #if(BLUE_TXCLKGEN_DP)
                BLUE_TXBITCLKGEN_CTR_REG = BLUE_backup.tx_clk_ctr;
                BLUE_TXBITCLKTX_COMPLETE_REG = BLUE_backup.tx_clk_compl;
            #else
                BLUE_TXBITCTR_PERIOD_REG = BLUE_backup.tx_period;
            #endif /*End BLUE_TXCLKGEN_DP */
            BLUE_TXSTATUS_MASK_REG = BLUE_backup.tx_mask;
        #endif /*End BLUE_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(BLUE_CONTROL_REG_REMOVED == 0u)
            BLUE_CONTROL_REG = BLUE_backup.cr;
        #endif /* End BLUE_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: BLUE_Sleep
********************************************************************************
*
* Summary:
*  Stops and saves the user configuration. Should be called
*  just prior to entering sleep.
*
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  BLUE_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void BLUE_Sleep(void)
{

    #if(BLUE_RX_ENABLED || BLUE_HD_ENABLED)
        if((BLUE_RXSTATUS_ACTL_REG  & BLUE_INT_ENABLE) != 0u)
        {
            BLUE_backup.enableState = 1u;
        }
        else
        {
            BLUE_backup.enableState = 0u;
        }
    #else
        if((BLUE_TXSTATUS_ACTL_REG  & BLUE_INT_ENABLE) !=0u)
        {
            BLUE_backup.enableState = 1u;
        }
        else
        {
            BLUE_backup.enableState = 0u;
        }
    #endif /* End BLUE_RX_ENABLED || BLUE_HD_ENABLED*/

    BLUE_Stop();
    BLUE_SaveConfig();
}


/*******************************************************************************
* Function Name: BLUE_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration. Should be called
*  just after awaking from sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  BLUE_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void BLUE_Wakeup(void)
{
    BLUE_RestoreConfig();
    #if( (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) )
        BLUE_ClearRxBuffer();
    #endif /* End (BLUE_RX_ENABLED) || (BLUE_HD_ENABLED) */
    #if(BLUE_TX_ENABLED || BLUE_HD_ENABLED)
        BLUE_ClearTxBuffer();
    #endif /* End BLUE_TX_ENABLED || BLUE_HD_ENABLED */

    if(BLUE_backup.enableState != 0u)
    {
        BLUE_Enable();
    }
}


/* [] END OF FILE */
