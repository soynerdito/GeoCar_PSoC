/*******************************************************************************
* File Name: PSOC4_PM.c
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

#include "PSOC4.h"


/***************************************
* Local data allocation
***************************************/

static PSOC4_BACKUP_STRUCT  PSOC4_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: PSOC4_SaveConfig
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
*  PSOC4_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PSOC4_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(PSOC4_CONTROL_REG_REMOVED == 0u)
            PSOC4_backup.cr = PSOC4_CONTROL_REG;
        #endif /* End PSOC4_CONTROL_REG_REMOVED */

        #if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
            PSOC4_backup.rx_period = PSOC4_RXBITCTR_PERIOD_REG;
            PSOC4_backup.rx_mask = PSOC4_RXSTATUS_MASK_REG;
            #if (PSOC4_RXHW_ADDRESS_ENABLED)
                PSOC4_backup.rx_addr1 = PSOC4_RXADDRESS1_REG;
                PSOC4_backup.rx_addr2 = PSOC4_RXADDRESS2_REG;
            #endif /* End PSOC4_RXHW_ADDRESS_ENABLED */
        #endif /* End PSOC4_RX_ENABLED | PSOC4_HD_ENABLED*/

        #if(PSOC4_TX_ENABLED)
            #if(PSOC4_TXCLKGEN_DP)
                PSOC4_backup.tx_clk_ctr = PSOC4_TXBITCLKGEN_CTR_REG;
                PSOC4_backup.tx_clk_compl = PSOC4_TXBITCLKTX_COMPLETE_REG;
            #else
                PSOC4_backup.tx_period = PSOC4_TXBITCTR_PERIOD_REG;
            #endif /*End PSOC4_TXCLKGEN_DP */
            PSOC4_backup.tx_mask = PSOC4_TXSTATUS_MASK_REG;
        #endif /*End PSOC4_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(PSOC4_CONTROL_REG_REMOVED == 0u)
            PSOC4_backup.cr = PSOC4_CONTROL_REG;
        #endif /* End PSOC4_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: PSOC4_RestoreConfig
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
*  PSOC4_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PSOC4_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(PSOC4_CONTROL_REG_REMOVED == 0u)
            PSOC4_CONTROL_REG = PSOC4_backup.cr;
        #endif /* End PSOC4_CONTROL_REG_REMOVED */

        #if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
            PSOC4_RXBITCTR_PERIOD_REG = PSOC4_backup.rx_period;
            PSOC4_RXSTATUS_MASK_REG = PSOC4_backup.rx_mask;
            #if (PSOC4_RXHW_ADDRESS_ENABLED)
                PSOC4_RXADDRESS1_REG = PSOC4_backup.rx_addr1;
                PSOC4_RXADDRESS2_REG = PSOC4_backup.rx_addr2;
            #endif /* End PSOC4_RXHW_ADDRESS_ENABLED */
        #endif  /* End (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) */

        #if(PSOC4_TX_ENABLED)
            #if(PSOC4_TXCLKGEN_DP)
                PSOC4_TXBITCLKGEN_CTR_REG = PSOC4_backup.tx_clk_ctr;
                PSOC4_TXBITCLKTX_COMPLETE_REG = PSOC4_backup.tx_clk_compl;
            #else
                PSOC4_TXBITCTR_PERIOD_REG = PSOC4_backup.tx_period;
            #endif /*End PSOC4_TXCLKGEN_DP */
            PSOC4_TXSTATUS_MASK_REG = PSOC4_backup.tx_mask;
        #endif /*End PSOC4_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(PSOC4_CONTROL_REG_REMOVED == 0u)
            PSOC4_CONTROL_REG = PSOC4_backup.cr;
        #endif /* End PSOC4_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: PSOC4_Sleep
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
*  PSOC4_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PSOC4_Sleep(void)
{

    #if(PSOC4_RX_ENABLED || PSOC4_HD_ENABLED)
        if((PSOC4_RXSTATUS_ACTL_REG  & PSOC4_INT_ENABLE) != 0u)
        {
            PSOC4_backup.enableState = 1u;
        }
        else
        {
            PSOC4_backup.enableState = 0u;
        }
    #else
        if((PSOC4_TXSTATUS_ACTL_REG  & PSOC4_INT_ENABLE) !=0u)
        {
            PSOC4_backup.enableState = 1u;
        }
        else
        {
            PSOC4_backup.enableState = 0u;
        }
    #endif /* End PSOC4_RX_ENABLED || PSOC4_HD_ENABLED*/

    PSOC4_Stop();
    PSOC4_SaveConfig();
}


/*******************************************************************************
* Function Name: PSOC4_Wakeup
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
*  PSOC4_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PSOC4_Wakeup(void)
{
    PSOC4_RestoreConfig();
    #if( (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) )
        PSOC4_ClearRxBuffer();
    #endif /* End (PSOC4_RX_ENABLED) || (PSOC4_HD_ENABLED) */
    #if(PSOC4_TX_ENABLED || PSOC4_HD_ENABLED)
        PSOC4_ClearTxBuffer();
    #endif /* End PSOC4_TX_ENABLED || PSOC4_HD_ENABLED */

    if(PSOC4_backup.enableState != 0u)
    {
        PSOC4_Enable();
    }
}


/* [] END OF FILE */
