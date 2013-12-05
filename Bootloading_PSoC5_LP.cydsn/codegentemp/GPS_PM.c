/*******************************************************************************
* File Name: GPS_PM.c
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

#include "GPS.h"


/***************************************
* Local data allocation
***************************************/

static GPS_BACKUP_STRUCT  GPS_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: GPS_SaveConfig
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
*  GPS_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void GPS_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(GPS_CONTROL_REG_REMOVED == 0u)
            GPS_backup.cr = GPS_CONTROL_REG;
        #endif /* End GPS_CONTROL_REG_REMOVED */

        #if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
            GPS_backup.rx_period = GPS_RXBITCTR_PERIOD_REG;
            GPS_backup.rx_mask = GPS_RXSTATUS_MASK_REG;
            #if (GPS_RXHW_ADDRESS_ENABLED)
                GPS_backup.rx_addr1 = GPS_RXADDRESS1_REG;
                GPS_backup.rx_addr2 = GPS_RXADDRESS2_REG;
            #endif /* End GPS_RXHW_ADDRESS_ENABLED */
        #endif /* End GPS_RX_ENABLED | GPS_HD_ENABLED*/

        #if(GPS_TX_ENABLED)
            #if(GPS_TXCLKGEN_DP)
                GPS_backup.tx_clk_ctr = GPS_TXBITCLKGEN_CTR_REG;
                GPS_backup.tx_clk_compl = GPS_TXBITCLKTX_COMPLETE_REG;
            #else
                GPS_backup.tx_period = GPS_TXBITCTR_PERIOD_REG;
            #endif /*End GPS_TXCLKGEN_DP */
            GPS_backup.tx_mask = GPS_TXSTATUS_MASK_REG;
        #endif /*End GPS_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(GPS_CONTROL_REG_REMOVED == 0u)
            GPS_backup.cr = GPS_CONTROL_REG;
        #endif /* End GPS_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: GPS_RestoreConfig
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
*  GPS_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void GPS_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(GPS_CONTROL_REG_REMOVED == 0u)
            GPS_CONTROL_REG = GPS_backup.cr;
        #endif /* End GPS_CONTROL_REG_REMOVED */

        #if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
            GPS_RXBITCTR_PERIOD_REG = GPS_backup.rx_period;
            GPS_RXSTATUS_MASK_REG = GPS_backup.rx_mask;
            #if (GPS_RXHW_ADDRESS_ENABLED)
                GPS_RXADDRESS1_REG = GPS_backup.rx_addr1;
                GPS_RXADDRESS2_REG = GPS_backup.rx_addr2;
            #endif /* End GPS_RXHW_ADDRESS_ENABLED */
        #endif  /* End (GPS_RX_ENABLED) || (GPS_HD_ENABLED) */

        #if(GPS_TX_ENABLED)
            #if(GPS_TXCLKGEN_DP)
                GPS_TXBITCLKGEN_CTR_REG = GPS_backup.tx_clk_ctr;
                GPS_TXBITCLKTX_COMPLETE_REG = GPS_backup.tx_clk_compl;
            #else
                GPS_TXBITCTR_PERIOD_REG = GPS_backup.tx_period;
            #endif /*End GPS_TXCLKGEN_DP */
            GPS_TXSTATUS_MASK_REG = GPS_backup.tx_mask;
        #endif /*End GPS_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(GPS_CONTROL_REG_REMOVED == 0u)
            GPS_CONTROL_REG = GPS_backup.cr;
        #endif /* End GPS_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: GPS_Sleep
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
*  GPS_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void GPS_Sleep(void)
{

    #if(GPS_RX_ENABLED || GPS_HD_ENABLED)
        if((GPS_RXSTATUS_ACTL_REG  & GPS_INT_ENABLE) != 0u)
        {
            GPS_backup.enableState = 1u;
        }
        else
        {
            GPS_backup.enableState = 0u;
        }
    #else
        if((GPS_TXSTATUS_ACTL_REG  & GPS_INT_ENABLE) !=0u)
        {
            GPS_backup.enableState = 1u;
        }
        else
        {
            GPS_backup.enableState = 0u;
        }
    #endif /* End GPS_RX_ENABLED || GPS_HD_ENABLED*/

    GPS_Stop();
    GPS_SaveConfig();
}


/*******************************************************************************
* Function Name: GPS_Wakeup
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
*  GPS_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void GPS_Wakeup(void)
{
    GPS_RestoreConfig();
    #if( (GPS_RX_ENABLED) || (GPS_HD_ENABLED) )
        GPS_ClearRxBuffer();
    #endif /* End (GPS_RX_ENABLED) || (GPS_HD_ENABLED) */
    #if(GPS_TX_ENABLED || GPS_HD_ENABLED)
        GPS_ClearTxBuffer();
    #endif /* End GPS_TX_ENABLED || GPS_HD_ENABLED */

    if(GPS_backup.enableState != 0u)
    {
        GPS_Enable();
    }
}


/* [] END OF FILE */
