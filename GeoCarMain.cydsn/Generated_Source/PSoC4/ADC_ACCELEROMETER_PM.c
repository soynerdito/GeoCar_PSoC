/*******************************************************************************
* File Name: ADC_ACCELEROMETER_PM.c
* Version 1.10
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
********************************************************************************
* Copyright 2008-2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "CyLib.h"
#include "ADC_ACCELEROMETER.h"


/***************************************
* Local data allocation
***************************************/

static ADC_ACCELEROMETER_BACKUP_STRUCT  ADC_ACCELEROMETER_backup =
{
    ADC_ACCELEROMETER_DISABLED
};


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SaveConfig
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
*******************************************************************************/
void ADC_ACCELEROMETER_SaveConfig(void)
{
    /* All configuration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_RestoreConfig
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
*******************************************************************************/
void ADC_ACCELEROMETER_RestoreConfig(void)
{
    /* All congiguration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_Sleep
********************************************************************************
*
* Summary:
*  Stops the ADC operation and saves the configuration registers and component
*  enable state. Should be called just prior to entering sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ADC_ACCELEROMETER_backup - modified.
*
*******************************************************************************/
void ADC_ACCELEROMETER_Sleep(void)
{
    /* During deepsleep/ hibernate mode keep SARMUX active, i.e. do not open
    *   all switches (disconnect), to be used for ADFT
    */
    ADC_ACCELEROMETER_SAR_DFT_CTRL_REG |= ADC_ACCELEROMETER_ADFT_OVERRIDE;
    if((ADC_ACCELEROMETER_SAR_CTRL_REG  & ADC_ACCELEROMETER_ENABLE) != 0u)
    {
        if((ADC_ACCELEROMETER_SAR_SAMPLE_CTRL_REG & ADC_ACCELEROMETER_CONTINUOUS_EN) != 0u)
        {
            ADC_ACCELEROMETER_backup.enableState = ADC_ACCELEROMETER_ENABLED | ADC_ACCELEROMETER_STARTED;
        }
        else
        {
            ADC_ACCELEROMETER_backup.enableState = ADC_ACCELEROMETER_ENABLED;
        }
        ADC_ACCELEROMETER_StopConvert();
        ADC_ACCELEROMETER_Stop();
    }
    else
    {
        ADC_ACCELEROMETER_backup.enableState = ADC_ACCELEROMETER_DISABLED;
    }
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_Wakeup
********************************************************************************
*
* Summary:
*  Restores the component enable state and configuration registers.
*  This should be called just after awaking from sleep mode.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  ADC_ACCELEROMETER_backup - used.
*
*******************************************************************************/
void ADC_ACCELEROMETER_Wakeup(void)
{
    ADC_ACCELEROMETER_SAR_DFT_CTRL_REG &= (uint32)~ADC_ACCELEROMETER_ADFT_OVERRIDE;
    if(ADC_ACCELEROMETER_backup.enableState != ADC_ACCELEROMETER_DISABLED)
    {
        ADC_ACCELEROMETER_Start();
        if((ADC_ACCELEROMETER_backup.enableState & ADC_ACCELEROMETER_STARTED) != 0u)
        {
            ADC_ACCELEROMETER_StartConvert();
        }
    }
}
/* [] END OF FILE */
