/*******************************************************************************
* File Name: Buzz_PM.c
* Version 3.0
*
* Description:
*  This file provides the power management source code to API for the
*  PWM.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#include "cytypes.h"
#include "Buzz.h"

static Buzz_backupStruct Buzz_backup;


/*******************************************************************************
* Function Name: Buzz_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration of the component.
*  
* Parameters:  
*  None
*
* Return: 
*  None
*
* Global variables:
*  Buzz_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
*******************************************************************************/
void Buzz_SaveConfig(void) 
{
    
    #if(!Buzz_UsingFixedFunction)
        #if(!Buzz_PWMModeIsCenterAligned)
            Buzz_backup.PWMPeriod = Buzz_ReadPeriod();
        #endif /* (!Buzz_PWMModeIsCenterAligned) */
        Buzz_backup.PWMUdb = Buzz_ReadCounter();
        #if (Buzz_UseStatus)
            Buzz_backup.InterruptMaskValue = Buzz_STATUS_MASK;
        #endif /* (Buzz_UseStatus) */
        
        #if(Buzz_DeadBandMode == Buzz__B_PWM__DBM_256_CLOCKS || \
            Buzz_DeadBandMode == Buzz__B_PWM__DBM_2_4_CLOCKS)
            Buzz_backup.PWMdeadBandValue = Buzz_ReadDeadTime();
        #endif /*  deadband count is either 2-4 clocks or 256 clocks */
        
        #if(Buzz_KillModeMinTime)
             Buzz_backup.PWMKillCounterPeriod = Buzz_ReadKillTime();
        #endif /* (Buzz_KillModeMinTime) */
        
        #if(Buzz_UseControl)
            Buzz_backup.PWMControlRegister = Buzz_ReadControlRegister();
        #endif /* (Buzz_UseControl) */
    #endif  /* (!Buzz_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: Buzz_RestoreConfig
********************************************************************************
* 
* Summary:
*  Restores the current user configuration of the component.
*
* Parameters:  
*  None
*
* Return: 
*  None
*
* Global variables:
*  Buzz_backup:  Variables of this global structure are used to  
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Buzz_RestoreConfig(void) 
{
        #if(!Buzz_UsingFixedFunction)
            #if(!Buzz_PWMModeIsCenterAligned)
                Buzz_WritePeriod(Buzz_backup.PWMPeriod);
            #endif /* (!Buzz_PWMModeIsCenterAligned) */
            Buzz_WriteCounter(Buzz_backup.PWMUdb);
            #if (Buzz_UseStatus)
                Buzz_STATUS_MASK = Buzz_backup.InterruptMaskValue;
            #endif /* (Buzz_UseStatus) */
            
            #if(Buzz_DeadBandMode == Buzz__B_PWM__DBM_256_CLOCKS || \
                Buzz_DeadBandMode == Buzz__B_PWM__DBM_2_4_CLOCKS)
                Buzz_WriteDeadTime(Buzz_backup.PWMdeadBandValue);
            #endif /* deadband count is either 2-4 clocks or 256 clocks */
            
            #if(Buzz_KillModeMinTime)
                Buzz_WriteKillTime(Buzz_backup.PWMKillCounterPeriod);
            #endif /* (Buzz_KillModeMinTime) */
            
            #if(Buzz_UseControl)
                Buzz_WriteControlRegister(Buzz_backup.PWMControlRegister); 
            #endif /* (Buzz_UseControl) */
        #endif  /* (!Buzz_UsingFixedFunction) */
    }


/*******************************************************************************
* Function Name: Buzz_Sleep
********************************************************************************
* 
* Summary:
*  Disables block's operation and saves the user configuration. Should be called 
*  just prior to entering sleep.
*  
* Parameters:  
*  None
*
* Return: 
*  None
*
* Global variables:
*  Buzz_backup.PWMEnableState:  Is modified depending on the enable 
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void Buzz_Sleep(void) 
{
    #if(Buzz_UseControl)
        if(Buzz_CTRL_ENABLE == (Buzz_CONTROL & Buzz_CTRL_ENABLE))
        {
            /*Component is enabled */
            Buzz_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            Buzz_backup.PWMEnableState = 0u;
        }
    #endif /* (Buzz_UseControl) */

    /* Stop component */
    Buzz_Stop();
    
    /* Save registers configuration */
    Buzz_SaveConfig();
}


/*******************************************************************************
* Function Name: Buzz_Wakeup
********************************************************************************
* 
* Summary:
*  Restores and enables the user configuration. Should be called just after 
*  awaking from sleep.
*  
* Parameters:  
*  None
*
* Return: 
*  None
*
* Global variables:
*  Buzz_backup.pwmEnable:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Buzz_Wakeup(void) 
{
     /* Restore registers values */
    Buzz_RestoreConfig();
    
    if(Buzz_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        Buzz_Enable();
    } /* Do nothing if component's block was disabled before */
    
}


/* [] END OF FILE */
