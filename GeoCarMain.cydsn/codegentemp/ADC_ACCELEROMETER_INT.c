/*******************************************************************************
* File Name: ADC_ACCELEROMETER_INT.c
* Version 1.10
*
*  Description:
*    This file contains the code that operates during the ADC_SAR interrupt
*    service routine.
*
*   Note:
*
********************************************************************************
* Copyright 2008-2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "CyLib.h"
#include "ADC_ACCELEROMETER.h"


/******************************************************************************
* Custom Declarations and Variables
* - add user inlcude files, prototypes and variables between the following
*   #START and #END tags
******************************************************************************/
/* `#START ADC_SYS_VAR`  */

/* `#END`  */

#if(ADC_ACCELEROMETER_IRQ_REMOVE == 0u)


    /******************************************************************************
    * Function Name: ADC_ACCELEROMETER_ISR
    *******************************************************************************
    *
    * Summary:
    *  Handle Interrupt Service Routine.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Reentrant:
    *  No.
    *
    ******************************************************************************/
    CY_ISR( ADC_ACCELEROMETER_ISR )
    {
        uint32 intr_status;

        /* Read interrupt status register */
        intr_status = ADC_ACCELEROMETER_SAR_INTR_REG;

        /************************************************************************
        *  Custom Code
        *  - add user ISR code between the following #START and #END tags
        *************************************************************************/
        /* `#START MAIN_ADC_ISR`  */

        /* `#END`  */

        /* Clear handled interrupt */
        ADC_ACCELEROMETER_SAR_INTR_REG = intr_status;
    }

#endif   /* End ADC_ACCELEROMETER_IRQ_REMOVE */


/* [] END OF FILE */