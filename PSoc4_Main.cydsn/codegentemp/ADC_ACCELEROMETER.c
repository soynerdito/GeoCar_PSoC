/*******************************************************************************
* File Name: ADC_ACCELEROMETER.c
* Version 1.10
*
* Description:
*  This file provides the source code to the API for the Sequencing Successive
*  Approximation ADC Component Component.
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
* Global data allocation
***************************************/
uint8 ADC_ACCELEROMETER_initVar = 0u;
volatile int16 ADC_ACCELEROMETER_offset[ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM];
volatile int32 ADC_ACCELEROMETER_countsPer10Volt[ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM];   /* Gain compensation */


/***************************************
* Local data allocation
***************************************/
/* Channels configuration generated by customiser */
static const uint32 CYCODE ADC_ACCELEROMETER_channelsConfig[] = { 0x00000400u, 0x00001400u, 0x00001400u };


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_Start
********************************************************************************
*
* Summary:
*  Performs all required initialization for this component
*  and enables the power. The power will be set to the appropriate
*  power based on the clock frequency.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The ADC_ACCELEROMETER_initVar variable is used to indicate when/if initial
*  configuration of this component has happened. The variable is initialized to
*  zero and set to 1 the first time ADC_Start() is called. This allows for
*  component Re-Start without re-initialization in all subsequent calls to the
*  ADC_ACCELEROMETER_Start() routine.
*  If re-initialization of the component is required the variable should be set
*  to zero before call of ADC_ACCELEROMETER_Start() routine, or the user may call
*  ADC_ACCELEROMETER_Init() and ADC_ACCELEROMETER_Enable() as done in the
*  ADC_ACCELEROMETER_Start() routine.
*
*******************************************************************************/
void ADC_ACCELEROMETER_Start(void)
{
    /* If not Initialized then initialize all required hardware and software */
    if(ADC_ACCELEROMETER_initVar == 0u)
    {
        ADC_ACCELEROMETER_Init();
        ADC_ACCELEROMETER_initVar = 1u;
    }
    ADC_ACCELEROMETER_Enable();
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  ADC_ACCELEROMETER_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The ADC_ACCELEROMETER_offset variable is initialized.
*
*******************************************************************************/
void ADC_ACCELEROMETER_Init(void)
{
    uint32 chNum;
    uint32 tmpRegVal;
    int32 counts;

    #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 1u)
        static const uint8 CYCODE ADC_ACCELEROMETER_InputsPlacement[] =
        {
            (uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_0_PORT << 4u) |
            (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_0_PIN
            ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_1_PORT << 4u) |
             (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_1_PIN
            #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 2u)
                ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_2_PORT << 4u) |
                 (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_2_PIN
            #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 2u */
            #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 3u)
                ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_3_PORT << 4u) |
                 (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_3_PIN
            #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 3u */
            #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 4u)
                ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_4_PORT << 4u) |
                 (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_4_PIN
            #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 4u */
            #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 5u)
                ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_5_PORT << 4u) |
                 (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_5_PIN
            #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 5u */
            #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 6u)
                ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_6_PORT << 4u) |
                 (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_6_PIN
            #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 6u */
            #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 7u)
                ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_7_PORT << 4u) |
                 (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_7_PIN
            #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 7u */
            #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 8u)
                ,(uint8)(ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_8_PORT << 4u) |
                 (uint8)ADC_ACCELEROMETER_cy_psoc4_sarmux_8__CH_8_PIN
            #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 8u */
        };
    #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 1u */

    #if(ADC_ACCELEROMETER_IRQ_REMOVE == 0u)
        /* Start and set interrupt vector */
        CyIntSetPriority(ADC_ACCELEROMETER_INTC_NUMBER, ADC_ACCELEROMETER_INTC_PRIOR_NUMBER);
        (void)CyIntSetVector(ADC_ACCELEROMETER_INTC_NUMBER, &ADC_ACCELEROMETER_ISR);
    #endif   /* End ADC_ACCELEROMETER_IRQ_REMOVE */

    /* Init SAR and MUX registers */
    ADC_ACCELEROMETER_SAR_CHAN_EN_REG = ADC_ACCELEROMETER_DEFAULT_EN_CHANNELS;
    ADC_ACCELEROMETER_SAR_CTRL_REG = ADC_ACCELEROMETER_DEFAULT_CTRL_REG_CFG;
    ADC_ACCELEROMETER_SAR_SAMPLE_CTRL_REG = ADC_ACCELEROMETER_DEFAULT_SAMPLE_CTRL_REG_CFG;
    ADC_ACCELEROMETER_SAR_RANGE_THRES_REG = ADC_ACCELEROMETER_DEFAULT_RANGE_THRES_REG_CFG;
    ADC_ACCELEROMETER_SAR_RANGE_COND_REG  = ADC_ACCELEROMETER_COMPARE_MODE;
    ADC_ACCELEROMETER_SAR_SAMPLE_TIME01_REG = ADC_ACCELEROMETER_DEFAULT_SAMPLE_TIME01_REG_CFG;
    ADC_ACCELEROMETER_SAR_SAMPLE_TIME23_REG = ADC_ACCELEROMETER_DEFAULT_SAMPLE_TIME23_REG_CFG;
    /* Connect Vm to VSSA when even one channel is single-ended or multiple channels configured */
    #if(ADC_ACCELEROMETER_DEFAULT_MUX_SWITCH0 != 0u)
        ADC_ACCELEROMETER_MUX_SWITCH0_REG |= ADC_ACCELEROMETER_DEFAULT_MUX_SWITCH0; 
        /* Set MUX_HW_CTRL_VSSA in MUX_SWITCH_HW_CTRL when multiple channels enabled */
        #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 1u)
            ADC_ACCELEROMETER_MUX_SWITCH_HW_CTRL_REG |= ADC_ACCELEROMETER_DEFAULT_MUX_SWITCH0; 
        #endif /* ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 1u */
    #endif /*ADC_ACCELEROMETER_CHANNELS_MODE !=0 */    

    ADC_ACCELEROMETER_SAR_SATURATE_INTR_MASK_REG = 0u;
    ADC_ACCELEROMETER_SAR_RANGE_INTR_MASK_REG = 0u;
    ADC_ACCELEROMETER_SAR_INTR_MASK_REG = ADC_ACCELEROMETER_SAR_INTR_MASK;
    ADC_ACCELEROMETER_ANA_TRIM_REG = ADC_ACCELEROMETER_TRIM_COEF;
    
    /* Read and modify default configuration based on characterization */
    tmpRegVal = ADC_ACCELEROMETER_SAR_DFT_CTRL_REG;
    #if(ADC_ACCELEROMETER_NOMINAL_CLOCK_FREQ > (ADC_ACCELEROMETER_MAX_FREQUENCY / 2))
        tmpRegVal &= (uint32)~ADC_ACCELEROMETER_DCEN;
        tmpRegVal |= ADC_ACCELEROMETER_SEL_CSEL_DFT_CHAR;
    #else  /* clock speed < 9 Mhz */  
        tmpRegVal |= ADC_ACCELEROMETER_DLY_INC;
    #endif /* clock speed > 9 Mhz */
    ADC_ACCELEROMETER_SAR_DFT_CTRL_REG = tmpRegVal;
    
    #if(ADC_ACCELEROMETER_MAX_RESOLUTION != ADC_ACCELEROMETER_RESOLUTION_12)
        ADC_ACCELEROMETER_WOUNDING_REG = ADC_ACCELEROMETER_ALT_WOUNDING;
    #endif /* ADC_ACCELEROMETER_MAX_RESOLUTION != ADC_ACCELEROMETER_RESOLUTION_12 */

    for(chNum = 0u; chNum < ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM; chNum++)
    {
        tmpRegVal = (ADC_ACCELEROMETER_channelsConfig[chNum] & ADC_ACCELEROMETER_CHANNEL_CONFIG_MASK);
        #if(ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 1u)
            tmpRegVal |= ADC_ACCELEROMETER_InputsPlacement[chNum];
        #endif /* End ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM > 1u */
        /* When the part is wounded to 10-bit then the SUB_RESOLUTION bit 
        *  will be ignored and the RESOLUTION bit selects between 10-bit 
        *  (0) and 8-bit (1) resolution.
        */
        #if((ADC_ACCELEROMETER_MAX_RESOLUTION != ADC_ACCELEROMETER_RESOLUTION_12) && \
            (ADC_ACCELEROMETER_ALT_WOUNDING == ADC_ACCELEROMETER_WOUNDING_10BIT))
            tmpRegVal &= (uint32)(~ADC_ACCELEROMETER_ALT_RESOLUTION_ON);
        #endif /* ADC_ACCELEROMETER_MAX_RESOLUTION != ADC_ACCELEROMETER_RESOLUTION_12 */

        #if(ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED)
        if(chNum < ADC_ACCELEROMETER_SEQUENCED_CHANNELS_NUM)
        #endif /* ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED */
        {
            CY_SET_REG32((reg32 *)(ADC_ACCELEROMETER_SAR_CHAN_CONFIG_IND + (uint32)(chNum << 2)), tmpRegVal);

            if((ADC_ACCELEROMETER_channelsConfig[chNum] & ADC_ACCELEROMETER_IS_SATURATE_EN_MASK) != 0u)
            {
                ADC_ACCELEROMETER_SAR_SATURATE_INTR_MASK_REG |= (uint16)((uint16)1 << chNum);
            }

            if((ADC_ACCELEROMETER_channelsConfig[chNum] & ADC_ACCELEROMETER_IS_RANGE_CTRL_EN_MASK) != 0u)
            {
                ADC_ACCELEROMETER_SAR_RANGE_INTR_MASK_REG |= (uint16)((uint16)1 << chNum);
            }
        }
        #if(ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED)
            else
            {
                CY_SET_REG32(ADC_ACCELEROMETER_SAR_INJ_CHAN_CONFIG_PTR, tmpRegVal | ADC_ACCELEROMETER_INJ_TAILGATING);

                if((ADC_ACCELEROMETER_channelsConfig[chNum] & ADC_ACCELEROMETER_IS_SATURATE_EN_MASK) != 0u)
                {
                    ADC_ACCELEROMETER_SAR_INTR_MASK_REG |= ADC_ACCELEROMETER_INJ_SATURATE_MASK;
                }

                if((ADC_ACCELEROMETER_channelsConfig[chNum] & ADC_ACCELEROMETER_IS_RANGE_CTRL_EN_MASK) != 0u)
                {
                    ADC_ACCELEROMETER_SAR_INTR_MASK_REG |= ADC_ACCELEROMETER_INJ_RANGE_MASK;
                }
            }
        #endif /* ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED */

        if((ADC_ACCELEROMETER_channelsConfig[chNum] & ADC_ACCELEROMETER_ALT_RESOLUTION_ON) != 0u)
        {
               counts = (int32)ADC_ACCELEROMETER_DEFAULT_MAX_WRK_ALT;
        }
        else
        {
               counts = (int32)ADC_ACCELEROMETER_SAR_WRK_MAX_12BIT;
        }

        if((ADC_ACCELEROMETER_channelsConfig[chNum] & ADC_ACCELEROMETER_DIFFERENTIAL_EN) == 0u)
        {  
            #if((ADC_ACCELEROMETER_DEFAULT_SE_RESULT_FORMAT_SEL == ADC_ACCELEROMETER__FSIGNED) && \
                (ADC_ACCELEROMETER_DEFAULT_NEG_INPUT_SEL == ADC_ACCELEROMETER__VREF))
                /* Set offset to the minus half scale to convert results to unsigned format */
                ADC_ACCELEROMETER_offset[chNum] = (int16)(counts / -2);
            #else    
                ADC_ACCELEROMETER_offset[chNum] = 0;
            #endif /* end DEFAULT_SE_RESULT_FORMAT_SEL == ADC_ACCELEROMETER__FSIGNED */    
        }
        else    /* Differential channel */
        {
            #if(ADC_ACCELEROMETER_DEFAULT_DIFF_RESULT_FORMAT_SEL == ADC_ACCELEROMETER__FUNSIGNED)
                /* Set offset to the half scale to convert results to signed format */
                ADC_ACCELEROMETER_offset[chNum] = (int16)(counts / 2);
            #else    
                ADC_ACCELEROMETER_offset[chNum] = 0;
            #endif /* end ADC_ACCELEROMETER_DEFAULT_DIFF_RESULT_FORMAT_SEL == ADC_ACCELEROMETER__FUNSIGNED */    
        }
        /* Calculate gain in counts per 10 volts with rounding */
        ADC_ACCELEROMETER_countsPer10Volt[chNum] = (int16)(((counts * ADC_ACCELEROMETER_10MV_COUNTS) + 
                            ADC_ACCELEROMETER_DEFAULT_VREF_MV_VALUE) / (ADC_ACCELEROMETER_DEFAULT_VREF_MV_VALUE * 2));
    }
}

/*******************************************************************************
* Function Name: ADC_SAR_1_Enable
********************************************************************************
*
* Summary:
*  Enables the clock and analog power for SAR ADC.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_Enable(void)
{
    ADC_ACCELEROMETER_SAR_CTRL_REG |= ADC_ACCELEROMETER_ENABLE;
    /* The block is ready to use 10 us after the enable signal is set high. */
    CyDelayUs(ADC_ACCELEROMETER_10US_DELAY);
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_Stop
********************************************************************************
*
* Summary:
*  This function stops ADC conversions and puts the ADC into its lowest power
*  mode.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_Stop(void)
{
    ADC_ACCELEROMETER_SAR_CTRL_REG &= (uint32)~ADC_ACCELEROMETER_ENABLE;
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_StartConvert
********************************************************************************
*
* Summary:
*  Description:
*  For free running mode, this API starts the conversion process and it
*  runs continuously.
*
*  In a triggered mode, this routine triggers every conversion by
*  writing into the FW_TRIGGER bit in SAR_START_CTRL reg. In triggered mode,
*  every conversion has to start by this API.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_StartConvert(void)
{
    #if(ADC_ACCELEROMETER_DEFAULT_SAMPLE_MODE_SEL == ADC_ACCELEROMETER__FREERUNNING)
        ADC_ACCELEROMETER_SAR_SAMPLE_CTRL_REG |= ADC_ACCELEROMETER_CONTINUOUS_EN;
    #else /* Firmware trigger */
        ADC_ACCELEROMETER_SAR_START_CTRL_REG = ADC_ACCELEROMETER_FW_TRIGGER;
    #endif /* End ADC_ACCELEROMETER_DEFAULT_SAMPLE_MODE == ADC_ACCELEROMETER__FREERUNNING */

}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_StopConvert
********************************************************************************
*
* Summary:
*  Forces the ADC to stop all conversions.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_StopConvert(void)
{
    #if(ADC_ACCELEROMETER_DEFAULT_SAMPLE_MODE_SEL == ADC_ACCELEROMETER__FREERUNNING)
        ADC_ACCELEROMETER_SAR_SAMPLE_CTRL_REG &= (uint32)(~ADC_ACCELEROMETER_CONTINUOUS_EN);
    #endif /* ADC_ACCELEROMETER_DEFAULT_SAMPLE_MODE == ADC_ACCELEROMETER__FREERUNNING */
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_IsEndConversion
********************************************************************************
*
* Summary:
*  Description: Checks for ADC end of conversion for the case one
*  channel and end of scan for the case of multiple channels. It acts
*  as a software version of the EOC. This function provides the
*  programmer with two options. In one mode this function
*  immediately returns with the conversion status. In the other mode,
*  the function does not return (blocking) until the conversion has
*  completed.
*
* Parameters:
*  ADC_RETURN_STATUS        -> Immediately returns conversion result status
*  ADC_WAIT_FOR_RESULT      -> Does not return until ADC complete
*  ADC_RETURN_STATUS_INJ    -> Immediately returns conversion result status
*                              for injection channel
*  ADC_WAIT_FOR_RESULT_INJ  -> Does not return until ADC completes injection
*                              channel conversion
*
* Return:
*  If a non-zero value is returned, the last conversion or scan has completed.
*  If the returned value is zero, the ADC is still in the process of a scan.
*
*******************************************************************************/
uint32 ADC_ACCELEROMETER_IsEndConversion(uint32 retMode)
{
    uint32 status = 0u;

    if((retMode & (ADC_ACCELEROMETER_RETURN_STATUS | ADC_ACCELEROMETER_WAIT_FOR_RESULT)) != 0u)
    {
        do
        {
            status = ADC_ACCELEROMETER_SAR_INTR_REG & ADC_ACCELEROMETER_EOS_MASK;
        }while((status == 0u) && ((retMode & ADC_ACCELEROMETER_WAIT_FOR_RESULT) != 0u));

        if(status != 0u)
        {
            /* Clear EOS bit */
            ADC_ACCELEROMETER_SAR_INTR_REG = ADC_ACCELEROMETER_EOS_MASK;
        }
    }

    #if(ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED)
        if((retMode & (ADC_ACCELEROMETER_RETURN_STATUS_INJ | ADC_ACCELEROMETER_WAIT_FOR_RESULT_INJ)) != 0u)
        {
            do
            {
                status |= ADC_ACCELEROMETER_SAR_INTR_REG & ADC_ACCELEROMETER_INJ_EOC_MASK;
            }while(((status & ADC_ACCELEROMETER_INJ_EOC_MASK) != 0u) &&
                   ((retMode & ADC_ACCELEROMETER_WAIT_FOR_RESULT_INJ) != 0u));

            if((status & ADC_ACCELEROMETER_INJ_EOC_MASK) != 0u)
            {
                /* Clear Injection EOS bit */
                ADC_ACCELEROMETER_SAR_INTR_REG = ADC_ACCELEROMETER_INJ_EOC_MASK;
            }
        }
    #endif /* ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED */

    return (status);
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_GetResult16
********************************************************************************
*
* Summary:
*  Gets the data available in the SAR DATA register.
*
* Parameters:
*  chan: The ADC channel in which to return the result. The first channel
*  is 0 and the injection channel if enabled is the number of valid channels.
*
* Return:
*  Returns converted data as a signed 16-bit integer
*
*******************************************************************************/
int16 ADC_ACCELEROMETER_GetResult16(uint32 chan)
{
    uint32 result;

    /* Halt CPU in debug mode if channel is out of valid range */
    CYASSERT(chan < ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM);

    if(chan < ADC_ACCELEROMETER_SEQUENCED_CHANNELS_NUM)
    {
        result = CY_GET_REG32((reg32 *)(ADC_ACCELEROMETER_SAR_CHAN_RESULT_IND + (uint32)(chan << 2u))) &
                ADC_ACCELEROMETER_RESULT_MASK;
    }
    else
    {
        #if(ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED)
            result = ADC_ACCELEROMETER_SAR_INJ_RESULT_REG & ADC_ACCELEROMETER_RESULT_MASK;
        #else
            result = 0u;
        #endif /* ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED */
    }

    return ( (int16)result );
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SetChanMask
********************************************************************************
*
* Summary:
*  Sets the channel enable mask.
*
* Parameters:
*  mask: Sets which channels that will be
*  scanned. Setting bits for channels that do not exist will have no
*  effect. For example, if only 6 channels were enabled, setting a
*  mask of 0x0103 would only enable the last two channels (0 and 1).
*  This API will not enable the injection channel.
*  Examples: If the component is setup to sequence through 8
*  channels, a mask of 0x000F would enable channels 0, 1, 2, and 3.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_SetChanMask(uint32 mask)
{
    ADC_ACCELEROMETER_SAR_CHAN_EN_REG = mask & ADC_ACCELEROMETER_MAX_CHANNELS_EN_MASK;
}

#if(ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED)


    /*******************************************************************************
    * Function Name: ADC_ACCELEROMETER_EnableInjection
    ********************************************************************************
    *
    * Summary:
    *  Enables the injection channel for the next scan only.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void ADC_ACCELEROMETER_EnableInjection(void)
    {
        ADC_ACCELEROMETER_SAR_INJ_CHAN_CONFIG_REG |= ADC_ACCELEROMETER_INJ_CHAN_EN;
    }

#endif /* ADC_ACCELEROMETER_INJ_CHANNEL_ENABLED */


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SetLowLimit
********************************************************************************
*
* Summary:
*  Sets the low limit parameter for a limit condition.
*
* Parameters:
*  lowLimit: The low limit for a limit condition.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_SetLowLimit(uint32 lowLimit)
{
    ADC_ACCELEROMETER_SAR_RANGE_THRES_REG &= (uint32)(~ADC_ACCELEROMETER_RANGE_LOW_MASK);
    ADC_ACCELEROMETER_SAR_RANGE_THRES_REG |= lowLimit & ADC_ACCELEROMETER_RANGE_LOW_MASK;
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SetHighLimit
********************************************************************************
*
* Summary:
*  Sets the low limit parameter for a limit condition.
*
* Parameters:
*  highLimit: The high limit for a limit condition.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_SetHighLimit(uint32 highLimit)
{
    ADC_ACCELEROMETER_SAR_RANGE_THRES_REG &= (uint32)(~ADC_ACCELEROMETER_RANGE_HIGH_MASK);
    ADC_ACCELEROMETER_SAR_RANGE_THRES_REG |= (uint32)(highLimit << ADC_ACCELEROMETER_RANGE_HIGH_OFFSET);
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SetLimitMask
********************************************************************************
*
* Summary:
*  Sets the channel limit condition mask.
*
* Parameters:
*  mask: Sets which channels that may cause a
*  limit condition interrupt. Setting bits for channels that do not exist
*  will have no effect. For example, if only 6 channels were enabled,
*  setting a mask of 0x0103 would only enable the last two channels (0 and 1).
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_SetLimitMask(uint32 mask)
{
    ADC_ACCELEROMETER_SAR_RANGE_INTR_MASK_REG = mask & ADC_ACCELEROMETER_MAX_CHANNELS_EN_MASK;
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SetSatMask
********************************************************************************
*
* Summary:
*  Sets the channel saturation event mask.
*
* Parameters:
*  mask: Sets which channels that may cause a
*  saturation event interrupt. Setting bits for channels that do not exist
*  will have no effect. For example, if only 8 channels were enabled,
*  setting a mask of 0x01C0 would only enable two channels (6 and 7).
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_ACCELEROMETER_SetSatMask(uint32 mask)
{
    ADC_ACCELEROMETER_SAR_SATURATE_INTR_MASK_REG = mask & ADC_ACCELEROMETER_MAX_CHANNELS_EN_MASK;
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SetOffset
********************************************************************************
*
* Summary:
*   Description: Sets the ADC offset which is used by the functions
*   ADC_CountsTo_uVolts, ADC_CountsTo_mVolts and ADC_CountsTo_Volts
*   to substract the offset from the given reading
*   before calculating the voltage conversion.
*
* Parameters:
*  chan: ADC channel number.
*  offset: This value is a measured value when the
*          inputs are shorted or connected to the same input voltage.
*
* Return:
*  None.
*
* Global variables:
*  ADC_ACCELEROMETER_Offset:  Modified to set the user provided offset.
*
*******************************************************************************/
void ADC_ACCELEROMETER_SetOffset(uint32 chan, int16 offset)
{
    /* Halt CPU in debug mode if channel is out of valid range */
    CYASSERT(chan < ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM);

    ADC_ACCELEROMETER_offset[chan] = offset;
}


/*******************************************************************************
* Function Name: ADC_ACCELEROMETER_SetGain
********************************************************************************
*
* Summary:
*  Description: Sets the ADC gain in counts per 10 volt for the voltage
*  conversion functions below. This value is set by default by the
*  reference and input range settings. It should only be used to further
*  calibrate the ADC with a known input or if an external reference is
*  used. Affects the ADC_CountsTo_uVolts, ADC_CountsTo_mVolts
*  and ADC_CountsTo_Volts functions by supplying the correct
*  conversion between ADC counts and voltage.
*
* Parameters:
*  chan: ADC channel number.
*  adcGain: ADC gain in counts per 10 volts.
*
* Return:
*  None.
*
* Global variables:
*  ADC_ACCELEROMETER_CountsPer10Volt:  modified to set the ADC gain in counts 
*   per 10 volt.
*
*******************************************************************************/
void ADC_ACCELEROMETER_SetGain(uint32 chan, int32 adcGain)
{
    /* Halt CPU in debug mode if channel is out of valid range */
    CYASSERT(chan < ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM);

    ADC_ACCELEROMETER_countsPer10Volt[chan] = adcGain;
}


#if(ADC_ACCELEROMETER_DEFAULT_JUSTIFICATION_SEL == ADC_ACCELEROMETER__RIGHT)


    /*******************************************************************************
    * Function Name: ADC_ACCELEROMETER_CountsTo_mVolts
    ********************************************************************************
    *
    * Summary:
    *  This function converts ADC counts to mVolts
    *  This function is not available when left data format justification selected.
    *
    * Parameters:
    *  chan: The ADC channel number.
    *  adcCounts: Result from the ADC conversion
    *
    * Return:
    *  Results in mVolts
    *
    * Global variables:
    *  ADC_ACCELEROMETER_countsPer10Volt:  used to convert ADC counts to mVolts.
    *  ADC_ACCELEROMETER_Offset:  Used as the offset while converting ADC counts 
    *   to mVolts.
    *
    *******************************************************************************/
    int16 ADC_ACCELEROMETER_CountsTo_mVolts(uint32 chan, int16 adcCounts)
    {
        int16 mVolts;

        /* Halt CPU in debug mode if channel is out of valid range */
        CYASSERT(chan < ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM);

        /* Divide the adcCount when accumulate averaging mode selected */
        #if(ADC_ACCELEROMETER_DEFAULT_AVG_MODE == ADC_ACCELEROMETER__ACCUMULATE)
            if((ADC_ACCELEROMETER_channelsConfig[chan] & ADC_ACCELEROMETER_AVERAGING_EN) != 0u)
            {
                adcCounts /= ADC_ACCELEROMETER_DEFAULT_AVG_SAMPLES_DIV;
            }    
        #endif /* ADC_ACCELEROMETER_DEFAULT_AVG_MODE == ADC_ACCELEROMETER__ACCUMULATE */

        /* Subtract ADC offset */
        adcCounts -= ADC_ACCELEROMETER_offset[chan];

        mVolts = (int16)((((int32)adcCounts * ADC_ACCELEROMETER_10MV_COUNTS) + ( (adcCounts > 0) ? 
                 (ADC_ACCELEROMETER_countsPer10Volt[chan] / 2) : (-(ADC_ACCELEROMETER_countsPer10Volt[chan] / 2)) )) 
                 / ADC_ACCELEROMETER_countsPer10Volt[chan]);

        return( mVolts );
    }


    /*******************************************************************************
    * Function Name: ADC_ACCELEROMETER_CountsTo_uVolts
    ********************************************************************************
    *
    * Summary:
    *  This function converts ADC counts to micro Volts
    *  This function is not available when left data format justification selected.
    *
    * Parameters:
    *  chan: The ADC channel number.
    *  adcCounts: Result from the ADC conversion
    *
    * Return:
    *  Results in uVolts
    *
    * Global variables:
    *  ADC_ACCELEROMETER_countsPer10Volt:  used to convert ADC counts to uVolts.
    *  ADC_ACCELEROMETER_Offset:  Used as the offset while converting ADC counts 
    *   to mVolts.
    *
    * Theory: 
    *  Care must be taken to not exceed the maximum value for a 31 bit signed
    *  number in the conversion to uVolts and at the same time not loose 
    *  resolution.
    *  To convert adcCounts to microVolts it is required to be multiplied
    *  on 10 million and later divide on gain in counts per 10V. 
    *
    *******************************************************************************/
    int32 ADC_ACCELEROMETER_CountsTo_uVolts(uint32 chan, int16 adcCounts)
    {
        int64 uVolts;

        /* Halt CPU in debug mode if channel is out of valid range */
        CYASSERT(chan < ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM);

        /* Divide the adcCount when accumulate averaging mode selected */
        #if(ADC_ACCELEROMETER_DEFAULT_AVG_MODE == ADC_ACCELEROMETER__ACCUMULATE)
            if((ADC_ACCELEROMETER_channelsConfig[chan] & ADC_ACCELEROMETER_AVERAGING_EN) != 0u)
            {
                adcCounts /= ADC_ACCELEROMETER_DEFAULT_AVG_SAMPLES_DIV;
            }    
        #endif /* ADC_ACCELEROMETER_DEFAULT_AVG_MODE == ADC_ACCELEROMETER__ACCUMULATE */

        /* Subtract ADC offset */
        adcCounts -= ADC_ACCELEROMETER_offset[chan];

        uVolts = ((int64)adcCounts * ADC_ACCELEROMETER_10UV_COUNTS) / ADC_ACCELEROMETER_countsPer10Volt[chan];

        return( (int32)uVolts );
    }


    /*******************************************************************************
    * Function Name: ADC_ACCELEROMETER_CountsTo_Volts
    ********************************************************************************
    *
    * Summary:
    *  Converts the ADC output to Volts as a floating point number.
    *  This function is not available when left data format justification selected.
    *
    * Parameters:
    *  chan: The ADC channel number.
    *  Result from the ADC conversion
    *
    * Return:
    *  Results in Volts
    *
    * Global variables:
    *  ADC_ACCELEROMETER_countsPer10Volt:  used to convert ADC counts to Volts.
    *  ADC_ACCELEROMETER_Offset:  Used as the offset while converting ADC counts 
    *   to mVolts.
    *
    *******************************************************************************/
    float32 ADC_ACCELEROMETER_CountsTo_Volts(uint32 chan, int16 adcCounts)
    {
        float32 volts;

        /* Halt CPU in debug mode if channel is out of valid range */
        CYASSERT(chan < ADC_ACCELEROMETER_TOTAL_CHANNELS_NUM);

        /* Divide the adcCount when accumulate averaging mode selected */
        #if(ADC_ACCELEROMETER_DEFAULT_AVG_MODE == ADC_ACCELEROMETER__ACCUMULATE)
            if((ADC_ACCELEROMETER_channelsConfig[chan] & ADC_ACCELEROMETER_AVERAGING_EN) != 0u)
            {
                adcCounts /= ADC_ACCELEROMETER_DEFAULT_AVG_SAMPLES_DIV;
            }    
        #endif /* ADC_ACCELEROMETER_DEFAULT_AVG_MODE == ADC_ACCELEROMETER__ACCUMULATE */

        /* Subtract ADC offset */
        adcCounts -= ADC_ACCELEROMETER_offset[chan];

        volts = ((float32)adcCounts * ADC_ACCELEROMETER_10V_COUNTS) / (float32)ADC_ACCELEROMETER_countsPer10Volt[chan];

        return( volts );
    }

#endif /* End ADC_ACCELEROMETER_DEFAULT_JUSTIFICATION_SEL == ADC_ACCELEROMETER__RIGHT */


/* [] END OF FILE */
