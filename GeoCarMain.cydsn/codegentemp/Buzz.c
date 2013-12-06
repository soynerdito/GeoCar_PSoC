/*******************************************************************************
* File Name: Buzz.c  
* Version 3.0
*
* Description:
*  The PWM User Module consist of an 8 or 16-bit counter with two 8 or 16-bit
*  comparitors. Each instance of this user module is capable of generating
*  two PWM outputs with the same period. The pulse width is selectable between
*  1 and 255/65535. The period is selectable between 2 and 255/65536 clocks. 
*  The compare value output may be configured to be active when the present 
*  counter is less than or less than/equal to the compare value.
*  A terminal count output is also provided. It generates a pulse one clock
*  width wide when the counter is equal to zero.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#include "cytypes.h"
#include "Buzz.h"

uint8 Buzz_initVar = 0u;


/*******************************************************************************
* Function Name: Buzz_Start
********************************************************************************
*
* Summary:
*  The start function initializes the pwm with the default values, the 
*  enables the counter to begin counting.  It does not enable interrupts,
*  the EnableInt command should be called if interrupt generation is required.
*
* Parameters:  
*  None  
*
* Return: 
*  None
*
* Global variables:
*  Buzz_initVar: Is modified when this function is called for the 
*   first time. Is used to ensure that initialization happens only once.
*
*******************************************************************************/
void Buzz_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Buzz_initVar == 0u)
    {
        Buzz_Init();
        Buzz_initVar = 1u;
    }
    Buzz_Enable();

}


/*******************************************************************************
* Function Name: Buzz_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the 
*  customizer of the component placed onto schematic. Usually called in 
*  Buzz_Start().
*
* Parameters:  
*  None
*
* Return: 
*  None
*
*******************************************************************************/
void Buzz_Init(void) 
{
    #if (Buzz_UsingFixedFunction || Buzz_UseControl)
        uint8 ctrl;
    #endif /* (Buzz_UsingFixedFunction || Buzz_UseControl) */
    
    #if(!Buzz_UsingFixedFunction) 
        #if(Buzz_UseStatus)
            /* Interrupt State Backup for Critical Region*/
            uint8 Buzz_interruptState;
        #endif /* (Buzz_UseStatus) */
    #endif /* (!Buzz_UsingFixedFunction) */
    
    #if (Buzz_UsingFixedFunction)
        /* You are allowed to write the compare value (FF only) */
        Buzz_CONTROL |= Buzz_CFG0_MODE;
        #if (Buzz_DeadBand2_4)
            Buzz_CONTROL |= Buzz_CFG0_DB;
        #endif /* (Buzz_DeadBand2_4) */
                
        ctrl = Buzz_CONTROL3 & ((uint8 )(~Buzz_CTRL_CMPMODE1_MASK));
        Buzz_CONTROL3 = ctrl | Buzz_DEFAULT_COMPARE1_MODE;
        
         /* Clear and Set SYNCTC and SYNCCMP bits of RT1 register */
        Buzz_RT1 &= ((uint8)(~Buzz_RT1_MASK));
        Buzz_RT1 |= Buzz_SYNC;     
                
        /*Enable DSI Sync all all inputs of the PWM*/
        Buzz_RT1 &= ((uint8)(~Buzz_SYNCDSI_MASK));
        Buzz_RT1 |= Buzz_SYNCDSI_EN;
       
    #elif (Buzz_UseControl)
        /* Set the default compare mode defined in the parameter */
        ctrl = Buzz_CONTROL & ((uint8)(~Buzz_CTRL_CMPMODE2_MASK)) & ((uint8)(~Buzz_CTRL_CMPMODE1_MASK));
        Buzz_CONTROL = ctrl | Buzz_DEFAULT_COMPARE2_MODE | 
                                   Buzz_DEFAULT_COMPARE1_MODE;
    #endif /* (Buzz_UsingFixedFunction) */
        
    #if (!Buzz_UsingFixedFunction)
        #if (Buzz_Resolution == 8)
            /* Set FIFO 0 to 1 byte register for period*/
            Buzz_AUX_CONTROLDP0 |= (Buzz_AUX_CTRL_FIFO0_CLR);
        #else /* (Buzz_Resolution == 16)*/
            /* Set FIFO 0 to 1 byte register for period */
            Buzz_AUX_CONTROLDP0 |= (Buzz_AUX_CTRL_FIFO0_CLR);
            Buzz_AUX_CONTROLDP1 |= (Buzz_AUX_CTRL_FIFO0_CLR);
        #endif /* (Buzz_Resolution == 8) */

        Buzz_WriteCounter(Buzz_INIT_PERIOD_VALUE);
    #endif /* (!Buzz_UsingFixedFunction) */
        
    Buzz_WritePeriod(Buzz_INIT_PERIOD_VALUE);

        #if (Buzz_UseOneCompareMode)
            Buzz_WriteCompare(Buzz_INIT_COMPARE_VALUE1);
        #else
            Buzz_WriteCompare1(Buzz_INIT_COMPARE_VALUE1);
            Buzz_WriteCompare2(Buzz_INIT_COMPARE_VALUE2);
        #endif /* (Buzz_UseOneCompareMode) */
        
        #if (Buzz_KillModeMinTime)
            Buzz_WriteKillTime(Buzz_MinimumKillTime);
        #endif /* (Buzz_KillModeMinTime) */
        
        #if (Buzz_DeadBandUsed)
            Buzz_WriteDeadTime(Buzz_INIT_DEAD_TIME);
        #endif /* (Buzz_DeadBandUsed) */

    #if (Buzz_UseStatus || Buzz_UsingFixedFunction)
        Buzz_SetInterruptMode(Buzz_INIT_INTERRUPTS_MODE);
    #endif /* (Buzz_UseStatus || Buzz_UsingFixedFunction) */
        
    #if (Buzz_UsingFixedFunction)
        /* Globally Enable the Fixed Function Block chosen */
        Buzz_GLOBAL_ENABLE |= Buzz_BLOCK_EN_MASK;
        /* Set the Interrupt source to come from the status register */
        Buzz_CONTROL2 |= Buzz_CTRL2_IRQ_SEL;
    #else
        #if(Buzz_UseStatus)
            
            /* CyEnterCriticalRegion and CyExitCriticalRegion are used to mark following region critical*/
            /* Enter Critical Region*/
            Buzz_interruptState = CyEnterCriticalSection();
            /* Use the interrupt output of the status register for IRQ output */
            Buzz_STATUS_AUX_CTRL |= Buzz_STATUS_ACTL_INT_EN_MASK;
            
             /* Exit Critical Region*/
            CyExitCriticalSection(Buzz_interruptState);
            
            /* Clear the FIFO to enable the Buzz_STATUS_FIFOFULL
                   bit to be set on FIFO full. */
            Buzz_ClearFIFO();
        #endif /* (Buzz_UseStatus) */
    #endif /* (Buzz_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: Buzz_Enable
********************************************************************************
*
* Summary: 
*  Enables the PWM block operation
*
* Parameters:  
*  None
*
* Return: 
*  None
*
* Side Effects: 
*  This works only if software enable mode is chosen
*
*******************************************************************************/
void Buzz_Enable(void) 
{
    /* Globally Enable the Fixed Function Block chosen */
    #if (Buzz_UsingFixedFunction)
        Buzz_GLOBAL_ENABLE |= Buzz_BLOCK_EN_MASK;
        Buzz_GLOBAL_STBY_ENABLE |= Buzz_BLOCK_STBY_EN_MASK;
    #endif /* (Buzz_UsingFixedFunction) */
    
    /* Enable the PWM from the control register  */
    #if (Buzz_UseControl || Buzz_UsingFixedFunction)
        Buzz_CONTROL |= Buzz_CTRL_ENABLE;
    #endif /* (Buzz_UseControl || Buzz_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: Buzz_Stop
********************************************************************************
*
* Summary:
*  The stop function halts the PWM, but does not change any modes or disable
*  interrupts.
*
* Parameters:  
*  None  
*
* Return: 
*  None
*
* Side Effects:
*  If the Enable mode is set to Hardware only then this function
*  has no effect on the operation of the PWM
*
*******************************************************************************/
void Buzz_Stop(void) 
{
    #if (Buzz_UseControl || Buzz_UsingFixedFunction)
        Buzz_CONTROL &= ((uint8)(~Buzz_CTRL_ENABLE));
    #endif /* (Buzz_UseControl || Buzz_UsingFixedFunction) */
    
    /* Globally disable the Fixed Function Block chosen */
    #if (Buzz_UsingFixedFunction)
        Buzz_GLOBAL_ENABLE &= ((uint8)(~Buzz_BLOCK_EN_MASK));
        Buzz_GLOBAL_STBY_ENABLE &= ((uint8)(~Buzz_BLOCK_STBY_EN_MASK));
    #endif /* (Buzz_UsingFixedFunction) */
}


#if (Buzz_UseOneCompareMode)
	#if (Buzz_CompareMode1SW)


		/*******************************************************************************
		* Function Name: Buzz_SetCompareMode
		********************************************************************************
		* 
		* Summary:
		*  This function writes the Compare Mode for the pwm output when in Dither mode,
		*  Center Align Mode or One Output Mode.
		*
		* Parameters:
		*  comparemode:  The new compare mode for the PWM output. Use the compare types
		*                defined in the H file as input arguments.
		*
		* Return:
		*  None
		*
		*******************************************************************************/
		void Buzz_SetCompareMode(uint8 comparemode) 
		{
		    #if(Buzz_UsingFixedFunction)
            
                #if(0 != Buzz_CTRL_CMPMODE1_SHIFT)
                    uint8 comparemodemasked = ((uint8)((uint8)comparemode << Buzz_CTRL_CMPMODE1_SHIFT));
                #else
                    uint8 comparemodemasked = comparemode;
                #endif /* (0 != Buzz_CTRL_CMPMODE1_SHIFT) */
            
	            Buzz_CONTROL3 &= ((uint8)(~Buzz_CTRL_CMPMODE1_MASK)); /*Clear Existing Data */
	            Buzz_CONTROL3 |= comparemodemasked;     
		                
		    #elif (Buzz_UseControl)
		        
                #if(0 != Buzz_CTRL_CMPMODE1_SHIFT)
                    uint8 comparemode1masked = ((uint8)((uint8)comparemode << Buzz_CTRL_CMPMODE1_SHIFT)) & 
    		                                    Buzz_CTRL_CMPMODE1_MASK;
                #else
                    uint8 comparemode1masked = comparemode & Buzz_CTRL_CMPMODE1_MASK;                
                #endif /* (0 != Buzz_CTRL_CMPMODE1_SHIFT) */
                
                #if(0 != Buzz_CTRL_CMPMODE2_SHIFT)                            
    		        uint8 comparemode2masked = ((uint8)((uint8)comparemode << Buzz_CTRL_CMPMODE2_SHIFT)) & 
    		                                   Buzz_CTRL_CMPMODE2_MASK;
                #else
    		        uint8 comparemode2masked = comparemode & Buzz_CTRL_CMPMODE2_MASK;                
                #endif /* (0 != Buzz_CTRL_CMPMODE2_SHIFT) */
                
		        /*Clear existing mode */
		        Buzz_CONTROL &= ((uint8)(~(Buzz_CTRL_CMPMODE1_MASK | Buzz_CTRL_CMPMODE2_MASK))); 
		        Buzz_CONTROL |= (comparemode1masked | comparemode2masked);
		        
		    #else
		        uint8 temp = comparemode;
		    #endif /* (Buzz_UsingFixedFunction) */
		}
	#endif /* Buzz_CompareMode1SW */

#else /* UseOneCompareMode */

	#if (Buzz_CompareMode1SW)


		/*******************************************************************************
		* Function Name: Buzz_SetCompareMode1
		********************************************************************************
		* 
		* Summary:
		*  This function writes the Compare Mode for the pwm or pwm1 output
		*
		* Parameters:  
		*  comparemode:  The new compare mode for the PWM output. Use the compare types
		*                defined in the H file as input arguments.
		*
		* Return: 
		*  None
		*
		*******************************************************************************/
		void Buzz_SetCompareMode1(uint8 comparemode) 
		{
		    #if(0 != Buzz_CTRL_CMPMODE1_SHIFT)
                uint8 comparemodemasked = ((uint8)((uint8)comparemode << Buzz_CTRL_CMPMODE1_SHIFT)) & 
    		                               Buzz_CTRL_CMPMODE1_MASK;
		    #else
                uint8 comparemodemasked = comparemode & Buzz_CTRL_CMPMODE1_MASK;                
            #endif /* (0 != Buzz_CTRL_CMPMODE1_SHIFT) */
                   
		    #if (Buzz_UseControl)
		        Buzz_CONTROL &= ((uint8)(~Buzz_CTRL_CMPMODE1_MASK)); /*Clear existing mode */
		        Buzz_CONTROL |= comparemodemasked;
		    #endif /* (Buzz_UseControl) */
		}
	#endif /* Buzz_CompareMode1SW */

#if (Buzz_CompareMode2SW)


    /*******************************************************************************
    * Function Name: Buzz_SetCompareMode2
    ********************************************************************************
    * 
    * Summary:
    *  This function writes the Compare Mode for the pwm or pwm2 output
    *
    * Parameters:  
    *  comparemode:  The new compare mode for the PWM output. Use the compare types
    *                defined in the H file as input arguments.
    *
    * Return: 
    *  None
    *
    *******************************************************************************/
    void Buzz_SetCompareMode2(uint8 comparemode) 
    {

        #if(0 != Buzz_CTRL_CMPMODE2_SHIFT)
            uint8 comparemodemasked = ((uint8)((uint8)comparemode << Buzz_CTRL_CMPMODE2_SHIFT)) & 
                                                 Buzz_CTRL_CMPMODE2_MASK;
        #else
            uint8 comparemodemasked = comparemode & Buzz_CTRL_CMPMODE2_MASK;            
        #endif /* (0 != Buzz_CTRL_CMPMODE2_SHIFT) */
        
        #if (Buzz_UseControl)
            Buzz_CONTROL &= ((uint8)(~Buzz_CTRL_CMPMODE2_MASK)); /*Clear existing mode */
            Buzz_CONTROL |= comparemodemasked;
        #endif /* (Buzz_UseControl) */
    }
    #endif /*Buzz_CompareMode2SW */

#endif /* UseOneCompareMode */


#if (!Buzz_UsingFixedFunction)


    /*******************************************************************************
    * Function Name: Buzz_WriteCounter
    ********************************************************************************
    * 
    * Summary:
    *  Writes a new counter value directly to the counter register. This will be 
    *  implemented for that currently running period and only that period. This API 
    *  is valid only for UDB implementation and not available for fixed function 
    *  PWM implementation.    
    *
    * Parameters:  
    *  counter:  The period new period counter value.
    *
    * Return: 
    *  None
    *
    * Side Effects: 
    *  The PWM Period will be reloaded when a counter value will be a zero
    *
    *******************************************************************************/
    void Buzz_WriteCounter(uint16 counter) \
                                       
    {
        CY_SET_REG16(Buzz_COUNTER_LSB_PTR, counter);
    }


    /*******************************************************************************
    * Function Name: Buzz_ReadCounter
    ********************************************************************************
    * 
    * Summary:
    *  This function returns the current value of the counter.  It doesn't matter
    *  if the counter is enabled or running.
    *
    * Parameters:  
    *  None  
    *
    * Return: 
    *  The current value of the counter.
    *
    *******************************************************************************/
    uint16 Buzz_ReadCounter(void) 
    {
        /* Force capture by reading Accumulator */
        /* Must first do a software capture to be able to read the counter */
        /* It is up to the user code to make sure there isn't already captured data in the FIFO */
        (void)Buzz_COUNTERCAP_LSB;
        
        /* Read the data from the FIFO (or capture register for Fixed Function)*/
        return (CY_GET_REG16(Buzz_CAPTURE_LSB_PTR));
    }

        #if (Buzz_UseStatus)


            /*******************************************************************************
            * Function Name: Buzz_ClearFIFO
            ********************************************************************************
            * 
            * Summary:
            *  This function clears all capture data from the capture FIFO
            *
            * Parameters:  
            *  None
            *
            * Return: 
            *  None
            *
            *******************************************************************************/
            void Buzz_ClearFIFO(void) 
            {
                while(0u != (Buzz_ReadStatusRegister() & Buzz_STATUS_FIFONEMPTY))
                {
                    (void)Buzz_ReadCapture();
                }
            }

        #endif /* Buzz_UseStatus */

#endif /* !Buzz_UsingFixedFunction */


/*******************************************************************************
* Function Name: Buzz_WritePeriod
********************************************************************************
* 
* Summary:
*  This function is used to change the period of the counter.  The new period 
*  will be loaded the next time terminal count is detected.
*
* Parameters:  
*  period:  Period value. May be between 1 and (2^Resolution)-1.  A value of 0 
*           will result in the counter remaining at zero.
*
* Return: 
*  None
*
*******************************************************************************/
void Buzz_WritePeriod(uint16 period) 
{
    #if(Buzz_UsingFixedFunction)
        CY_SET_REG16(Buzz_PERIOD_LSB_PTR, (uint16)period);
    #else
        CY_SET_REG16(Buzz_PERIOD_LSB_PTR, period);
    #endif /* (Buzz_UsingFixedFunction) */
}

#if (Buzz_UseOneCompareMode)


    /*******************************************************************************
    * Function Name: Buzz_WriteCompare
    ********************************************************************************
    * 
    * Summary:
    *  This funtion is used to change the compare1 value when the PWM is in Dither
    *  mode. The compare output will reflect the new value on the next UDB clock. 
    *  The compare output will be driven high when the present counter value is 
    *  compared to the compare value based on the compare mode defined in 
    *  Dither Mode.
    *
    * Parameters:  
    *  compare:  New compare value.  
    *
    * Return: 
    *  None
    *
    * Side Effects:
    *  This function is only available if the PWM mode parameter is set to
    *  Dither Mode, Center Aligned Mode or One Output Mode
    *
    *******************************************************************************/
    void Buzz_WriteCompare(uint16 compare) \
                                       
    {	
		#if(Buzz_UsingFixedFunction)
			CY_SET_REG16(Buzz_COMPARE1_LSB_PTR, (uint16)compare);
		#else
	        CY_SET_REG16(Buzz_COMPARE1_LSB_PTR, compare);	
		#endif /* (Buzz_UsingFixedFunction) */
        
        #if (Buzz_PWMMode == Buzz__B_PWM__DITHER)
            #if(Buzz_UsingFixedFunction)
    			CY_SET_REG16(Buzz_COMPARE2_LSB_PTR, (uint16)(compare + 1u));
    		#else
    	        CY_SET_REG16(Buzz_COMPARE2_LSB_PTR, (compare + 1u));	
    		#endif /* (Buzz_UsingFixedFunction) */
        #endif /* (Buzz_PWMMode == Buzz__B_PWM__DITHER) */
    }


#else


    /*******************************************************************************
    * Function Name: Buzz_WriteCompare1
    ********************************************************************************
    * 
    * Summary:
    *  This funtion is used to change the compare1 value.  The compare output will 
    *  reflect the new value on the next UDB clock.  The compare output will be 
    *  driven high when the present counter value is less than or less than or 
    *  equal to the compare register, depending on the mode.
    *
    * Parameters:  
    *  compare:  New compare value.  
    *
    * Return: 
    *  None
    *
    *******************************************************************************/
    void Buzz_WriteCompare1(uint16 compare) \
                                        
    {
        #if(Buzz_UsingFixedFunction)
            CY_SET_REG16(Buzz_COMPARE1_LSB_PTR, (uint16)compare);
        #else
            CY_SET_REG16(Buzz_COMPARE1_LSB_PTR, compare);
        #endif /* (Buzz_UsingFixedFunction) */
    }


    /*******************************************************************************
    * Function Name: Buzz_WriteCompare2
    ********************************************************************************
    * 
    * Summary:
    *  This funtion is used to change the compare value, for compare1 output.  
    *  The compare output will reflect the new value on the next UDB clock.  
    *  The compare output will be driven high when the present counter value is 
    *  less than or less than or equal to the compare register, depending on the 
    *  mode.
    *
    * Parameters:  
    *  compare:  New compare value.  
    *
    * Return: 
    *  None
    *
    *******************************************************************************/
    void Buzz_WriteCompare2(uint16 compare) \
                                        
    {
        #if(Buzz_UsingFixedFunction)
            CY_SET_REG16(Buzz_COMPARE2_LSB_PTR, compare);
        #else
            CY_SET_REG16(Buzz_COMPARE2_LSB_PTR, compare);
        #endif /* (Buzz_UsingFixedFunction) */
    }
#endif /* UseOneCompareMode */

#if (Buzz_DeadBandUsed)


    /*******************************************************************************
    * Function Name: Buzz_WriteDeadTime
    ********************************************************************************
    * 
    * Summary:
    *  This function writes the dead-band counts to the corresponding register
    *
    * Parameters:  
    *  deadtime:  Number of counts for dead time 
    *
    * Return: 
    *  None
    *
    *******************************************************************************/
    void Buzz_WriteDeadTime(uint8 deadtime) 
    {
        /* If using the Dead Band 1-255 mode then just write the register */
        #if(!Buzz_DeadBand2_4)
            CY_SET_REG8(Buzz_DEADBAND_COUNT_PTR, deadtime);
        #else
            /* Otherwise the data has to be masked and offset */        
            /* Clear existing data */
            Buzz_DEADBAND_COUNT &= ((uint8)(~Buzz_DEADBAND_COUNT_MASK));
            
            /* Set new dead time */
            #if(Buzz_DEADBAND_COUNT_SHIFT)        
                Buzz_DEADBAND_COUNT |= ((uint8)((uint8)deadtime << Buzz_DEADBAND_COUNT_SHIFT)) & 
                                                    Buzz_DEADBAND_COUNT_MASK;
            #else
                Buzz_DEADBAND_COUNT |= deadtime & Buzz_DEADBAND_COUNT_MASK;        
            #endif /* (Buzz_DEADBAND_COUNT_SHIFT) */
        
        #endif /* (!Buzz_DeadBand2_4) */
    }


    /*******************************************************************************
    * Function Name: Buzz_ReadDeadTime
    ********************************************************************************
    * 
    * Summary:
    *  This function reads the dead-band counts from the corresponding register
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  Dead Band Counts
    *
    *******************************************************************************/
    uint8 Buzz_ReadDeadTime(void) 
    {
        /* If using the Dead Band 1-255 mode then just read the register */
        #if(!Buzz_DeadBand2_4)
            return (CY_GET_REG8(Buzz_DEADBAND_COUNT_PTR));
        #else
        
            /* Otherwise the data has to be masked and offset */
            #if(Buzz_DEADBAND_COUNT_SHIFT)      
                return ((uint8)(((uint8)(Buzz_DEADBAND_COUNT & Buzz_DEADBAND_COUNT_MASK)) >> 
                                                                           Buzz_DEADBAND_COUNT_SHIFT));
            #else
                return (Buzz_DEADBAND_COUNT & Buzz_DEADBAND_COUNT_MASK);
            #endif /* (Buzz_DEADBAND_COUNT_SHIFT) */
        #endif /* (!Buzz_DeadBand2_4) */
    }
#endif /* DeadBandUsed */

#if (Buzz_UseStatus || Buzz_UsingFixedFunction)


    /*******************************************************************************
    * Function Name: Buzz_SetInterruptMode
    ********************************************************************************
    * 
    * Summary:
    *  This function configures the interrupts mask control of theinterrupt 
    *  source status register.
    *
    * Parameters:  
    *  uint8 interruptMode: Bit field containing the interrupt sources enabled 
    *
    * Return: 
    *  None
    *
    *******************************************************************************/
    void Buzz_SetInterruptMode(uint8 interruptMode)  
    {
    	CY_SET_REG8(Buzz_STATUS_MASK_PTR, interruptMode);
    }


    /*******************************************************************************
    * Function Name: Buzz_ReadStatusRegister
    ********************************************************************************
    * 
    * Summary:
    *  This function returns the current state of the status register. 
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  uint8 : Current status register value. The status register bits are:
    *  [7:6] : Unused(0)
    *  [5]   : Kill event output
    *  [4]   : FIFO not empty
    *  [3]   : FIFO full
    *  [2]   : Terminal count
    *  [1]   : Compare output 2
    *  [0]   : Compare output 1
    *
    *******************************************************************************/
    uint8 Buzz_ReadStatusRegister(void)   
    {
    	uint8 result;
    	
    	result = CY_GET_REG8(Buzz_STATUS_PTR);
    	return (result);
    }

#endif /* (Buzz_UseStatus || Buzz_UsingFixedFunction) */


#if (Buzz_UseControl)


    /*******************************************************************************
    * Function Name: Buzz_ReadControlRegister
    ********************************************************************************
    * 
    * Summary:
    *  Returns the current state of the control register. This API is available 
    *  only if the control register is not removed.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  uint8 : Current control register value
    *
    *******************************************************************************/    
    uint8 Buzz_ReadControlRegister(void) 
    {
    	uint8 result;
    	
    	result = CY_GET_REG8(Buzz_CONTROL_PTR);
    	return (result);
    }


    /*******************************************************************************
    * Function Name: Buzz_WriteControlRegister
    ********************************************************************************
    * 
    * Summary:
    *  Sets the bit field of the control register. This API is available only if 
    *  the control register is not removed.
    *
    * Parameters:  
    *  uint8 control: Control register bit field, The status register bits are:
    *  [7]   : PWM Enable
    *  [6]   : Reset
    *  [5:3] : Compare Mode2
    *  [2:0] : Compare Mode2
    *
    * Return: 
    *  None
    *
    *******************************************************************************/  
    void Buzz_WriteControlRegister(uint8 control) 
    {
    	CY_SET_REG8(Buzz_CONTROL_PTR, control);
    }
	
#endif /* (Buzz_UseControl) */


#if (!Buzz_UsingFixedFunction)


    /*******************************************************************************
    * Function Name: Buzz_ReadCapture
    ********************************************************************************
    * 
    * Summary:
    *  Reads the capture value from the capture FIFO.
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  uint8/uint16: The current capture value
    *
    *******************************************************************************/  
    uint16 Buzz_ReadCapture(void)  
    {
    	return (CY_GET_REG16(Buzz_CAPTURE_LSB_PTR));
    }
	
#endif /* (!Buzz_UsingFixedFunction) */


#if (Buzz_UseOneCompareMode)


    /*******************************************************************************
    * Function Name: Buzz_ReadCompare
    ********************************************************************************
    * 
    * Summary:
    *  Reads the compare value for the compare output when the PWM Mode parameter is 
    *  set to Dither mode, Center Aligned mode, or One Output mode.
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  uint8/uint16: Current compare value
    *
    *******************************************************************************/  
    uint16 Buzz_ReadCompare(void)  
    {
		#if(Buzz_UsingFixedFunction)
            return ((uint16)CY_GET_REG16(Buzz_COMPARE1_LSB_PTR));
        #else
            return (CY_GET_REG16(Buzz_COMPARE1_LSB_PTR));
        #endif /* (Buzz_UsingFixedFunction) */
    }

#else


    /*******************************************************************************
    * Function Name: Buzz_ReadCompare1
    ********************************************************************************
    * 
    * Summary:
    *  Reads the compare value for the compare1 output.
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  uint8/uint16: Current compare value.
    *
    *******************************************************************************/  
    uint16 Buzz_ReadCompare1(void) 
    {
		return (CY_GET_REG16(Buzz_COMPARE1_LSB_PTR));
    }


    /*******************************************************************************
    * Function Name: Buzz_ReadCompare2
    ********************************************************************************
    * 
    * Summary:
    *  Reads the compare value for the compare2 output.
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  uint8/uint16: Current compare value.
    *
    *******************************************************************************/  
    uint16 Buzz_ReadCompare2(void)  
    {
		return (CY_GET_REG16(Buzz_COMPARE2_LSB_PTR));
    }

#endif /* (Buzz_UseOneCompareMode) */


/*******************************************************************************
* Function Name: Buzz_ReadPeriod
********************************************************************************
* 
* Summary:
*  Reads the period value used by the PWM hardware.
*
* Parameters:  
*  None
*
* Return: 
*  uint8/16: Period value
*
*******************************************************************************/ 
uint16 Buzz_ReadPeriod(void) 
{
	#if(Buzz_UsingFixedFunction)
        return ((uint16)CY_GET_REG16(Buzz_PERIOD_LSB_PTR));
    #else
        return (CY_GET_REG16(Buzz_PERIOD_LSB_PTR));
    #endif /* (Buzz_UsingFixedFunction) */
}

#if ( Buzz_KillModeMinTime)


    /*******************************************************************************
    * Function Name: Buzz_WriteKillTime
    ********************************************************************************
    * 
    * Summary:
    *  Writes the kill time value used by the hardware when the Kill Mode 
    *  is set to Minimum Time.
    *
    * Parameters:  
    *  uint8: Minimum Time kill counts
    *
    * Return: 
    *  None
    *
    *******************************************************************************/ 
    void Buzz_WriteKillTime(uint8 killtime) 
    {
    	CY_SET_REG8(Buzz_KILLMODEMINTIME_PTR, killtime);
    }


    /*******************************************************************************
    * Function Name: Buzz_ReadKillTime
    ********************************************************************************
    * 
    * Summary:
    *  Reads the kill time value used by the hardware when the Kill Mode is set 
    *  to Minimum Time.
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  uint8: The current Minimum Time kill counts
    *
    *******************************************************************************/ 
    uint8 Buzz_ReadKillTime(void) 
    {
    	return (CY_GET_REG8(Buzz_KILLMODEMINTIME_PTR));
    }

#endif /* ( Buzz_KillModeMinTime) */

/* [] END OF FILE */
