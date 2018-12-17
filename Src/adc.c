/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ADC init function */
void MX_ADC_Init(void)
{
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**ADC GPIO Configuration  
  PA0   ------> ADC_IN0 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC DMA Init */
  
  /* ADC Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_0);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* ADC interrupt Init */
  NVIC_SetPriority(ADC1_COMP_IRQn, 1);
  NVIC_EnableIRQ(ADC1_COMP_IRQn);

  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
  /**Common config 
  */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

}

/* USER CODE BEGIN 1 */

/**
  * @brief  Configure ADC (ADC instance: ADC1) and GPIO used by ADC channels.
  * @note   In case re-use of this function outside of this example:
  *         This function includes checks of ADC hardware constraints before
  *         executing some configuration functions.
  *         - In this example, all these checks are not necessary but are
  *           implemented anyway to show the best practice usages
  *           corresponding to reference manual procedure.
  *           (On some STM32 series, setting of ADC features are not
  *           conditioned to ADC state. However, in order to be compliant with
  *           other STM32 series and to show the best practice usages,
  *           ADC state is checked anyway with same constraints).
  *           Software can be optimized by removing some of these checks,
  *           if they are not relevant considering previous settings and actions
  *           in user application.
  *         - If ADC is not in the appropriate state to modify some parameters,
  *           the setting of these parameters is bypassed without error
  *           reporting:
  *           it can be the expected behavior in case of recall of this 
  *           function to update only a few parameters (which update fullfills
  *           the ADC state).
  *           Otherwise, it is up to the user to set the appropriate error 
  *           reporting in user application.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void adcConfigure_ADC(void)
{
  /*## Configuration of GPIO used by ADC channels ############################*/
  
  /* Note: On this STM32 device, ADC1 channel 0 is mapped on GPIO pin PA.00 */ 
  
  /* Enable GPIO Clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure GPIO in analog mode to be used as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
  
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable ADC1 interruptions */
  NVIC_SetPriority(ADC1_COMP_IRQn, 0); /* ADC IRQ greater priority than DMA IRQ */
  NVIC_EnableIRQ(ADC1_COMP_IRQn);
  
  /*## Configuration of ADC ##################################################*/
  
  /*## Configuration of ADC hierarchical scope: common to several ADC ########*/
  
  /* Enable ADC clock (core clock) */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       All ADC instances of the ADC common group must be disabled.        */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if(__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */
    
    /* Set ADC clock (conversion clock) common to several ADC instances */
    /* Note: On this STM32 serie, ADC common clock asynchonous prescaler      */
    /*       is applied to each ADC instance if ADC instance clock is         */
    /*       set to clock source asynchronous                                 */
    /*       (refer to function "LL_ADC_SetClock()" below).                   */
    // LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_ASYNC_DIV1);
    
    /* Set ADC measurement path to internal channels */
    // LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);
    
    
  /*## Configuration of ADC hierarchical scope: multimode ####################*/
  
    /* Note: Feature not available on this STM32 serie */ 
    
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC instance #################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */
    
    /* Set ADC clock (conversion clock) */
    LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
    
    /* Set ADC data resolution */
    // LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    
    /* Set ADC conversion data alignment */
    // LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    
    /* Set ADC low power mode */
    // LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
    
    /* Set ADC channels sampling time */
    /* Note: On this STM32 serie, sampling time is common to all channels     */
    /*       of the entire ADC instance.                                      */
    /*       Therefore, sampling time is configured here under ADC instance   */
    /*       scope (not under channel scope as on some other STM32 devices    */
    /*       on which sampling time is channel wise).                         */
    /* Note: Considering interruption occurring after each number of          */
    /*       "ADC_CONVERTED_DATA_BUFFER_SIZE" ADC conversions                 */
    /*       (IT from DMA transfer complete),                                 */
    /*       select sampling time and ADC clock with sufficient               */
    /*       duration to not create an overhead situation in IRQHandler.      */
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
    
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC group regular ############*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled or enabled without conversion on going        */
  /*       on group regular.                                                  */
  if ((LL_ADC_IsEnabled(ADC1) == 0)               ||
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    /* Set ADC group regular trigger source */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM2_TRGO);
    
    /* Set ADC group regular trigger polarity */
    // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
    
    /* Set ADC group regular continuous mode */
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    
    /* Set ADC group regular conversion data transfer */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    
    /* Set ADC group regular overrun behavior */
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    
    /* Set ADC group regular sequencer */
    /* Note: On this STM32 serie, ADC group regular sequencer is              */
    /*       not fully configurable: sequencer length and each rank           */
    /*       affectation to a channel are fixed by channel HW number.         */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_REG_SetSequencerChannels()".                             */
    
    /* Set ADC group regular sequencer discontinuous mode */
    // LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    
    /* Set ADC group regular sequence: channel on rank corresponding to       */
    /* channel number.                                                        */
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_0);
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC group injected ###########*/
  
  /* Note: Feature not available on this STM32 serie */ 
  
  
  /*## Configuration of ADC hierarchical scope: channels #####################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled or enabled without conversion on going        */
  /*       on either groups regular or injected.                              */
  if ((LL_ADC_IsEnabled(ADC1) == 0)               ||
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    /* Set ADC channels sampling time */
    /* Note: On this STM32 serie, sampling time is common to all channels     */
    /*       of the entire ADC instance.                                      */
    /*       See sampling time configured above, at ADC instance scope.       */
    
  }
  
  
  /*## Configuration of ADC transversal scope: analog watchdog ###############*/
  
  /* Note: On this STM32 serie, there is only 1 analog watchdog available.    */
  
  /* Set ADC analog watchdog: channels to be monitored */
  // LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_DISABLE);
  
  /* Set ADC analog watchdog: thresholds */
  // LL_ADC_ConfigAnalogWDThresholds(ADC1, __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B), 0x000);
  
  
  /*## Configuration of ADC transversal scope: oversampling ##################*/
  
  /* Set ADC oversampling scope */
  // LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  
  /* Set ADC oversampling parameters */
  // LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_2, LL_ADC_OVS_SHIFT_NONE);
  
  
  /*## Configuration of ADC interruptions ####################################*/
  /* Enable interruption ADC group regular overrun */
  LL_ADC_EnableIT_OVR(ADC1);
  
  /* Note: in this example, ADC group regular end of conversions              */
  /*       (number of ADC conversions defined by DMA buffer size)             */
  /*       are notified by DMA transfer interruptions).                       */
  
}

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @note   Operations:
  *         - ADC instance
  *           - Run ADC self calibration
  *           - Enable ADC
  *         - ADC group regular
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  *         - ADC group injected
  *           Feature not available                                  (feature not available on this STM32 serie)
  * @param  None
  * @retval None
  */
void adcActivate_ADC(void)
{
  __IO uint32_t wait_loop_index = 0;
  __IO uint32_t backup_setting_adc_dma_transfer = 0;
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
  
  /*## Operation on ADC hierarchical scope: ADC instance #####################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Disable ADC DMA transfer request during calibration */
    /* Note: Specificity of this STM32 serie: Calibration factor is           */
    /*       available in data register and also transfered by DMA.           */
    /*       To not insert ADC calibration factor among ADC conversion data   */
    /*       in array variable, DMA transfer must be disabled during          */
    /*       calibration.                                                     */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    
    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1);
    
    /* Poll for ADC effectively calibrated */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }
    
    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    /* Restore ADC DMA transfer request after calibration */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
    
    /* Enable ADC */
    LL_ADC_Enable(ADC1);
    
    /* Poll for ADC ready to convert */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_ENABLE_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }
    
    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
    /*       status afterwards.                                               */
    /*       This flag should be cleared at ADC Deactivation, before a new    */
    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
  }
  
  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */
  
  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: Feature not available on this STM32 serie */ 
  
}

/* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the function          */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of this feature is conditioned to     */
  /*       ADC state:                                                         */
  /*       ADC must be enabled without conversion on going on group regular,  */
  /*       without ADC disable command on going.                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.            																		*/
void adcStart(void)
{
	if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
      (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    LL_ADC_REG_StartConversion(ADC1);
  }
  else
  {
    /* Error: ADC conversion start could not be performed */
     Error_Handler();
  }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
