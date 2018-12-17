/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*defines for SWD terminal*/

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 5000

/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       ((uint32_t)3300)



/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

/* number of ADC samples for averaging */
#define NUM_TO_AVERAGE 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*variables for SWD terminal*/
	struct __FILE { int handle; /* Add whatever you need here */ };
	FILE __stdout;
	FILE __stdin;
	
	volatile uint8_t printfFlag = 0;
	int fputc(int ch, FILE *f);

/* Variables for calculating average value */
 uint32_t averSum = 0 ; 
 uint32_t averCounter = 0;

/* Variables for ADC conversion data */
__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data */

/* Variables for ADC conversion data computation to physical values */
/* Value of voltage calculated from ADC conversion data (unit: mV) */
__IO uint16_t aADCxConvertedData_Voltage_mVolt; 

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not yet been started yet (initial state)              */
__IO uint8_t ubDmaTransferStatus = 2; /* Variable set into DMA interruption callback */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	
	


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	
	MX_GPIO_Init();
 
  /* USER CODE BEGIN 2 */
	
	/* Init variable containing ADC conversion data */
	aADCxConvertedData[0] = VAR_CONVERTED_DATA_INIT_VALUE;
	
	/* Configure DMA for data transfer from ADC */
  dmaConfigure_DMA();
	
	/* Configure timer as a time base used to trig ADC conversion start */
  timConfigure_TIM_TimeBase_ADC_trigger();
	
	/* Configure ADC */
  /* Note: This function configures the ADC but does not enable it.           */
  /*       To enable it, use function "Activate_ADC()".                       */
  /*       This is intended to optimize power consumption:                    */
  /*       1. ADC configuration can be done once at the beginning             */
  /*          (ADC disabled, minimal power consumption)                       */
  /*       2. ADC enable (higher power consumption) can be done just before   */
  /*          ADC conversions needed.                                         */
  /*          Then, possible to perform successive "Activate_ADC()",          */
  /*          "Deactivate_ADC()", ..., without having to set again            */
  /*          ADC configuration.                                              */
  adcConfigure_ADC();
	
	/* Configure DAC channel */
  dacConfigure_DAC();
	
	/* Activate DAC channel */
  dacActivate_DAC();
	
	/* Activate ADC */
  /* Perform ADC activation procedure to make it ready to convert. */
  adcActivate_ADC();
	
	/* Update status variable of DMA transfer */
  ubDmaTransferStatus = 0;
	
	adcStart();
	
	printf("Start SWD \r ");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*send data to debug SWD terminal every second*/
		if(printfFlag)
		{
			printfFlag = 0;
			printf("%dmV \r ", aADCxConvertedData_Voltage_mVolt);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {
    
  }
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_5);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {
  
  }
  LL_Init1msTick(2097000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(2097000);
}

/* USER CODE BEGIN 4 */

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferComplete_Callback()
{
	/* Set the data to be loaded in the data holding register */
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, aADCxConvertedData[0]);
    
  /* Trig DAC conversion by software */
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
  averCounter++;
	averSum += aADCxConvertedData[0];
  if(averCounter == NUM_TO_AVERAGE)
	{
		/* Computation of ADC conversions raw data to physical values               */
		/* using LL ADC driver helper macro.                                        */
		aADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, averSum/NUM_TO_AVERAGE, LL_ADC_RESOLUTION_12B);
		averCounter = 0;
		averSum = 0;
		printfFlag = 1;     // to send data to SWD terminal
		gpioLED_Toggle();
  }
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;
}


/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void AdcDmaTransferError_Callback()
{
  /* Error detected during DMA transfer */
  Error_Handler();
}
/**
  * @brief  ADC group regular overrun interruption callback
  * @note   This function is executed when ADC group regular
  *         overrun error occurs.
  * @retval None
  */
void AdcGrpRegularOverrunError_Callback(void)
{
  /* Note: Disable ADC interruption that caused this error before entering in */
  /*       infinite loop below.                                               */
  
  /* Disable ADC group regular overrun interruption */
  LL_ADC_DisableIT_OVR(ADC1);
  
  /* Error from ADC */
  Error_Handler();
}

/**
  * @brief  DAC underrun interruption callback
  * @note   This function is executed when DAC channel underrun error occurs.
  * @retval None
  */
void DacUnderrunError_Callback(void)
{
  /* Note: Disable DAC interruption that caused this error before entering in */
  /*       infinite loop below.                                               */
  
  /* Disable interruption DAC channel1 underrun */
  LL_DAC_DisableIT_DMAUDR1(DAC1);
  
  /* Error from DAC */
  Error_Handler();
}


/**
  * @brief for debugging via SWO using printf
  * @note  
  * @retval None
  */
int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) 
	 {
			while (ITM_Port32(0) == 0);
			ITM_Port8(0) = ch;
	 }
   return(ch);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	gpioLED_Blinking(LED_BLINK_ERROR);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
