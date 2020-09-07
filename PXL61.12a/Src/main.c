/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : 6btn keypap
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;


/* USER CODE BEGIN PV */
/* Private define ------------------------------------------------------------*/
//---------------------------RGB1 MACROS
#define TEST_HIGH (GPIOA-> BSRR = GPIO_PIN_1)
#define TEST_LOW (GPIOA-> BRR = GPIO_PIN_1)

#define CLK1_HIGH (GPIOA->BSRR = GPIO_PIN_10)   /* STP16CPC26_CLK1 PIN (Clock input)  */
#define CLK1_LOW (GPIOA->BRR = GPIO_PIN_10)

#define LE1_HIGH (GPIOA->BSRR = GPIO_PIN_11)    /* STP16CPC26_LE1 PIN (Latch input)   */
#define LE1_LOW (GPIOA->BRR = GPIO_PIN_11)

#define SDI1_HIGH (GPIOA->BSRR = GPIO_PIN_15)   /* STP16CPC26_SDI1 PIN (Data input)   */
#define SDI1_LOW (GPIOA->BRR = GPIO_PIN_15)

#define OE1_HIGH (GPIOB->BSRR = GPIO_PIN_7)     /* STP16CPC26_OE1 PIN (output enable) */
#define OE1_LOW (GPIOB->BRR = GPIO_PIN_7)

//---------------------------RGB2 MACROS
#define CLK2_HIGH (GPIOB->BSRR = GPIO_PIN_0)    /* STP16CPC26_CLK2 PIN (Clock input)  */
#define CLK2_LOW (GPIOB->BRR = GPIO_PIN_0)

#define LE2_HIGH (GPIOA->BSRR = GPIO_PIN_4)     /* STP16CPC26_LE2 PIN (Latch input)   */
#define LE2_LOW (GPIOA->BRR = GPIO_PIN_4)

#define SDI2_HIGH (GPIOB->BSRR = GPIO_PIN_1)    /* STP16CPC26_SDI2 PIN (Data input)   */
#define SDI2_LOW (GPIOB->BRR = GPIO_PIN_1)

#define OE2_HIGH (GPIOA->BSRR = GPIO_PIN_9)     /* STP16CPC26_OE2 PIN (output enable) */
#define OE2_LOW (GPIOA->BRR = GPIO_PIN_9)       

//---------------------------OUT MACROS
#define SW2_OUTPUT_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET)
#define SW2_OUTPUT_LOW HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET)

#define SW4_OUTPUT_HIGH HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define SW4_OUTPUT_LOW HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)

#define SW5_OUTPUT_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET)
#define SW5_OUTPUT_LOW HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET)

#define SW6_OUTPUT_HIGH HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET)
#define SW6_OUTPUT_LOW HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET)

/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC0ConvertedVoltage = 0, ADC1ConvertedVoltage = 0; /* ADC voltage conversion results */
static __IO uint32_t TimingDelay; /* SysTick timer global variable */

uint8_t i;
uint16_t button_pressed;        /* button pressed variables */
uint16_t d;                     /* STP16CPC26 Data input value: display on LEDs*/
uint16_t RGB1416, RGB2324;      /* ADC LED display variables */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void BATTERY_TEST(void); /* Battery test routine */
void config_ext_channel_ADC(uint32_t channel, uint8_t val); /* ADC Configuration function */
void Delay(__IO uint32_t nTime); /* Delay function using SysTick */
uint32_t r_single_ext_channel_ADC(uint32_t channel); /* ADC Conversion function */
void RGB1_SETUP(void); /* LEDS management function */
void RGB2_SETUP(void); /* LEDS management function */
//void SET_SDA_INPUT(void);
//void SET_SDA_OUTPUT(void);
void TimingDelay_Decrement(void); /* Decrements the TimingDelay variable */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
   
  if (SysTick_Config(SystemCoreClock / 1000))  
  { 
    while (1);  /* Capture error */
  } /*EndIf */
  
  /* USER CODE BEGIN 2 */
//  RGB1416 = 0x0400;
//  RGB2324 = 0x1000;
    RGB1416 = 0x2886;
    RGB2324 = 0x4824;
  
  button_pressed = 0;
  SW2_OUTPUT_LOW;
  SW4_OUTPUT_LOW;
  SW5_OUTPUT_LOW;
  SW6_OUTPUT_LOW;
  
  BATTERY_TEST();
  
  //-------------------------d = 0x0402;
  //d = 0x0002 + RGB1416;
  
  d = 0100+ 1000 + 0010 + 0100;
  RGB1_SETUP();
  //------------------------d = 0x9024;
  //d = 0x8024 + RGB2324;
  d = 0x4824 + RGB2324;
  RGB2_SETUP();
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    BATTERY_TEST();
     
    /*switch (button_pressed)
    {
      case 0 :
        //d = 0x0402;
        d = 0x0002  + RGB1416;
        RGB1_SETUP();
        //d = 0x9024;
        d = 0x8024 + RGB2324;
        RGB2_SETUP();
        SW2_OUTPUT_LOW;
        SW4_OUTPUT_LOW;
        SW5_OUTPUT_LOW;
        SW6_OUTPUT_LOW;
        break;
        
      case 1 : // SW1---sal light
        break;
      
      case 2 : // SW2---susp up
        //d = 0x0401;
        d = 0x0001  + RGB1416;
        RGB1_SETUP();
        //d = 0x9024;
        d = 0x8024 + RGB2324; 
        RGB2_SETUP();
        SW2_OUTPUT_HIGH;
        break;
        
      case 3: // SW3---rear scene
        break;
      
      case 4: // SW4----susp down
        //d = 0x0402;
        d = 0x0002  + RGB1416;
        RGB1_SETUP();
        //d = 0x5024;
        d = 0x4024 + RGB2324;     
        RGB2_SETUP();
        SW4_OUTPUT_HIGH;
        break;
    
      case 5: // SW5----ramp out
        //d = 0x0402;
        d = 0x0002  + RGB1416;
        RGB1_SETUP();
        //d = 0x9022;
        d = 0x8022 + RGB2324;
        RGB2_SETUP();
        SW5_OUTPUT_HIGH;
        break;
    
      case 6: // SW6-----ramp in
        //d = 0x0402;
        d = 0x0002  + RGB1416;
        RGB1_SETUP();
        //d = 0x9014;
        d = 0x8014 + RGB2324;
        RGB2_SETUP();
        SW6_OUTPUT_HIGH;
        break;
      
      default :
        //d = 0x0402;
        d = 0x0002  + RGB1416;
        RGB1_SETUP();
        //d = 0x9024;
        d = 0x8024 + RGB2324;
        RGB2_SETUP();
        SW2_OUTPUT_LOW;
        SW4_OUTPUT_LOW;
        SW5_OUTPUT_LOW;
        SW6_OUTPUT_LOW;
        break;      
    } /* EndOfSwitch */
    
    Delay(100);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */    
  }/* EndofWhile */
  
  /* USER CODE END 3 */
} /* EndOfMainFunction */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)


  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    
      HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

// TX_DATA, DATA_IN, SW6, SW5, SW4, CLOCK, , SW3
  GPIO_InitStruct.Pin = GPIO_PIN_2 |GPIO_PIN_3 |GPIO_PIN_5 |GPIO_PIN_6 |GPIO_PIN_12;
  //GPIO_InitStruct.Pin = GPIO_PIN_2 |GPIO_PIN_3 |GPIO_PIN_5 |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 // LED11, LED9, LED8, LED7
  GPIO_InitStruct.Pin = GPIO_PIN_1 |GPIO_PIN_4 | GPIO_PIN_7 |GPIO_PIN_8 |GPIO_PIN_9 |GPIO_PIN_10 |GPIO_PIN_11 |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Configure GPIO pin : PA0 | PA1 
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 
  // SW3, SW2
  GPIO_InitStruct.Pin = GPIO_PIN_3 |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  
  /*Configure GPIO pins : OE2_Pin OE1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0 |GPIO_PIN_1 |GPIO_PIN_5 |GPIO_PIN_6 |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);
  
  /**Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

/* USER CODE BEGIN 4 */
//------------------BATTERY TEST ROUTINE FUNCTION
/**
  * @brief  None
  * @param  None
  * @retval None
  */
void BATTERY_TEST(void)
{
  ADC0ConvertedVoltage = r_single_ext_channel_ADC(ADC_CHANNEL_1);
  ADC1ConvertedVoltage = r_single_ext_channel_ADC(ADC_CHANNEL_0);
  
  if (ADC0ConvertedVoltage < 800)
  {
    RGB1416 = 0x0400;
  } /*EndIf */
  else {
    RGB1416 = 0x0200;
  } /*EndElse */
  
  
  if (ADC1ConvertedVoltage < 800)
  {
    RGB2324 = 0x1000;
  } /*EndIf */
  else {
    RGB2324 = 0x0800;
  } /*EndElse */   
} /* End BATTERY_TEST */

/**
  * @brief  ADC Configuration function
  * @param  channel
  * @param  val
  * @retval None
  */
void config_ext_channel_ADC(uint32_t channel, uint8_t val)
{
  ADC_ChannelConfTypeDef sConfig;
  
  sConfig.Channel = channel;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  
  if(1 == val)
  {
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  } /*EndIf */
  else
  {
    sConfig.Rank = ADC_RANK_NONE;
  } /*EndElse */
  
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
} /* End config_ext_channel_ADC */

/**
  * @brief  None
  * @param  nTime
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
} /* End Delay */

/**
  * @brief  ADC Conversion function
  * @param  channel
  * @retval digital_result
  */
uint32_t r_single_ext_channel_ADC(uint32_t channel)
{
  uint32_t digital_result;
  
  config_ext_channel_ADC(channel, 1);
  
  HAL_ADCEx_Calibration_Start(&hadc);
  
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 100);
  digital_result = HAL_ADC_GetValue(&hadc);
  digital_result = (digital_result *3300)/0xFFF;
  HAL_ADC_Stop(&hadc);
  
  config_ext_channel_ADC(channel, 0);
  
  return digital_result;
} /* End r_single_ext_channel_ADC */

//-----------RGB1 FUNCTION
/**
  * @brief  Display function using STP16CP26 (U3)
  * @note   Controls upper LEDs
  * @param  None
  * @retval None
  */
void RGB1_SETUP(void)
{
    LE1_LOW; /* hold of the previous values */
    OE1_HIGH; /* all outputs are switched off */
    SDI1_HIGH;
    CLK1_LOW;
    //d1 = d; 
  
    for(i = 0; i < 16; i++)
    {
        if(d & 0x01)
        {
           SDI1_HIGH;
        } /*EndIf */
        else
        {
            SDI1_LOW;
        } /*EndElse */
 
        /* On every rising edge of CLK, a new data on SDI pin is sampled */
        CLK1_HIGH;
        CLK1_LOW;
        
        if (i ==15) /* All bits have been loaded in the shift register */
        {       
            LE1_HIGH;
            LE1_LOW;
        } /*EndIf */
        
        d = d >> 1;                
     } /* EndFor loop */

    OE1_LOW; /* outputs OUT0 to OUT15 are written */
} /* End RGB1_SETUP */

//-----------RGB2 FUNCTION
/**
  * @brief  Display function using STP16CP26 (U4)
  * @note   Controls lower LEDs
  * @param  None
  * @retval None
  */
void RGB2_SETUP(void)
{
    LE2_LOW; /* hold of the previous values */
    OE2_HIGH; /* all outputs are switched off */
    SDI2_HIGH;
    CLK2_LOW;
   
    for(i = 0; i < 16; i++)
    {
        if(d & 0x01)
        {
           SDI2_HIGH;
        } /*EndIf */
        else
        {
            SDI2_LOW;
        } /*EndElse */
 
        /* On every rising edge of CLK, a new data on SDI pin is sampled */
        CLK2_HIGH;
        CLK2_LOW;
        
        if (i ==15) /* All bits have been loaded in the shift register */
        {       
            LE2_HIGH;
            LE2_LOW;
        } /*EndIf */
        
        d = d >> 1;                
     } /* EndFor loop */

    OE2_LOW; /* outputs OUT0 to OUT15 are written */
} /* End RGB2_SETUP */

//-------------DIGITAL POTENTIOMETER SETUP
/**
  * @brief  None
  * @param  None
  * @retval None
  */
/*void SET_SDA_INPUT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}*/ /* End SET_SDA_INPUT */


//------------DIGI POT FUNCTION
/**
  * @brief  None
  * @param  None
  * @retval None
  */
/*void SET_SDA_OUTPUT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}*/ /* End SET_SDA_OUTPUT */


/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  } /*EndIf */
} /* End TimingDelay_Decrement*/

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
