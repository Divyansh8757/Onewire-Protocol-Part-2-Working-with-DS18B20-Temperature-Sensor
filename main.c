#include "main.h"
#include <stdio.h>

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USB_DRD_FS_HCD_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

uint16_t DSB_Read();						// Function to Read from the sensor
DSB_Write(uint8_t data);					// Function to write commands to the sensor
uint8_t DSB_Start();						// Function to initialize the sensor
void SET_PIN_Out();						// Function to set GPIO pin to Output mode
void SET_PIN_In();						// Function to set GPIO pin to Input mode
void delay(int us);						// Function to create delay in microseconds


int checktime=0;						// Variable to count microseconds delay
uint8_t res=0;							 
uint16_t Raw=0;							// Variable to store raw 16 bit data that is read from the sensor
uint8_t Response=0;						// Variable to store response after initialization of the sensor
int i=0;							
int intT=0;							// Variable to store Integer part of the temperature
int decT=0;							// Variable to store Decimal part of the temperature
float temp=0.0;							// Variable to store converted temperature value
char message[64];						// Variable to display message through UART
uint8_t error[64]
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  SystemPower_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USB_DRD_FS_HCD_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	
  HAL_TIM_Base_Start(&htim16);							// Timer to count delays in microseconds

	
  while (1)
  {
	  	Response = DSB_Start();						// Initialize the sensor
	   	if(Response == 1){
	  	DSB_Write(0xCC);						// Write SKIP ROM command to the sensor
	  	DSB_Write(0x44);						// Write CONVERT T command to the sensor. This command tells sensor to read temperature and store it
	  	HAL_Delay(750);							// Delay to let the sensor complete the read and convert the temperature value
	  	DSB_Start();							// Restart the sensor
	  	DSB_Write(0xCC);						// Write SKIP ROM Command
	  	DSB_Write(0xBE);						// Write READ ROM Command
	  	Raw=DSB_Read();							// Read the raw 16 bit converted value from the sensor
	  	intT = Raw >> 4;						// Last 4 bits are for the decimal value of the tempersature
	  	decT = Raw & 0x000F;						// Store the last 4 bits of decimal part
	  	temp = intT+ (decT/12.0);					// Final Conversion of temperature in float
	  	sprintf(message, "\x1b[2J \x1b[HTemperature = %.3f C", temp);		// Escape characters with respect to Putty terminal window	
	  	HAL_UART_Transmit(&huart1, (uint8_t*)message, sizeof(message), 10);	// Display the converted value of tempertature
	  	HAL_Delay(3000);
		}

	  else{
		sprintf(error, "Sensor Initialization Failed\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)error, sizeof(error), 10);		// Print Error Message
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_GPIO;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 160-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_CRS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Sensor Finctions start here 

void delay(int us){							// Fumction to count microseconds delay
__HAL_TIM_SET_COUNTER(&htim16,0);
while((__HAL_TIM_GET_COUNTER(&htim16))<us){}
}

void SET_PIN_In(){							// Function to set GPIO pin to Input mode
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SET_PIN_Out(){							// Function to set GPIO pin to Output mode
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t DSB_Start(){							// Function to Initialize the Sensor
	 uint8_t Presence = 0;						// Return variable
	 SET_PIN_Out();							// Set GPIO pin as Output
	 __HAL_TIM_SET_COUNTER(&htim16,0);
	 do{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, 0);			// Write Pin High for 480 microseconds
	 }while((__HAL_TIM_GET_COUNTER(&htim16))<=480);
	 SET_PIN_In();							// Set GPIO pin as Input
	 delay(60);							// Relaxation time of 60 microseconds
	 __HAL_TIM_SET_COUNTER(&htim16,0);
	 do{
		 checktime=__HAL_TIM_GET_COUNTER(&htim16);	
	 }while(!(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)));			// Check if the Input is low for more than 60 microseconds and less the 240 microseconds

	 if(checktime>=60 && checktime<=240)
		 Presence=1;						// If conditions are true that means sensor is working fine
	 else
		 Presence=-1;
	 delay(480-checktime);						// Deleay for a total of 480 microseconds including reading input time 
	 return Presence;
 }

DSB_Write(uint8_t data){						// Function to Write Commands to sensor
SET_PIN_In();								// Set Gpio pin as Input
delay(5);
if(!(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)))				// if pin is high that means sensor is either busy or not responding
	return -1;
for(i=0;i<8;i++){				
	if((data&(1<<i))!=0){						// To write 0
		SET_PIN_Out();						// Set GPIO pin as Output
		delay(2);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,0);			// Pull the pin low for 10 microseconds and release the pin
		delay(10);						
		SET_PIN_In();						// Set pin Input to release
		delay(55);						
	}	
	else{								// To write 1
		SET_PIN_Out();						// Set GPIO pin as Output 
		delay(2);						
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,0);			// Pull the pin low for 60 seconds
		delay(60);
	}
	SET_PIN_In();
	delay(5);
	}
}

uint16_t DSB_Read(){							// Function to read from the sensor
uint16_t value=0;							// create a return variable
for(i=0;i<15;i++){						
	SET_PIN_Out();							// Set GPIO pin as Output
	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);				// Pull the pin low for 15 microseconds and then release
	delay(15);
	SET_PIN_In();							// Set GPIO pin as Input to release
	delay(5);

	if(!(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))){			// if the sensor pulls the pin low that means it is transmitting 0
	value |= (0<<i);
	while(!(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))){}			// wait untill pin is pulled back to high
	}
	else{								// else if the sensor pulls the pin high that means it is transmitting 1
		value |= (1<<i);
		delay(50);						// pulls hig for 50 microseconds or less 
	}
}
return value;								// return value
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
