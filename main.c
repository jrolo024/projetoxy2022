/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCC_AHBENR        0x4002381C
#define GPIOC_MODER        0x40020800
#define GPIOC_IDR        0x40020810
#define RCC_APB2        0x40023820
#define SYSCFG_EXTICR4    0x40010014
#define EXTI_IMR        0x40010400
#define EXTI_RTSR        0x40010408
#define NVIC_ISER       0xE000E104
#define EXTI_PR            0x40010414
#define TIM2_CR1        0x40000000
#define TIM2_DIER1        0x4000000C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned int alarm_low=30,alarm_high=130;
unsigned int time1=0,j=0, test2=0,time2=0,test=0,bpm=0,etat=1,ac_cap=0,ech=0,k=1,cpt_ech=0,i=0;
const int8_t tab_ech[1000] = {0,-1,-1,-1,-2,-2,-2,-2,-3,-3,-3,-3,-4,-4,-4,-4,-5,-5,-5,-5,-6,-6,-6,-6,-7,-7,-7,-7,-7,-8,-8,-8,-8,-9,-9,-9,-9,-9,-10,-10,-10,-10,-10,-10,-11,-11,-11,-11,-11,-11,-12,-12,-12,-12,-12,-12,-12,-12,-12,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-12,-12,-12,-12,-12,-12,-12,-12,-11,-11,-11,-11,-11,-10,-10,-10,-10,-10,-9,-9,-9,-9,-8,-8,-8,-7,-7,-7,-7,-6,-6,-6,-5,-5,-4,-4,-4,-3,-3,-3,-2,-2,-1,-1,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,8,8,9,9,10,10,11,12,12,13,13,14,15,15,16,17,17,18,18,19,20,20,21,22,23,23,24,25,25,26,27,28,28,29,30,31,31,32,33,34,34,35,36,37,37,38,39,40,41,41,42,43,44,45,45,46,47,48,49,49,50,51,52,53,54,54,55,56,57,58,59,59,60,61,62,63,64,64,65,66,67,68,69,69,70,71,72,73,73,74,75,76,77,78,78,79,80,81,82,82,83,84,85,85,86,87,88,89,89,90,91,92,92,93,94,94,95,96,97,97,98,99,99,100,101,101,102,103,103,104,105,105,106,107,107,108,108,109,110,110,111,111,112,112,113,113,114,114,115,115,116,116,117,117,118,118,119,119,119,120,120,121,121,121,122,122,122,123,123,123,123,124,124,124,124,125,125,125,125,126,126,126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,126,126,126,126,126,126,126,125,125,125,125,124,124,124,124,123,123,123,122,122,122,121,121,121,120,120,119,119,118,118,117,117,116,116,115,115,114,114,113,113,112,111,111,110,110,109,108,108,107,106,106,105,104,103,103,102,101,100,99,99,98,97,96,95,94,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,64,63,62,61,60,59,58,56,55,54,53,52,51,49,48,47,46,44,43,42,41,40,38,37,36,34,33,32,31,29,28,27,26,24,23,22,20,19,18,17,15,14,13,11,10,9,7,6,5,3,2,1,0,-2,-3,-4,-6,-7,-8,-10,-11,-12,-14,-15,-16,-18,-19,-20,-21,-23,-24,-25,-27,-28,-29,-30,-32,-33,-34,-35,-37,-38,-39,-40,-42,-43,-44,-45,-47,-48,-49,-50,-51,-53,-54,-55,-56,-57,-59,-60,-61,-62,-63,-64,-65,-67,-68,-69,-70,-71,-72,-73,-74,-75,-76,-77,-78,-79,-80,-81,-82,-83,-84,-85,-86,-87,-88,-89,-90,-91,-92,-93,-94,-95,-95,-96,-97,-98,-99,-100,-100,-101,-102,-103,-104,-104,-105,-106,-106,-107,-108,-109,-109,-110,-111,-111,-112,-112,-113,-114,-114,-115,-115,-116,-116,-117,-117,-118,-118,-119,-119,-120,-120,-121,-121,-122,-122,-122,-123,-123,-123,-124,-124,-124,-125,-125,-125,-125,-126,-126,-126,-126,-126,-127,-127,-127,-127,-127,-127,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-127,-127,-127,-127,-127,-127,-127,-126,-126,-126,-126,-125,-125,-125,-125,-124,-124,-124,-124,-123,-123,-123,-122,-122,-122,-121,-121,-120,-120,-120,-119,-119,-118,-118,-117,-117,-116,-116,-115,-115,-114,-114,-113,-113,-112,-112,-111,-111,-110,-109,-109,-108,-108,-107,-106,-106,-105,-104,-104,-103,-102,-102,-101,-100,-100,-99,-98,-98,-97,-96,-95,-95,-94,-93,-93,-92,-91,-90,-90,-89,-88,-87,-86,-86,-85,-84,-83,-83,-82,-81,-80,-79,-79,-78,-77,-76,-75,-74,-74,-73,-72,-71,-70,-70,-69,-68,-67,-66,-65,-65,-64,-63,-62,-61,-60,-60,-59,-58,-57,-56,-55,-55,-54,-53,-52,-51,-50,-50,-49,-48,-47,-46,-46,-45,-44,-43,-42,-42,-41,-40,-39,-38,-38,-37,-36,-35,-35,-34,-33,-32,-32,-31,-30,-29,-29,-28,-27,-26,-26,-25,-24,-24,-23,-22,-22,-21,-20,-19,-19,-18,-18,-17,-16,-16,-15,-14,-14,-13,-13,-12,-11,-11,-10,-10,-9,-9,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,-1,0,0,1,1,2,2,2,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,8,8,8,8,9,9,9,9,9,10,10,10,10,10,10,11,11,11,11,11,11,11,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,10,10,10,10,10,10,9,9,9,9,9,9,8,8,8,8,8,7,7,7,7,6,6,6,6,6,5,5,5,5,4,4,4,4,3,3,3,3,2,2,2,2,1,1,1,1,0,0,0};
int tabp[8]={0,0,0,0,0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT (&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 80;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 639;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){
	uint8_t address = 0x18;
	HAL_I2C_Master_Transmit(&hi2c1, 0x50, &address, 1, 50);
	uint8_t data[2];
	HAL_I2C_Master_Receive(&hi2c1, 0x50, data, 2, 50);
	ac_cap = (((uint16_t)(data[0]&0x0F))<<8) + (uint16_t)(data[1]);

	if(ech>=1000){
		ech=0;
	}
	else ech+=k;

	//ac_cap=2048+tab_ech[ech];
	  switch(etat){
	  case 1:
		  if(ac_cap>2148){
			  etat=2;
		  }
		  break;
	  case 2:
		  if(ac_cap<1948){
			  etat=3;
			  cpt_ech=0;
		  }
		  break;
	  case 3:
		  cpt_ech++;
		  if(ac_cap>2148){
			  etat=4;
		  }
		  break;
	  case 4:
		  {
		cpt_ech++;
		  if(ac_cap<1948){
			  if(ac_cap)
			  etat=3;
			  if(cpt_ech!=0){
				  if(i==8){i=0;}
				  tabp[i]=30000/cpt_ech;
				  i++;
				  cpt_ech=0;
			  }
		  }
		  }
		  break;
	  default:
		  break;
	  }

	  test2=0;
	  for(j=0;j<8;j++){
		  test2+=tabp[j];
	  }
	  bpm=test2/8;
	  if(bpm<=alarm_high){
	  		  if((bpm>=alarm_low-1) && (bpm<60)){
	  			  TIM2->ARR = 999;
	  			  TIM2->PSC = 63999;
	  			  TIM2->CCR1=500;
	  		  }
	  		  else if((bpm>=60) && (bpm<90)){
	  			  TIM2->ARR = 999;
	  			  TIM2->PSC = 31999;
	  			  TIM2->CCR1=500;
	  		  }
	  		  else if((bpm>90) && (bpm<=130)){
	  			  TIM2->ARR = 999;
	  			  TIM2->PSC = 21332;
	  			  TIM2->CCR1=500;
	  		  }
	  		  else if((bpm>130) && (bpm<=170)){
	  			  TIM2->ARR = 999;
	  			  TIM2->PSC = 15999;
	  			  TIM2->CCR1=500;
	  		  }
	  		  else if((bpm>170) && (bpm<=190)){
	  			  TIM2->ARR = 999;
	  			  TIM2->PSC = 12799;
	  			  TIM2->CCR1=500;
	  		  }
	  		  else if ((bpm>190)){
	  			  TIM2->ARR = 999;
	  			  TIM2->PSC = 10666;
	  			  TIM2->CCR1=500;
	  		  }

	  	  }
	  	  else {
	  		 TIM2->ARR = 999;
	  		 TIM2->PSC = 10666;
	  		 TIM2->CCR1= 1000;
	  	  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  /* Prevent unused argument(s) compilation warning */

	test=0;
  if((*((uint32_t*)GPIOC_IDR) & (1<<13))>>13){

	  time2 = uwTick;
	  test = time2 - time1;
	  if(test<=1000){
		  if(k==8){k=1;}
		 else k++;
	  }
	  if(test > 1000 && test <= 2000){
		  alarm_high=130;
	  }
	  if(test > 2000 && test <= 3000){
		  alarm_high=160;

	  }
	  if(test > 3000 && test <= 4000){
		  alarm_high=190;

	  }
	  time1=0;
	  time2=0;
  }else{
	  time1 = uwTick;
  }


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
