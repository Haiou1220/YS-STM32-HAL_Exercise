/* USER CODE BEGIN Header */
/* **************用户-文件说明-开始*******************************************************************/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
	/*串口重定向 + 串口命令控制led-dma  */
	/* **************用户-文件说明-结束*******************************************************************/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* **************用户-头文件-开始***************************************/
#include "string.h"
#include "stdio.h"//在 stm32f1xx_hal_msp.c 重定向printf
/* **************用户-头文件-结束***************************************/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel2;
/* USER CODE BEGIN PV */
/* 用户--私有变量--开始 -----------------------------------------------------------------------------*/

		uint8_t Rx_Buffer[1];/*接收缓存*/
		uint8_t Tx_Buffer[100];/*发送缓存*/
		uint8_t cmd;
		uint8_t cmd_rec_flag;
		uint8_t  SrcAddress[50];
		uint8_t DstAddress[50];//DMA-memToMen

/* 用户--私有变量--结束-----------------------------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 用户--代码0--开始-----------------------------------------------------------------------------*/

		
	static void Show_Message(void)
{
			printf("\r\n这是一个通过串口控制LED\n");
	
			printf("指令:\n");
			printf("   指令   ------      状态 \n");
			printf("     1    ------    LED-off\n");
			printf("     2    ------    LED-on\n");

}

/* 用户--代码0--结束----------------------------------------------------------------------------*/
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	/* **************用户-代码2-开始***************************************************************/
	
	memset(SrcAddress,'S',50);
	memset(DstAddress,'D',50);//初始化 填充
			
		printf("printf:使用printf重定向 而不是uatr函数发送了\n");
		Show_Message();
	
	//HAL_UART_Transmit();
	HAL_UART_Receive_IT(&huart1,&cmd,1);//开启命令接收中断
		
	
	HAL_DMA_Start(&hdma_memtomem_dma1_channel2, (uint32_t)&SrcAddress, (uint32_t)&DstAddress, 50);//hdma_memtomem_dma1_channel2
		
	/* **************用户-代码2-开始***********************************************************/	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	if(cmd_rec_flag==1)
	{
		cmd_rec_flag = 0;
		switch(cmd)
		{
			case '1':
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
				break;
			
			case '2':
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
				
				break;
			
			default:
				
				break;
		}	
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel2
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel2 on DMA1_Channel2 */
  hdma_memtomem_dma1_channel2.Instance = DMA1_Channel2;
  hdma_memtomem_dma1_channel2.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel2.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel2.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel2.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel2.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel2.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel2) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Button_Pin */
  GPIO_InitStruct.Pin = LED_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LED_Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint8_t time=0;//按键计数器
	uint32_t count = 0;
	/*LED_Button 中断函数*/
	if(GPIO_Pin==LED_Button_Pin)
	{
		HAL_Delay(100);
		
		if(HAL_GPIO_ReadPin(LED_Button_GPIO_Port,LED_Button_Pin)== GPIO_PIN_RESET)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			
			HAL_Delay(50);
		
			printf("\nButton-on:%d\n",++time);
			
			//printf("%d\n",__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
			//printf("%x\n",HAL_UART_GetState(&huart1));
			printf("start:%d\n",count);
			while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)==RESET)
			{
				count++;
			}
			printf("end:%d\n",count);
	
			printf("DstAddress:old is D Now is what?\n");
			
			
			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX)
			{
					HAL_UART_Transmit_DMA(&huart1,  DstAddress, 50);//发送 数据块
			}
				printf("@%x\n",HAL_UART_GetState(&huart1));
		}

		
	
	}
}
/*接收中断 回调函数*/
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	printf("enter:::HAL_UART_RxCpltCallback收到字符:%c\n",cmd);
	 cmd_rec_flag = 1;//使能命令开关
		HAL_UART_Receive_IT(&huart1,&cmd,1);//再次开启接收中断
		printf("@%x\n",HAL_UART_GetState(&huart1));
	
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
