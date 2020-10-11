
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "dwt_stm32_delay.h"
#include "CTR_nrf24l01.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
void NRF24L01_Config()
{
	CTR_nrfPinConfig();
}
//----------------------khai bao bien cho ham truyen nhan NRF24L01 A-----------------------//
uint8_t chuoi[32];
char ok[32]="ok\r\n";
char check_ok[32]="ok";
uint8_t check_lai_lan_cuoi[32]="check lai lan cuoi\r\n";
uint8_t test[32]="start\r\n";
char T[32]=" C   ";
char H[32]=" %\r\n";
char xuong_hang[32]="\r\n";
char chuoi_truyen[32];

int a=0;
long reset=0;
int thoi_gian_check=0;
//----------------------khai bao bien cho ham truyen nhan NRF24L01 A-----------------------//
//bien ngat//
int count=0;
//bien ngat//

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char bat_san[32]="        Bat san\r\n";
char tat_san[32]="        Tat san\r\n";
char bat[32]="        Bat\r\n";
char bat_khong_duoc[32]="        Bat khong duoc\r\n";
char tat[32]="        Tat\r\n";
char tat_khong_duoc[32]="        Tat khong duoc\r\n";
void read()
{
	if(count != 0)
	{
		if(count%2 == 0 || count%2 != 0)
		{
			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\nChe do dieu khien tay\r\n",26,5);
			snprintf(chuoi_truyen, sizeof(chuoi_truyen), "DK TAY");
		}
		a=1;//de khong gui ok
		reset=801;//de khi vao che do dieu khien tay thi khong tu reset dong co nua
	}
	else
	{
		if(strcmp(chuoi, "ON_1b") == 0 )
		//if(strcmp(chuoi, "ON_2b") == 0 )
		{
			if(HAL_GPIO_ReadPin(read_motor_GPIO_Port, read_motor_Pin)==1)
			{
				HAL_UART_Transmit(&huart1,bat_san,32,5);
				snprintf(chuoi_truyen, sizeof(chuoi_truyen), "ON_1b");
				//snprintf(chuoi_truyen, sizeof(chuoi_truyen), "ON_2b");
				//khong can bat nua
			}
			else if(HAL_GPIO_ReadPin(read_motor_GPIO_Port, read_motor_Pin)==0)
			{
				HAL_UART_Transmit(&huart1,bat,32,5);
				HAL_GPIO_WritePin(LED_motor_GPIO_Port, LED_motor_Pin, GPIO_PIN_SET);
				if(HAL_GPIO_ReadPin(read_motor_GPIO_Port, read_motor_Pin)==0)
				{
					HAL_UART_Transmit(&huart1,bat_khong_duoc,32,5);
					snprintf(chuoi_truyen, sizeof(chuoi_truyen), "Khong bat duoc bom_1b");
					//snprintf(chuoi_truyen, sizeof(chuoi_truyen), "Khong bat duoc bom_2b");
				}
				else
				{
					snprintf(chuoi_truyen, sizeof(chuoi_truyen), "ON_1b");
					//snprintf(chuoi_truyen, sizeof(chuoi_truyen), "ON_2b");
				}
			}
			snprintf(chuoi_truyen, sizeof(chuoi_truyen), "ON_1b");
			//snprintf(chuoi_truyen, sizeof(chuoi_truyen), "ON_2b");
		}
		else if(strcmp(chuoi, "OFF_1b") == 0)
		//else if(strcmp(chuoi, "OFF_2b") == 0)
		{
			if(HAL_GPIO_ReadPin(read_motor_GPIO_Port, read_motor_Pin)==0)
			{
				//khong can tat nua
				HAL_UART_Transmit(&huart1,tat_san,32,5);
				snprintf(chuoi_truyen, sizeof(chuoi_truyen), "OFF_1b");
				//snprintf(chuoi_truyen, sizeof(chuoi_truyen), "OFF_2b");
			}
			else if(HAL_GPIO_ReadPin(read_motor_GPIO_Port, read_motor_Pin)==1)
			{
				HAL_UART_Transmit(&huart1,tat,32,5);
				HAL_GPIO_WritePin(LED_motor_GPIO_Port, LED_motor_Pin, GPIO_PIN_RESET);
				if(HAL_GPIO_ReadPin(read_motor_GPIO_Port, read_motor_Pin)==1)
				{
					HAL_UART_Transmit(&huart1,tat_khong_duoc,32,5);
					snprintf(chuoi_truyen, sizeof(chuoi_truyen), "Khong tat duoc bom_1b");
					//snprintf(chuoi_truyen, sizeof(chuoi_truyen), "Khong tat duoc bom_2b");
				}
				else
				{
					snprintf(chuoi_truyen, sizeof(chuoi_truyen), "OFF_1b");
					//snprintf(chuoi_truyen, sizeof(chuoi_truyen), "OFF_2b");
				}
			}
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	DWT_Delay_Init();
	NRF24L01_Config();
	CTR_nrfInit(); // khoi tao
	CTR_spiRWreg(CTR_WRITE_RE + CTR_STATUS, 0xFF);//XOA BIT tren thanh Status
	CTR_nrfSetRX();
	// them doan ma can gui vao buffer gui
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{ 
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		reset++;
		a=0;
		thoi_gian_check=0;
		if(CTR_nrfGetPacket(chuoi)==1)
		{
			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\nchuoi nhan\r\n",14,40);
			HAL_UART_Transmit(&huart1,chuoi,32,5);
			HAL_GPIO_WritePin(led_receive_GPIO_Port, led_receive_Pin, GPIO_PIN_SET);
			if(strcmp(chuoi, "ON_1b") == 0 || strcmp(chuoi, "OFF_1b") == 0)
			//if(strcmp(chuoi, "ON_2b") == 0 || strcmp(chuoi, "OFF_2b") == 0)
			{
				//HAL_UART_Transmit(&huart1,chuoi,32,5);//xem chuoi nhan duoc tren pc				
				read();
				memset(chuoi,0x00,32);//xoa chuoi
				HAL_UART_Transmit(&huart1,(uint8_t*)"dang truyen\r\n",15,5);
				HAL_UART_Transmit(&huart1,chuoi_truyen,32,5);
				CTR_nrfSetTX();
				CTR_nrfSendPacket(chuoi_truyen);
				if(a==0)
				{
					HAL_UART_Transmit(&huart1,(uint8_t*)"\r\nkiem tra co nhan duoc ok khong\r\n",36,5); 
					CTR_nrfSetRX();
					HAL_Delay(1000);
				}
				else
				{
					HAL_UART_Transmit(&huart1,(uint8_t*)"\r\nKET THUC\r\n",14,5);
				}
				while(a==0)
				{
					thoi_gian_check+=1;
					if(CTR_nrfGetPacket(chuoi)==1)
					{
						HAL_UART_Transmit(&huart1,chuoi,32,5);
						int chuoi_ok = strcmp(chuoi, ok);
						if(chuoi_ok == 0)
						{
							a=1;//thoat vong cho ok
							reset=0;
							HAL_UART_Transmit(&huart1,(uint8_t*)"truyen ok\r\n",12,5); 
							CTR_nrfSetTX();
							CTR_nrfSendPacket(check_ok);
							HAL_UART_Transmit(&huart1,(uint8_t*)"KET THUC\r\n",10,5);
						}
						else
						{
							a=1;
							//HAL_UART_Transmit(&huart1,chuoi,32,5);
							HAL_UART_Transmit(&huart1,(uint8_t*)"chuoi loi\r\n",12,5);
							CTR_spiRWreg(CTR_WRITE_RE + CTR_STATUS, 0xFF);//XOA BIT tren thanh Status
							//CTR_nrfSetRX();
							//thoi_gian_check=6;
						}
						memset(chuoi,0x00,32);//xoa chuoi
					}
					//neu khong nhan duoc chuoi ok thi tat motor
					if(thoi_gian_check>50)
					{
						a=1;
						HAL_UART_Transmit(&huart1,(uint8_t*)"het gio cho => khong thanh cong\r\n",35,5);
						if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==0)
						{
							//neu tat roi thi thoi
						}
						else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==1)
						{
							HAL_GPIO_WritePin(LED_motor_GPIO_Port, LED_motor_Pin, GPIO_PIN_RESET);
							//tat neu motor chua tat sau khi ket thuc qua trinh ntruyen nhan loi
						}
					}
				}
			}
			memset(chuoi,0x00,32);//xoa chuoi
			CTR_spiRWreg(CTR_WRITE_RE + CTR_STATUS, 0xFF);//XOA BIT tren thanh Status
			CTR_nrfSetRX();
			HAL_GPIO_WritePin(led_receive_GPIO_Port, led_receive_Pin, GPIO_PIN_RESET);
		}
		if(reset==800)
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==0)
			{
				//neu tat roi thi thoi
				//HAL_GPIO_WritePin(LED_motor_GPIO_Port, LED_motor_Pin, GPIO_PIN_SET);
			}
			else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==1)
			{
				HAL_GPIO_WritePin(LED_motor_GPIO_Port, LED_motor_Pin, GPIO_PIN_RESET);
				//tat neu motor chua tat sau khi ket thuc qua trinh ntruyen nhan loi
			}
			reset=0;
			HAL_UART_Transmit(&huart1,(uint8_t*)"RESET LAI THIET BI\r\n",22,5);
		}
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_motor_Pin|led_receive_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Control_motor_Pin */
  GPIO_InitStruct.Pin = Control_motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Control_motor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : read_motor_Pin */
  GPIO_InitStruct.Pin = read_motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(read_motor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_motor_Pin */
  GPIO_InitStruct.Pin = LED_motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_motor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_receive_Pin */
  GPIO_InitStruct.Pin = led_receive_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(led_receive_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOB,Control_motor_Pin)==0)
	{
		
		count++;
		if(count%2!=0)
		{
			HAL_GPIO_WritePin(LED_motor_GPIO_Port, LED_motor_Pin, GPIO_PIN_SET);
		}
		else if(count%2==0)
		{
			HAL_GPIO_WritePin(LED_motor_GPIO_Port, LED_motor_Pin, GPIO_PIN_RESET);
		}
		DWT_Delay_us(200000);
		while(HAL_GPIO_ReadPin(GPIOB,Control_motor_Pin)==0);
		
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
