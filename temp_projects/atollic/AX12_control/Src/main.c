/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "MW-AHRSv1.h"
#include "math.h"

#define AX_WRITE_DATA  3

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CanTxMsgTypeDef CantxMessage;
CanRxMsgTypeDef CanrxMessage;

MW_AHRS ahrs_obj = { 0, };

uint32_t mainCounter = 0;
uint32_t tim6Counter = 0;
uint32_t tim7Counter = 0;

uint8_t sendData[100] = { 0, };

typedef struct _ax12 {
	uint8_t id;
	uint8_t reg;
	uint16_t data;
} AX12;

float yaw_data;
float roll_data;
float pitch_data;

AX12 motor1;
AX12 motor2;
AX12 motor3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void can_tx(void);
void can_init_user(void);

void ax12_SetRegister(uint8_t id, uint8_t regstart, uint16_t data);

void mw_ahrsv1_trans(void);
void mw_ahrsv1_rcv(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

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
	MX_CAN1_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_USART1_UART_Init();

	/* USER CODE BEGIN 2 */

	can_init_user();
	mw_ahrsv1_trans();

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		mainCounter++;
		HAL_Delay(1);

		yaw_data = ahrs_obj.e_yaw + 150;
		if(yaw_data <= 0){
			yaw_data = 0;
		}
		else if(yaw_data >= 300){
			yaw_data = 300;
		}

		roll_data = ahrs_obj.e_roll + 150;
		if(roll_data <= 0){
			roll_data = 0;
		}
		else if(roll_data >= 300){
			roll_data = 300;
		}

		pitch_data = ahrs_obj.e_pitch + 150;
		if(pitch_data <= 0){
			pitch_data = 0;
		}
		else if(pitch_data >= 300){
			pitch_data = 300;
		}

		ax12_SetRegister(1, 30, 1023-(yaw_data*3.4));
		HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);
		ax12_SetRegister(1, 32, 150);
		HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);

		ax12_SetRegister(2, 30, 1023-(roll_data*3.4));
		HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);
		ax12_SetRegister(2, 32, 150);
		HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);

		ax12_SetRegister(3, 30, 1023-(pitch_data*3.4));
		HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);
		ax12_SetRegister(3, 32, 150);
		HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);

		/*ax12_SetRegister(2, 30, 512);
		HAL_UART_Transmit(&huart1, sendData, 9, 10);

		ax12_SetRegister(2, 32, 150);
		HAL_UART_Transmit(&huart1, sendData, 9, 10);

		HAL_Delay(1000);

		ax12_SetRegister(2, 30, 712);
		HAL_UART_Transmit(&huart1, sendData, 9, 10);

		ax12_SetRegister(2, 32, 80);
		HAL_UART_Transmit(&huart1, sendData, 9, 10);

		HAL_Delay(1000);*/

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void) {

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 9;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_3TQ;
	hcan1.Init.BS2 = CAN_BS2_1TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM6 init function */
static void MX_TIM6_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 89;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 999;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM7 init function */
static void MX_TIM7_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 900;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 4000;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 1000000;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

}

/* USER CODE BEGIN 4 */

void ax12_SetRegister(uint8_t id, uint8_t regstart, uint16_t data) {
	sendData[0] = 0xFF;
	sendData[1] = 0xFF;
	sendData[2] = id;
	sendData[3] = 5;
	sendData[4] = AX_WRITE_DATA;
	sendData[5] = regstart;
	sendData[6] = (data & 0xFF);
	sendData[7] = (data & 0xFF00) >> 8;
	sendData[8] = (0xFF
			- ((id + 5 + AX_WRITE_DATA + regstart + (data & 0xFF)
					+ ((data & 0xFF00) >> 8)) % 256));

}

void ax12_set_data(AX12 *data) {

}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
	for (int i = 0; i < 8; i++) {
		ahrs_obj.can_read_data[i] = CanrxMessage.Data[i];
	}

	mw_ahrs_input_data(&ahrs_obj);

	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}

}
void mw_ahrsv1_trans(void) {
	int i = 0;
	mw_ahrs_set_period(&ahrs_obj, 1);
	HAL_CAN_Transmit(&hcan1, 10);
	mw_ahrs_set_data_type(&ahrs_obj, 0, 0, 1, 1);
	HAL_CAN_Transmit(&hcan1, 10);

	for (i = 0; i < 8; i++) {
		CantxMessage.Data[i] = ahrs_obj.can_write_data[i];
	}

	if (HAL_CAN_Transmit(&hcan1, 10) != HAL_OK) {
		Error_Handler();
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}
void can_init_user(void) {
	CAN_FilterConfTypeDef canFilter;

	hcan1.pTxMsg = &CantxMessage;
	hcan1.pRxMsg = &CanrxMessage;

	canFilter.FilterNumber = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilter.FilterIdHigh = 0x0000 << 5;
	canFilter.FilterIdLow = 0x0000;
	canFilter.FilterMaskIdHigh = 0x0000 << 5;
	canFilter.FilterMaskIdLow = 0x0000;
	canFilter.FilterFIFOAssignment = 0;
	canFilter.FilterActivation = ENABLE;
	canFilter.BankNumber = 0;

	if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
		Error_Handler();
	}

	CantxMessage.StdId = 0x01;
	CantxMessage.ExtId = 0x01;
	CantxMessage.RTR = CAN_RTR_DATA;
	CantxMessage.IDE = CAN_ID_STD;
	CantxMessage.DLC = 8;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		tim6Counter++;
		//	ax12_SetRegister( 1, 30, 512);
		//	ax12_SetRegister(1,32, 80);
	} else if (htim->Instance == TIM7) {
		tim7Counter++;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {

		//sendCompleteflag = 1
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		//
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

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
