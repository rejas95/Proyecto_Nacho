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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery_glass_lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
LCD_HandleTypeDef hlcd;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_uart5_rx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_LCD_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t Menu, recibir, enviar, bandera, nopulsacion;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
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
	MX_DMA_Init();
	MX_UART5_Init();
	MX_LCD_Init();
	/* USER CODE BEGIN 2 */
	BSP_LCD_GLASS_Init();
	BSP_LCD_GLASS_Clear();
	enviar = 0;
	Menu = 0;
	bandera = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (Menu == 0) {

			if (bandera == 1) {
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString("PULSE");
				HAL_Delay(3000);
				bandera = 0;

			} else {

				BSP_LCD_GLASS_ScrollSentence(" Seleccionar modo UP ", 1,
				SCROLL_SPEED_MEDIUM);
				bandera = 1;
			}

		}
		if (Menu == 1) {
			BSP_LCD_GLASS_ScrollSentence(" Recibir ", 1,
			SCROLL_SPEED_MEDIUM);
		}
		if (Menu == 2) {
			if (enviar == 0) {
				BSP_LCD_GLASS_ScrollSentence(
						" Enviar.Seleccionar onda .RIGHT O LEFT ", 1,
						SCROLL_SPEED_MEDIUM);
			}
			if (enviar == 1) {
				BSP_LCD_GLASS_ScrollSentence(" Onda cuadrada ", 1,
				SCROLL_SPEED_MEDIUM);

			}
			if (enviar == 2) {
				BSP_LCD_GLASS_ScrollSentence(" Onda triangular ", 1,
				SCROLL_SPEED_MEDIUM);
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
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure LSE Drive Capability
	 */
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection =
	RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_UART5;
	PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/**Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* LCD init function */
static void MX_LCD_Init(void) {

	hlcd.Instance = LCD;
	hlcd.Init.Prescaler = LCD_PRESCALER_16;
	hlcd.Init.Divider = LCD_DIVIDER_16;
	hlcd.Init.Duty = LCD_DUTY_1_4;
	hlcd.Init.Bias = LCD_BIAS_1_4;
	hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
	hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
	hlcd.Init.DeadTime = LCD_DEADTIME_0;
	hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
	hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
	hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
	hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
	hlcd.Init.HighDrive = LCD_HIGHDRIVE_DISABLE;
	if (HAL_LCD_Init(&hlcd) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART5 init function */
static void MX_UART5_Init(void) {

	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_7B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 * Configure DMA for memory to memory transfers
 *   hdma_memtomem_dma1_channel1
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;
	__HAL_RCC_DMA2_CLK_ENABLE()
	;

	/* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
	hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
	hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_0;
	hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
	hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
	hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
	hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/* DMA interrupt init */
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
	/* DMA2_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/** Configure pins
 PE2   ------> SAI1_MCLK_A
 PE4   ------> SAI1_FS_A
 PE5   ------> SAI1_SCK_A
 PE6   ------> SAI1_SD_A
 PE7   ------> SAI1_SD_B
 PE9   ------> SAI1_FS_B
 PE10   ------> QUADSPI_CLK
 PE11   ------> QUADSPI_NCS
 PE12   ------> QUADSPI_BK1_IO0
 PE13   ------> QUADSPI_BK1_IO1
 PE14   ------> QUADSPI_BK1_IO2
 PE15   ------> QUADSPI_BK1_IO3
 PB10   ------> I2C2_SCL
 PB11   ------> I2C2_SDA
 PA11   ------> USB_OTG_FS_DM
 PA12   ------> USB_OTG_FS_DP
 PD1   ------> SPI2_SCK
 PD3   ------> SPI2_MISO
 PD4   ------> SPI2_MOSI
 PD5   ------> USART2_TX
 PD6   ------> USART2_RX
 PB6   ------> I2C1_SCL
 PB7   ------> I2C1_SDA
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, AUDIO_RST_Pin | LD_G_Pin | XL_CS_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD_R_Pin | M3V3_REG_ON_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_VBUS_GPIO_Port, OTG_FS_VBUS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : SAI1_MCK_Pin SAI1_FS_Pin SAI1_SCK_Pin SAI1_SD_Pin
	 AUDIO_DIN_Pin */
	GPIO_InitStruct.Pin = SAI1_MCK_Pin | SAI1_FS_Pin | SAI1_SCK_Pin
			| SAI1_SD_Pin | AUDIO_DIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : AUDIO_RST_Pin */
	GPIO_InitStruct.Pin = AUDIO_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(AUDIO_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MFX_IRQ_OUT_Pin OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = MFX_IRQ_OUT_Pin | OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 MAG_INT_Pin MAG_DRDY_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | MAG_INT_Pin | MAG_DRDY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : CENTRO_Pin IZQUIERDA_Pin DERECHA_Pin ARRIBA_Pin
	 ABAJO_Pin */
	GPIO_InitStruct.Pin = CENTRO_Pin | IZQUIERDA_Pin | DERECHA_Pin | ARRIBA_Pin
			| ABAJO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : MFX_WAKEUP_Pin */
	GPIO_InitStruct.Pin = MFX_WAKEUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MFX_WAKEUP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD_R_Pin */
	GPIO_InitStruct.Pin = LD_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LD_R_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD_G_Pin */
	GPIO_InitStruct.Pin = LD_G_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LD_G_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : AUDIO_CLK_Pin */
	GPIO_InitStruct.Pin = AUDIO_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
	HAL_GPIO_Init(AUDIO_CLK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : QSPI_CLK_Pin QSPI_CS_Pin QSPI_D0_Pin QSPI_D1_Pin
	 QSPI_D2_Pin QSPI_D3_Pin */
	GPIO_InitStruct.Pin = QSPI_CLK_Pin | QSPI_CS_Pin | QSPI_D0_Pin | QSPI_D1_Pin
			| QSPI_D2_Pin | QSPI_D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : MFX_I2C_SLC_Pin MFX_I2C_SDA_Pin */
	GPIO_InitStruct.Pin = MFX_I2C_SLC_Pin | MFX_I2C_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin OTG_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin | OTG_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_DM_Pin OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_FS_DM_Pin | OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_SCK_Pin MEMS_MISO_Pin MEMS_MOSI_Pin */
	GPIO_InitStruct.Pin = MEMS_SCK_Pin | MEMS_MISO_Pin | MEMS_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
	GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : GYRO_CS_Pin */
	GPIO_InitStruct.Pin = GYRO_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GYRO_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : M3V3_REG_ON_Pin */
	GPIO_InitStruct.Pin = M3V3_REG_ON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(M3V3_REG_ON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
	GPIO_InitStruct.Pin = I2C1_SCL_Pin | I2C1_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : GYRO_INT2_Pin */
	GPIO_InitStruct.Pin = GYRO_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GYRO_INT2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : XL_CS_Pin */
	GPIO_InitStruct.Pin = XL_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(XL_CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if ((GPIO_Pin == ARRIBA_Pin) && (Menu != 2)) {
		Menu = Menu + 1;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); // Cambiar el estado del LED rojo
	} else {
		Menu = 0;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	}

	if ((GPIO_Pin == DERECHA_Pin) && (Menu = 2) && (enviar <= 1)) {
		enviar = enviar + 1;

	}
	if ((GPIO_Pin == IZQUIERDA_Pin) && (Menu = 2) && (enviar >= 2)) {
		enviar = enviar - 1;
	}

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
