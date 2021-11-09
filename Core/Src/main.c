/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <string.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t adcRaw[3];

/* USER CODE END 0 */

uint8_t TM1637_digit2segments[] = {
/*76543210 */
0b00111111,
0b00000110,
0b01011011,
0b01001111,
0b01100110,
0b01101101,
0b01111101,
0b00000111,
0b01111111,
0b01101111
};

void TM1637_start()
{
    HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TM1637_IO_GPIO_Port, TM1637_IO_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void TM1637_stop()
{
    HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TM1637_IO_GPIO_Port, TM1637_IO_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(TM1637_IO_GPIO_Port, TM1637_IO_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

void TM1637_send_byte(uint8_t c)
{
    int i;
    for(i=0;i<8;i++) {
        HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_RESET);

        if(c & 0x1) {
            HAL_GPIO_WritePin(TM1637_IO_GPIO_Port, TM1637_IO_Pin, GPIO_PIN_SET);
        }else {
            HAL_GPIO_WritePin(TM1637_IO_GPIO_Port, TM1637_IO_Pin, GPIO_PIN_RESET);
        }

        HAL_Delay(1);
        HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_SET);
        HAL_Delay(1);

        c = c>>1;
    }
    
    HAL_GPIO_WritePin(TM1637_IO_GPIO_Port, TM1637_IO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

void TM1637_display(uint8_t d, int pos)
{
    TM1637_start();
    TM1637_send_byte(0xC0 + (pos&0x3));
    TM1637_send_byte(TM1637_digit2segments[d]);
    TM1637_stop();
}

void TM1637_clear(int pos)
{
    TM1637_start();
    TM1637_send_byte(0xC0 + (pos&0x3));
    TM1637_send_byte(0);
    TM1637_stop();
}

void TM1637_init()
{
    int i;

    TM1637_start();
    TM1637_send_byte(0x40);
    HAL_GPIO_WritePin(TM1637_IO_GPIO_Port, TM1637_IO_Pin, GPIO_PIN_SET);
    TM1637_stop();

    TM1637_start();
    TM1637_send_byte(0x8C);
    TM1637_stop();

    for(i=0;i<4;i++) {
        TM1637_clear(i);
    }
}

void __DS1623_send_data(uint8_t data)
{
    uint8_t i;
    for(i=0;i<8;i++) {
        HAL_GPIO_WritePin(DS1623_CLK_GPIO_Port, DS1623_CLK_Pin, GPIO_PIN_RESET); 
        if(data & 0x01 ) {
            HAL_GPIO_WritePin(DS1623_DQ_GPIO_Port, DS1623_DQ_Pin, GPIO_PIN_SET); 
        } else {
            HAL_GPIO_WritePin(DS1623_DQ_GPIO_Port, DS1623_DQ_Pin, GPIO_PIN_RESET); 
        }
        HAL_Delay(1);
        HAL_GPIO_WritePin(DS1623_CLK_GPIO_Port, DS1623_CLK_Pin, GPIO_PIN_SET); 
        HAL_Delay(1);

        data = data >> 1;
    }

    HAL_GPIO_WritePin(DS1623_DQ_GPIO_Port, DS1623_DQ_Pin, GPIO_PIN_SET); 
}

uint16_t __DS1623_recv_data(uint8_t numbits)
{
    uint8_t i;
    uint8_t value = 0;

    for(i=0;i<numbits;i++) {
        HAL_GPIO_WritePin(DS1623_CLK_GPIO_Port, DS1623_CLK_Pin, GPIO_PIN_RESET); 
        HAL_Delay(1);

        if(GPIO_PIN_SET == HAL_GPIO_ReadPin(DS1623_DQ_GPIO_Port, DS1623_DQ_Pin)) {
            value |= 1<<i;
        }

        HAL_GPIO_WritePin(DS1623_CLK_GPIO_Port, DS1623_CLK_Pin, GPIO_PIN_SET); 
        HAL_Delay(1);
    }

    return value;
}

uint8_t DS1623_read_config()
{
    uint8_t ret;
    HAL_GPIO_WritePin(DS1623_RST_N_GPIO_Port, DS1623_RST_N_Pin, GPIO_PIN_SET); 
    HAL_Delay(1);
    __DS1623_send_data(0xAC);

    ret = __DS1623_recv_data(8);
    HAL_GPIO_WritePin(DS1623_RST_N_GPIO_Port, DS1623_RST_N_Pin, GPIO_PIN_RESET); 
    return ret;
}

void DS1623_start_conversion()
{
    HAL_GPIO_WritePin(DS1623_RST_N_GPIO_Port, DS1623_RST_N_Pin, GPIO_PIN_SET); 
    HAL_Delay(1);
    __DS1623_send_data(0xEE);
    HAL_GPIO_WritePin(DS1623_RST_N_GPIO_Port, DS1623_RST_N_Pin, GPIO_PIN_RESET); 
}

uint16_t DS1623_read_temp()
{
    uint8_t ret;
    HAL_GPIO_WritePin(DS1623_RST_N_GPIO_Port, DS1623_RST_N_Pin, GPIO_PIN_SET); 
    HAL_Delay(1);
    __DS1623_send_data(0xAA);

    ret = __DS1623_recv_data(9);
    HAL_GPIO_WritePin(DS1623_RST_N_GPIO_Port, DS1623_RST_N_Pin, GPIO_PIN_RESET); 
    return ret;
}

int L12S_config()
{
    uint8_t config[] = {
        0xaa, 0x5a,
        0x22, 0x33, /* Module ID */
        0x11, 0x22, /* Network ID */
        0x00, 0x00, /* RF power, 0dbm */
        0x00, 0x04, /* 9600 bps */
        0x00, 0x64, /* Channel */
        0x00, 0x00,
        0x00, 0x12,
        0x00, 0x00
    };

    int i;
    /* Calculate checksum. */
    config[17] = 0;
    for(i=0;i<17;i++) {
        config[17] += config[i];
    }

    for(i=0;i<3;i++) {

        /* Delay some time to let L12S stable. */
        HAL_Delay(3000);

        HAL_GPIO_WritePin(LC12S_SET_GPIO_Port, LC12S_SET_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);

        HAL_UART_Transmit(&huart1, config, 18, 1000);
        HAL_UART_Receive(&huart1, config, 18, 1000);

        HAL_GPIO_WritePin(LC12S_SET_GPIO_Port, LC12S_SET_Pin, GPIO_PIN_SET);

        if(config[1] == 0x5b) {
        	return 0; // We got respone.
        }
    }

    return -1;
}

void display_temp(uint8_t t)
{
    if(t&1) {
        TM1637_display(5, 2);
    } else {
        TM1637_display(0, 2);
    }

    t=t>>1;
    TM1637_display(t%10, 1);
    TM1637_display(t/10, 0);
}

void run_thermalstat()
{
  uint16_t t;

  uint8_t set_temp = 22;

  for(;;) {

    DS1623_start_conversion();
    HAL_Delay(1000);
    t = DS1623_read_temp();

    display_temp(t);

    if(t>10*2 && t<50*2) { /* reasonable range */
        if(t>(2*set_temp+1)) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"OFFOFF", 6, 1000);
        } else if(t<(2*set_temp-1)) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"ONONON", 6, 1000);
        } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"_SYNC_", 6, 1000);
        }
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"OFFOFF", 6, 1000);
    }

    HAL_Delay(10000);
  }
}

void run_actuator()
{
    uint8_t buf[6];
    uint32_t last_command = 0;
    int i;
    
    for(;;) {
        uint8_t c;
        if(HAL_OK == HAL_UART_Receive(&huart1, &c, 1, 1000)) {

            /* Shift in received character to buf */
            for(i=0;i<5;i++) {
                buf[i] = buf[i+1];
            }
            buf[5] = c;

            if(0==memcmp(buf, "ONONON", 6)) {
                HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
                last_command = HAL_GetTick();
            } else if( 0==memcmp(buf, "OFFOFF", 6)) {
                HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
                last_command = HAL_GetTick();
            } else if( 0==memcmp(buf, "_SYNC_", 6)) {
                last_command = HAL_GetTick();
            }
        }

        /* If no command in one minute, turn relay off */
        if(HAL_GetTick() - last_command > 60*1000) {
            HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
        }
    }
}

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */


  TM1637_init();

  if(L12S_config()) {
      /* Blink LED to indicate L12S config failed */
	  for(;;) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
	  }
  }

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  run_actuator();

  //run_thermalstat();


  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS1623_DQ_GPIO_Port, DS1623_DQ_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_Pin|DS1623_RST_N_Pin|DS1623_CLK_Pin|DS1623_DQ_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, TM1637_CLK_Pin|TM1637_IO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LC12S_SET_GPIO_Port, LC12S_SET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_DOWN_Pin */
  GPIO_InitStruct.Pin = BTN_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_DOWN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_DAT_Pin */
  GPIO_InitStruct.Pin = DHT11_DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_DAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_UP_Pin */
  GPIO_InitStruct.Pin = BTN_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_Pin DS1623_RST_N_Pin DS1623_CLK_Pin TM1637_CLK_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin|DS1623_RST_N_Pin|DS1623_CLK_Pin|TM1637_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DS1623_DQ_Pin */
  GPIO_InitStruct.Pin = DS1623_DQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS1623_DQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LC12S_SET_Pin */
  GPIO_InitStruct.Pin = LC12S_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LC12S_SET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TM1637_IO_Pin */
  GPIO_InitStruct.Pin = TM1637_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TM1637_IO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
