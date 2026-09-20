/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_ms(int ms) {
for (int i=0; i < ms; i++) {
  while (!(SysTick->CTRL & (1<<16)));
}
}
void sendChar(char c) {
	while (!(USART2->SR & (1<<7)));
	USART2->DR = c;
}
void sendStr(char *str) {
	while (*str) {
		sendChar(*str);
		str++;
	}
}

#define MPU_ADDR  0x68      // 7-bit address
#define WHO_AM_I  0x75
#define PWR_MGMT_1  0x6B
#define ACCEL_XOUT_H 0x3B

void sendHex(uint8_t b) {
	char hex[] ="0123456789ABCDEF";
	sendChar(hex[b >> 4]);
	sendChar(hex[b & 0x0F]);
}

void mpu_probe(void) {
	uint32_t t;

	sendStr("bus busy = ");
	sendHex((I2C1->SR2 >> 1) & 1);      // BUSY flag: 1 means line stuck low
	sendStr("\r\n");

	I2C1->CR1 |= (1 << 8);              // START
	t = 200000;
	while (!(I2C1->SR1 & (1 << 0))) {
		if (--t == 0) { sendStr("FAIL: no start condition\r\n"); return; }
	}
	sendStr("start ok\r\n");

	I2C1->DR = (MPU_ADDR << 1) | 0;     // address + write
	t = 200000;
	while (!(I2C1->SR1 & (1 << 1))) {
		if (I2C1->SR1 & (1 << 10)) {    // AF = acknowledge failure
			sendStr("FAIL: NACK - nothing at 0x68\r\n");
			I2C1->SR1 = ~(1 << 10);
			I2C1->CR1 |= (1 << 9);
			return;
		}
		if (--t == 0) {
			sendStr("FAIL: timeout waiting ADDR\r\n");
			I2C1->CR1 |= (1 << 9);
			return;
		}
	}
	(void)I2C1->SR1;
	(void)I2C1->SR2;
	sendStr("addr ok - device answered\r\n");
	I2C1->CR1 |= (1 << 9);              // STOP
}

void i2c_start(void) {
	I2C1->CR1 |= (1<<8); //generate a start condition
	while (!(I2C1->SR1 & (1 << 0))); //start condition was sent

}

uint8_t i2c_addr(uint8_t addr, uint8_t read) {
    uint32_t t = 200000;

    I2C1->DR = (addr << 1) | read;

    while (!(I2C1->SR1 & (1 << 1))) {       // wait for ADDR
        if (I2C1->SR1 & (1 << 10)) {        // AF = nobody answered
            I2C1->SR1 = ~(1 << 10);         // clear AF
            I2C1->CR1 |= (1 << 9);          // STOP, release the bus
            return 0;
        }
        if (--t == 0) {
            I2C1->CR1 |= (1 << 9);          // STOP
            return 0;
        }
    }

    (void)I2C1->SR1;
    (void)I2C1->SR2;                        // clear ADDR
    return 1;
}

void i2c_write(uint8_t data) {
	while (!(I2C1->SR1 & (1<<7)));
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1 << 2)));
}

void i2c_stop(void) {
	I2C1->CR1 |= (1<<9);
}

uint8_t mpu_read_reg(uint8_t reg) {
    uint8_t val;

    i2c_start();
    if (!i2c_addr(MPU_ADDR, 0)) { sendStr("NACK: write addr\r\n"); return 0xFF; }
    i2c_write(reg);

    i2c_start();
    I2C1->CR1 &= ~(1 << 10);            // ACK off before clearing ADDR
    I2C1->DR = (MPU_ADDR << 1) | 1;
    while (!(I2C1->SR1 & (1 << 1)));    // ADDR
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    I2C1->CR1 |= (1 << 9);              // STOP

    while (!(I2C1->SR1 & (1 << 6)));    // RXNE
    val = I2C1->DR;

    I2C1->CR1 |= (1 << 10);
    return val;
}

void mpu_write_reg(uint8_t reg, uint8_t val) {
	i2c_start();
	if (!i2c_addr(MPU_ADDR, 0)) { sendStr("NACK: write addr\r\n"); return; } //check if chip answers with a write request
	i2c_write(reg);
	i2c_write(val);
	i2c_stop();
}

void mpu_read_burst(uint8_t reg, uint8_t *buf, uint8_t n) {
	i2c_start();
	if (!i2c_addr(MPU_ADDR, 0)) { sendStr("NACK: burst wr\r\n"); return; }
	i2c_write(reg);

	I2C1->CR1 |= (1 << 10);
	i2c_start();
	if (!i2c_addr(MPU_ADDR, 1)) { sendStr("NACK: burst rd\r\n"); return; }

	while (n > 3) {
		while (!(I2C1->SR1 & (1 << 6)));
		*buf++ = I2C1->DR;
		n--;
	}

	while (!(I2C1->SR1 & (1<<2)));
		I2C1->CR1 &= ~(1 << 10);
		*buf++ = I2C1->DR;
		I2C1->CR1 |= (1 << 9);
		*buf++ = I2C1->DR;
		while(!(I2C1->SR1 & (1 << 6)));
		*buf++ = I2C1->DR;

		I2C1->CR1 |= (1 << 10);

}

void sendInt(int16_t v) {
	char buf[7];
	int i = 6;
	uint16_t u;

	if (v < 0) { sendStr("-"); u = -(int32_t)v; } else u = v;

	buf[i] = '\0';
	do { buf[--i] = '0' + (u % 10); u /= 10; } while (u);
	sendStr(&buf[i]);
}
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
  /* USER CODE BEGIN 2 */
  RCC->AHB1ENR |= (1 << 0);// GPIOA clock (from before)
  RCC->AHB1ENR |= (1 << 1);   // GPIOB clock enable (bit 1 = port B)
  RCC->APB1ENR |= (1 << 17);//USARTEN enable
  RCC->APB1ENR |= (1 << 21);// I2C1EN
  	// USART2 clock (new)
  GPIOA->MODER &= ~(3 << (2*2));//Resetting the 2 pio's at pin 2 to 0
  GPIOA->MODER |= (2 << (2*2));
  GPIOA->MODER &= ~(3 << (2*3));
  GPIOA->MODER |= (2 << (2*3));
  GPIOB->MODER &= ~(0xF << (8*2)); // resetting pins PB8 and PB9 to 0
  GPIOB->MODER |= (0xA << (8*2)); //setting PB8 and PB9 to AF mode (1010)


  GPIOA->AFR[0] &= ~(0xF << (4*2));
  GPIOA->AFR[0] |=  (7   << (4*2));   // PA2 → AF7 (USART2)
  GPIOA->AFR[0] &= ~(0xF << (4*3));
  GPIOA->AFR[0] |=  (7   << (4*3));   // PA3 → AF7 (USART2)
  GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
  GPIOB->AFR[1] |= (4 << 0) | (4 << 4);  // PB8 = AF4 (4*0), PB9 = AF4 (4*1) (I2C)

  USART2->BRR = (22 << 4) | 13; //Setting Baud Rate Register via conversion formula
  USART2->CR1 |= (1<<13); //Activating the USART2
  USART2->CR1 |= (1<<3); //Transmitter Enabled
  USART2->CR1 |= (1<<2); //Receiver Enabled

  GPIOB->OTYPER |= (1 << 8) | (1 << 9); //Setting pin 8 and 9 to open-drain
  GPIOB->PUPDR &= ~(0xF << 16); //Resetting pins
  GPIOB->PUPDR |= (0X5 << 16); //Setting the pins to 01 and 01

  I2C1->CR1 |= (1 << 15);    // SOFTWARE RESET or SWRST
  I2C1->CR1 &= ~(1 << 15);
  I2C1->CR2 |= (42 << 0); //Telling the I2C the clock speed (42MHz apb1)
  I2C1->CCR |= (210 << 0); //Using the formula from the ref sheet to set clock to 100kHz
  I2C1->TRISE = 43; //telling the I2C to wait 43 clock ticks before registering signal
  I2C1->CR1 |= (1 << 0); //Enabling peripheral


  SysTick->LOAD = 84000 - 1; //start count down from 1ms because 42MHz
  SysTick->VAL = 0; //resets count down to 0
  SysTick->CTRL = (1<<2) | (1<<0); //flips to on (on/off switch) and activates clock source

  delay_ms(100);              // MPU boot time
  mpu_probe();                // does anything answer at 0x68?

  mpu_write_reg(PWR_MGMT_1, 0x01);   // wake up
  delay_ms(100);

  // confirm it actually woke
  sendStr("PWR = 0x");
  sendHex(mpu_read_reg(PWR_MGMT_1));
  sendStr("\r\n");

  uint8_t raw[14];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		mpu_read_burst(ACCEL_XOUT_H, raw, 14);

		int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
		int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
		int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
		int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
		int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
		int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

		sendStr("A "); sendInt(ax); sendStr(" "); sendInt(ay); sendStr(" "); sendInt(az);
		sendStr("  G "); sendInt(gx); sendStr(" "); sendInt(gy); sendStr(" "); sendInt(gz);
		sendStr("\r\n");

		delay_ms(100);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
