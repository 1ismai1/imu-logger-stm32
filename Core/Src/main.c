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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "ff.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t sd_buf[512];
FATFS fs;
FIL file;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void imu_task(void *arg);
void filter_task(void *arg);
void sd_task(void *arg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_ms(uint32_t ms) {
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    vTaskDelay(pdMS_TO_TICKS(ms));   // inside a task: sleep, CPU goes to other tasks
  else
    HAL_Delay(ms);                   // before FreeRTOS starts: busy-wait on HAL's tick
}

/* ---------------- UART2 ---------------- */
void uart2_init(void) {
  RCC->AHB1ENR |= (1 << 0);// GPIOA clock (from before)
  RCC->APB1ENR |= (1 << 17);//USARTEN enable
  	// USART2 clock (new)
  GPIOA->MODER &= ~(3 << (2*2));//Resetting the 2 pio's at pin 2 to 0
  GPIOA->MODER |= (2 << (2*2));
  GPIOA->MODER &= ~(3 << (2*3));
  GPIOA->MODER |= (2 << (2*3));
  GPIOA->AFR[0] &= ~(0xF << (4*2));
  GPIOA->AFR[0] |=  (7   << (4*2));   // PA2 → AF7 (USART2)
  GPIOA->AFR[0] &= ~(0xF << (4*3));
  GPIOA->AFR[0] |=  (7   << (4*3));   // PA3 → AF7 (USART2)

  USART2->BRR = (22 << 4) | 13; //Setting Baud Rate Register via conversion formula
  USART2->CR1 |= (1<<13); //Activating the USART2
  USART2->CR1 |= (1<<3); //Transmitter Enabled
  USART2->CR1 |= (1<<2); //Receiver Enabled
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

void sendInt(int16_t v) {
	char buf[7];
	int i = 6;
	uint16_t u;

	if (v < 0) { sendStr("-"); u = -(int32_t)v; } else u = v;

	buf[i] = '\0';
	do { buf[--i] = '0' + (u % 10); u /= 10; } while (u);
	sendStr(&buf[i]);
}

void sendFloat(float v) {
	uint16_t m;
	float u;
	float d;

	if (v < 0) { sendStr("-"); u = -(float)v; } else u = v;


	m = u;
	d = (u - m)*100;

	sendInt(m);
	sendStr(".");
	if (d < 10) {
			sendStr("0");
		}
	sendInt(d);
}

void sendHex(uint8_t b) {
	char hex[] ="0123456789ABCDEF";
	sendChar(hex[b >> 4]);
	sendChar(hex[b & 0x0F]);
}

/* ---------------- I2C1 ---------------- */
void i2c1_init(void) {
  RCC->AHB1ENR |= (1 << 1);   // GPIOB clock enable (bit 1 = port B)
  RCC->APB1ENR |= (1 << 21);// I2C1EN
  GPIOB->MODER &= ~(0xF << (8*2)); // resetting pins PB8 and PB9 to 0
  GPIOB->MODER |= (0xA << (8*2)); //setting PB8 and PB9 to AF mode (1010)

  GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
  GPIOB->AFR[1] |= (4 << 0) | (4 << 4);  // PB8 = AF4 (4*0), PB9 = AF4 (4*1) (I2C)

  GPIOB->OTYPER |= (1 << 8) | (1 << 9); //Setting pin 8 and 9 to open-drain
  GPIOB->PUPDR &= ~(0xF << 16); //Resetting pins
  GPIOB->PUPDR |= (0X5 << 16); //Setting the pins to 01 and 01

  I2C1->CR1 |= (1 << 15);    // SOFTWARE RESET or SWRST
  I2C1->CR1 &= ~(1 << 15);
  I2C1->CR2 |= (42 << 0); //Telling the I2C the clock speed (42MHz apb1)
  I2C1->CCR |= (210 << 0); //Using the formula from the ref sheet to set clock to 100kHz
  I2C1->TRISE = 43; //telling the I2C to wait 43 clock ticks before registering signal
  I2C1->CR1 |= (1 << 0); //Enabling peripheral
}

uint8_t i2c_wait(uint16_t mask) {
	uint32_t start = HAL_GetTick();
	while(!(I2C1->SR1 & mask)) {
		if (HAL_GetTick() - start > 2) return 0;
	}
	return 1;
}

uint8_t i2c_fail(void) {
	I2C1->CR1 |= (1<<9);
	I2C1->CR1 |= (1<<10);
	return 0;
}

uint8_t i2c_start(void) {
	I2C1->CR1 |= (1<<8); //generate a start condition
	return i2c_wait(1 << 0);
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

uint8_t i2c_write(uint8_t data) {
	if (!i2c_wait(1 << 7)) return 0;
	I2C1->DR = data;
	return i2c_wait(1 << 2);

}

void i2c_stop(void) {
	I2C1->CR1 |= (1<<9);
}



/* ---------------- SPI1 ---------------- */
void spi1_init(void) {
  RCC->AHB1ENR |= (1 << 0);// GPIOA clock (from before)
  RCC->AHB1ENR |= (1 << 1);   // GPIOB clock enable (bit 1 = port B)
  RCC->APB2ENR |= (1 << 12); //SPI1
  GPIOA->MODER &= ~((3 << 10 ) | (3 << 12) | (3 << 14));
  GPIOA->MODER |= ((2 << 10) | (2 << 12) | (2 << 14)); // RESETTING AND ENABLING AF FOR SPI

  GPIOA->AFR[0] &= ~((0xF << 20) | (0xF << 24) | (0xF << 28)); // RESETTING AND ASSIGNING AF5
  GPIOA->AFR[0] |= ((5 << 20) | (5 << 24) | (5 << 28));
  GPIOA->OSPEEDR |= (3 <<10) | (3 << 12) | (3 << 14); //fast pins

  GPIOB->MODER &= ~(3 << 12); //setting PB6 to be CS pin
  GPIOB->MODER |= (1 << 12); // set output
  GPIOB->BSRR = (1 << 6); //setting pin HIGH (basically setting it idle)

  GPIOA->PUPDR &= ~(3 << 12);
  GPIOA->PUPDR |= (1 << 12);

  SPI1->CR1 = (1 << 2) //MSTR:  stm32 is master
  	  	  	| (7 << 3) //BR: 84MHz / 256 = 328kHz (Sd cards need less than 400kHz)
			| (1 << 9) //SSM: handling CS in software
			| (1 << 8);//SSI: needed with SSM or SPI will shut off
  SPI1->CR1 |= (1 << 6); //SPE: turn SPI on


}

uint8_t spi_transfer(uint8_t b)
{
	while (!(SPI1->SR & (1 << 1))); //Waiting to send byte
	SPI1->DR = b;
	while(!(SPI1->SR & (1 << 0))); // waiting to receive bite
	return SPI1->DR; // read data
}

/* ---------------- SD card reader ---------------- */
void sd_select(void)
{
    GPIOB->BSRR = (1 << 22);   // CS LOW
}

void sd_deselect(void)
{
    GPIOB->BSRR = (1 << 6);    // CS HIGH
    spi_transfer(0xFF);        // extra byte so the card lets go of MISO
}

void sd_power_up(void)
{
    GPIOB->BSRR = (1 << 6);    // CS HIGH
    for (int i = 0; i < 10; i++) spi_transfer(0xFF);   // 80 clocks
}

uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r = 0xFF;

    sd_select();

    spi_transfer(0x40 | cmd);   // command byte
    spi_transfer(arg >> 24);    // argument, top byte first
    spi_transfer(arg >> 16);
    spi_transfer(arg >> 8);
    spi_transfer(arg);          // bottom byte
    spi_transfer(crc);

    for (int i = 0; i < 10; i++) {
        r = spi_transfer(0xFF);
        if (r != 0xFF) break;
    }

    return r;   // CS stays LOW on purpose (see below)
}

uint8_t sd_init(void)
{
	// Back to slow speed: the card must be woken up below 400 kHz
	SPI1->CR1 &= ~(1 << 6);   // SPI off (speed can only change while off)
	SPI1->CR1 |=  (7 << 3);   // BR = 111 → 84 MHz / 256 ≈ 328 kHz
	SPI1->CR1 |=  (1 << 6);   // SPI on
    uint8_t r;
    uint8_t buf[4];

    sd_power_up();

    // waiting for  card that may be stuck mid-read from a previous run
    sd_select();
    for (int i = 0; i < 600; i++) spi_transfer(0xFF);
    sd_deselect();

    sd_select();
    uint32_t t = 0;
    while (spi_transfer(0xFF) != 0xFF && ++t < 50000);   // up to ~1.5 s
    sd_deselect();
    // CMD0: go to SPI mode
    // CMD0: go to SPI mode. Retry, because a stuck card may ignore the first one
    for (int tries = 0; tries < 10; tries++) {
        r = sd_send_cmd(0, 0, 0x95);
        sd_deselect();
        if (r == 0x01) break;
        delay_ms(10);
    }
    if (r != 0x01) { sendStr("CMD0 reply = 0x"); sendHex(r); sendStr("\r\n"); return 1; }

    // CMD8: voltage check
    r = sd_send_cmd(8, 0x1AA, 0x87);
    for (int i = 0; i < 4; i++) buf[i] = spi_transfer(0xFF);
    sd_deselect();
    if (r != 0x01 || buf[2] != 0x01 || buf[3] != 0xAA) return 2;

    // ACMD41: start up (keep asking, max ~1 s)
    for (int tries = 0; tries < 1000; tries++) {
        sd_send_cmd(55, 0, 0x01);
        sd_deselect();
        r = sd_send_cmd(41, 0x40000000, 0x01);
        sd_deselect();
        if (r == 0) break;
        delay_ms(1);
    }
    if (r != 0) return 3;

    // CMD58: check it's a high-capacity card
    r = sd_send_cmd(58, 0, 0x01);
    for (int i = 0; i < 4; i++) buf[i] = spi_transfer(0xFF);
    sd_deselect();
    if (r != 0 || !(buf[0] & 0x40)) return 4;

    //Speeding up SPI to 84 MHz /32 = 2.6 MHz (limited by jumper cables. Change when your on PCB)
    SPI1->CR1 &= ~(1 << 6);
    SPI1->CR1 &= ~(7 << 3);
    SPI1->CR1 |= (2 << 3);
    SPI1->CR1 |= (1 << 6);

    return 0;
}

uint8_t sd_read_block(uint32_t block, uint8_t *buf)
{
	uint8_t r = sd_send_cmd(17, block, 0x01); //read from desired block
	if (r != 0) { sd_deselect(); return 1; }

	//wait for the 0xFE start token
	for (int i = 0; i < 100000; i++) {
		r =spi_transfer(0xFF);
		if (r == 0xFE) break;
	}
	if (r != 0xFE) {sd_deselect(); return 2; }

	// read through 512 bytes
	for (int i = 0; i < 512; i++) buf[i] = spi_transfer(0xFF);  // 0xFF, not 0xFE
	spi_transfer(0xFF);   // CRC byte 1 (ignored)
	spi_transfer(0xFF);   // CRC byte 2 (ignored)
	sd_deselect();        // release the card
	return 0;
}

uint8_t sd_write_block(uint32_t block, uint8_t *buf)
{
	uint8_t r = sd_send_cmd(24, block, 0x01);
	if (r != 0) {sd_deselect(); return 1; }

	spi_transfer(0xFF);
	spi_transfer(0xFE);

	for (int i = 0; i < 512; i++) spi_transfer(buf[i]); // the data being sent

	spi_transfer(0xFF);
	spi_transfer(0xFF);
	for (int k = 0; k < 10; k++) {
	    r = spi_transfer(0xFF);
	    if (r != 0xFF) break;
	}

	if ((r & 0x1F) != 0x05) { sd_deselect(); return 2; }

	uint32_t wait = 0;
	while(spi_transfer(0xFF) == 0) {
		if (++wait > 500000) { sd_deselect(); return 3; }
	}

	sd_deselect();
	return 0;
}
/* ---------------- MPU-6050 ---------------- */

#define MPU_ADDR  0x68      // 7-bit address
#define WHO_AM_I  0x75
#define PWR_MGMT_1  0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

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


uint8_t mpu_write_reg(uint8_t reg, uint8_t val) {
    if (!i2c_start() || !i2c_addr(MPU_ADDR, 0) ||
        !i2c_write(reg) || !i2c_write(val)) return i2c_fail();
    i2c_stop();
    return 1;
}

uint8_t mpu_init(void) {
    return mpu_write_reg(PWR_MGMT_1, 0x01) &&
           mpu_write_reg(ACCEL_CONFIG, 0x10) &&
           mpu_write_reg(GYRO_CONFIG, 0x18);
}

uint8_t mpu_read_burst(uint8_t reg, uint8_t *buf, uint8_t n) {
    if (!i2c_start() || !i2c_addr(MPU_ADDR, 0) || !i2c_write(reg)) return i2c_fail();

    I2C1->CR1 |= (1 << 10);                        // ACK on
    if (!i2c_start() || !i2c_addr(MPU_ADDR, 1)) return i2c_fail();

    while (n > 3) {
        if (!i2c_wait(1 << 6)) return i2c_fail();  // RXNE
        *buf++ = I2C1->DR;
        n--;
    }

    if (!i2c_wait(1 << 2)) return i2c_fail();      // BTF
    I2C1->CR1 &= ~(1 << 10);                       // ACK off
    *buf++ = I2C1->DR;
    I2C1->CR1 |= (1 << 9);                         // STOP
    *buf++ = I2C1->DR;
    if (!i2c_wait(1 << 6)) return i2c_fail();      // RXNE
    *buf++ = I2C1->DR;

    I2C1->CR1 |= (1 << 10);                        // ACK back on
    return 1;
}

/* ---------------- Task messages + queues ----------------*/
/* imu_task --[imu_q]--> filter_task --[log_q]--> sd_task --> SD card */
typedef struct {
	uint32_t ms;        // tick when sampled
	uint8_t  raw[14];   // the burst read, untouched
	uint8_t  restart;   // 1 = first sample after (re)connecting: re-snap the filters
} imu_msg_t;

typedef struct {
	uint32_t ms;
	int16_t  ax, ay, az, gx, gy, gz;     // raw counts (gyro bias removed)
	int16_t  roll_x100, pitch_x100;      // degrees x 100
} log_msg_t;

static QueueHandle_t imu_q;   // 16 samples:  filter is fast, small cushion is enough
static QueueHandle_t log_q;   // 64 rows = 0.64 s cushion for slow SD writes

// stats, printed once a second by sd_task
static volatile uint32_t imu_drops = 0;    // imu_q was full, sample thrown away
static volatile uint32_t log_drops = 0;    // log_q was full, row thrown away
static volatile uint32_t imu_faults = 0;   // failed reads (sensor unplugged / asleep)
static volatile uint32_t imu_q_max = 0;    // worst backlog seen in each queue
static volatile uint32_t log_q_max = 0;

static TaskHandle_t imu_h, filter_h, sd_h;   // so sd_task can ask each task how much stack it used

void debug_pin_init(void) {
  RCC->AHB1ENR |= (1 << 0);     // GPIOA clock on
  GPIOA->MODER &= ~(3 << 16);   // clear PA8's mode bits
  GPIOA->MODER |=  (1 << 16);   // 01 = output
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uart2_init();
  i2c1_init();
  spi1_init();
  debug_pin_init();

  delay_ms(100);              // MPU boot time
  mpu_probe();                // does anything answer at 0x68?

  if (!mpu_init()) sendStr("MPU init FAIL\r\n");
  delay_ms(100);


  uint8_t sd_err = sd_init();
  sendStr("SD init = ");
  sendInt(sd_err);
  sendStr("\r\n");


  FRESULT r;
  r = f_mount(&fs, "", 1);                               sendStr("mount "); sendInt(r); sendStr("\r\n");
  char name[] = "LOG000.CSV";
  for (int n = 0; n <1000; n++) {
	name[3] = '0' + n/100;
	name[4] = '0' + (n/10)%10;
	name[5] = '0' + n % 10;
	r = f_open(&file, name, FA_WRITE | FA_CREATE_NEW);
	if (r != FR_EXIST) break;
  }
  sendStr("file "); sendStr(name); sendStr(" open "); sendInt(r); sendStr("\r\n");
  if (r != FR_OK) { sendStr("NO LOG FILE - stopping\r\n"); while (1); }
  f_printf(&file, "ms,ax,ay,az,gx,gy,gz,roll_x100,pitch_x100\n");   // header row

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  imu_q = xQueueCreate(16, sizeof(imu_msg_t));
  log_q = xQueueCreate(64, sizeof(log_msg_t));
  if (!imu_q || !log_q) sendStr("queue create FAIL\r\n");
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Stack sizes are in words (4 bytes), sized from measured high-water marks
  // (used: imu 44, filter 96, sd ~132 words) with 2.7-3.9x margin.
  // Priorities on CubeMX's 0-55 scale:
  // sampling must never be late, SD writing is allowed to be late (the queues absorb it).
  if (xTaskCreate(imu_task,    "imu",    128,  NULL, osPriorityHigh,        &imu_h) != pdPASS ||  // 40
      xTaskCreate(filter_task, "filter", 256,  NULL, osPriorityAboveNormal, &filter_h) != pdPASS ||  // 32
      xTaskCreate(sd_task,     "sd",     512,  NULL, osPriorityBelowNormal, &sd_h) != pdPASS)    // 16
    sendStr("task create FAIL (out of heap?)\r\n");
  // heartbeat (defaultTask, created by CubeMX above) is Normal = 24
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hi2c1.Init.OwnAddress1 = 0;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ---------------- Tasks ---------------- */

/* imu_task: read the MPU-6050 every 10 ms (100 Hz), hand the raw bytes on. */
void imu_task(void *arg) {
  (void)arg;
  uint8_t restart = 1;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));   // wake exactly every 10 ms

    imu_msg_t m;
    GPIOA->BSRR = (1 << 8);                        // debug pin HIGH = reading
    uint8_t ok = mpu_read_burst(ACCEL_XOUT_H, m.raw, 14) &&
                 (m.raw[0] | m.raw[1] | m.raw[2] | m.raw[3] | m.raw[4] | m.raw[5]) != 0;  // all-zero = asleep (Bug 17)
    GPIOA->BSRR = (1 << (8 + 16));                 // debug pin LOW

    if (!ok) {                       // only this task touches I2C, so fix it right here
      imu_faults++;
      i2c1_init();                   // SWRST + reconfigure
      mpu_init();                    // sensor still missing? next read fails, we retry in 10 ms
      restart = 1;
      continue;
    }

    m.ms = xTaskGetTickCount();
    m.restart = restart;
    restart = 0;
    if (xQueueSend(imu_q, &m, 0) != pdPASS) imu_drops++;   // never wait: sampling can't be late
  }
}

/* filter_task: raw bytes -> scaled values -> roll/pitch (complementary filter). */
void filter_task(void *arg) {
  (void)arg;
  float angle = 0;     // filtered roll
  float pitch_f = 0;   // filtered pitch
  imu_msg_t m;

  for (;;) {
    xQueueReceive(imu_q, &m, portMAX_DELAY);       // sleep until a sample arrives
    uint32_t n = uxQueueMessagesWaiting(imu_q) + 1;
    if (n > imu_q_max) imu_q_max = n;

    uint8_t *raw = m.raw;
    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    //subtracting gyroscope biases
    gx -= -101;
    gy -= -52;
    gz -= -1;

    float ax_g = ax/4096.0f - 0.095f;
    float ay_g = ay/4096.0f + 0.025f;
    float az_g = az/4096.0f + 0.22f;

    float acc_angle = atan2f(ay_g, az_g) * 57.2958f;
    float acc_pitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 57.2958f;
    if (m.restart) {
      angle   = acc_angle;   // start both filters at the true angle, not 0
      pitch_f = acc_pitch;
    }
    float gx_dps = gx/16.4f;
    float gy_dps = gy/16.4f;

    float error = acc_angle - angle;
    if (error > 180)       error -= 360;
    else if (error < -180) error += 360;

    angle = (angle + gx_dps*0.01f) + 0.02f*error;
    if (angle > 180)       angle -= 360;
    else if (angle < -180) angle += 360;
    pitch_f = (pitch_f + gy_dps*0.01f) + 0.02f*(acc_pitch - pitch_f);
    if (pitch_f > 90)       pitch_f = 90;
    else if (pitch_f < -90) pitch_f = -90;

    log_msg_t row = { m.ms, ax, ay, az, gx, gy, gz,
                      (int16_t)(angle*100), (int16_t)(pitch_f*100) };
    if (xQueueSend(log_q, &row, 0) != pdPASS) log_drops++;
  }
}

/* sd_task: write CSV rows. Once per second of TIME (not per 100 rows, so it
   keeps reporting even if the sensor is gone): save the file and print stats.
   Every 10 s also print how much stack each task never touched.
   Lowest priority: if the SD card stalls, rows just pile up in log_q. */
void sd_task(void *arg) {
  (void)arg;
  sendStr("tasks running\r\n");

  // PA9 (Arduino D8) = HIGH while f_sync runs, for the logic analyzer
  GPIOA->MODER &= ~(3 << 18);
  GPIOA->MODER |=  (1 << 18);

  TickType_t next_report = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
  uint8_t reports = 0;
  log_msg_t r;

  for (;;) {
    // Wait for a row, but never past the next report time
    int32_t wait = (int32_t)(next_report - xTaskGetTickCount());
    if (wait < 0) wait = 0;

    if (xQueueReceive(log_q, &r, (TickType_t)wait) == pdPASS) {
      uint32_t n = uxQueueMessagesWaiting(log_q) + 1;
      if (n > log_q_max) log_q_max = n;
      f_printf(&file, "%lu, %d, %d, %d, %d, %d, %d, %d, %d\n",
               r.ms, r.ax, r.ay, r.az, r.gx, r.gy, r.gz, r.roll_x100, r.pitch_x100);
    }

    if ((int32_t)(xTaskGetTickCount() - next_report) >= 0) {   // 1 s passed
      next_report += pdMS_TO_TICKS(1000);

      GPIOA->BSRR = (1 << 9);                      // D8 HIGH = saving
      FRESULT fr = f_sync(&file);
      GPIOA->BSRR = (1 << (9 + 16));               // D8 LOW
      if (fr != FR_OK) sendStr("sync FAIL\r\n");

      sendStr("imuq ");  sendInt((int16_t)imu_q_max);
      sendStr(" logq "); sendInt((int16_t)log_q_max);
      sendStr(" drop "); sendInt((int16_t)imu_drops); sendStr("/"); sendInt((int16_t)log_drops);
      sendStr(" fault "); sendInt((int16_t)imu_faults); sendStr("\r\n");

      if (++reports >= 10) {                       // every 10 s
        reports = 0;
        // "High-water mark" = the LEAST free stack a task has ever had, in words.
        // Small number = that task came close to running out.
        sendStr("stack free (words): imu ");  sendInt((int16_t)uxTaskGetStackHighWaterMark(imu_h));
        sendStr(" filter ");                  sendInt((int16_t)uxTaskGetStackHighWaterMark(filter_h));
        sendStr(" sd ");                      sendInt((int16_t)uxTaskGetStackHighWaterMark(sd_h));
        sendStr(" hb ");                      sendInt((int16_t)uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle));
        sendStr(" | heap min free (bytes): "); sendInt((int16_t)xPortGetMinimumEverFreeHeapSize());
        sendStr("\r\n");
      }
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Heartbeat task: square wave on PA10 (Arduino D2), flips every 250 ms.
     Proves the scheduler is switching tasks. Watch it on the logic analyzer. */
  GPIOA->MODER &= ~(3 << 20);   // PA10 mode bits cleared
  GPIOA->MODER |=  (1 << 20);   // 01 = output
  uint8_t on = 0;
  for(;;)
  {
    on = !on;
    // BSRR, not ODR ^= : a read-modify-write of ODR could be interrupted by
    // imu_task changing PA8, and we'd write PA8's old value back.
    GPIOA->BSRR = on ? (1 << 10) : (1 << (10 + 16));
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
