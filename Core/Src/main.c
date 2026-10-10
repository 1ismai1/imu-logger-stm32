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
#include <string.h>
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
void sdw_task(void *arg);

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

void sendU32(uint32_t u) {          // sendInt tops out at 32767, counters can go higher
	char buf[11];
	int i = 10;
	buf[i] = '\0';
	do { buf[--i] = '0' + (u % 10); u /= 10; } while (u);
	sendStr(&buf[i]);
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

/* ---------------- SPI1 TX DMA (DMA2 Stream3, Channel 3) ----------------
   Sends a whole block to the SD card while the CPU does other work.
   FROM = buf (moves forward), TO = SPI1->DR (stays put), HOW MANY = n.
   SPI1 raises a DMA request every time TXE = 1 ("ready for the next byte"),
   the DMA copies one byte, and when the count hits 0 it fires an interrupt. */
#define SPI_DMA        DMA2_Stream3          // (Stream5 is kept free for USART1_RX -> ESP32 link later)
#define SPI_DMA_FLAGS  (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | \
                        DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3)

static TaskHandle_t     spi_dma_waiter = NULL;   // task sleeping until the block is sent
static volatile uint8_t spi_dma_err    = 0;

void spi1_dma_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;           // DMA2 clock on
  SPI_DMA->CR &= ~DMA_SxCR_EN;
  while (SPI_DMA->CR & DMA_SxCR_EN);            // stream must be fully off before setup

  SPI_DMA->PAR = (uint32_t)&SPI1->DR;           // TO: SPI data register (fixed)
  SPI_DMA->CR  = (3 << 25)                      // CHSEL = 3 -> this stream listens to SPI1_TX
               | (2 << 16)                      // PL = high priority
               | DMA_SxCR_MINC                  // memory side moves forward (PINC stays off!)
               | (1 << 6);                      // DIR = 01: memory -> peripheral
                                                // sizes = bytes, FIFO off (direct mode)

  // Priority 6: numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5),
  // otherwise the handler is NOT allowed to call FreeRTOS ...FromISR functions.
  NVIC_SetPriority(DMA2_Stream3_IRQn, 6);
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/* Runs when the DMA finishes (or hits an error): wake the task that started it. */
void DMA2_Stream3_IRQHandler(void) {
  BaseType_t woke = pdFALSE;
  uint32_t isr = DMA2->LISR;                    // streams 0-3 report in LISR, 4-7 in HISR
  DMA2->LIFCR = SPI_DMA_FLAGS;                  // clear this stream's flags
  if (isr & DMA_LISR_TEIF3) spi_dma_err = 1;
  if (spi_dma_waiter) vTaskNotifyGiveFromISR(spi_dma_waiter, &woke);
  portYIELD_FROM_ISR(woke);                     // switch straight to it if it outranks us
}

/* Send n bytes with DMA. Returns 1 = ok, 0 = timeout/error.
   Inside a task: the task SLEEPS until the DMA interrupt wakes it.
   Before the scheduler starts (f_mount etc.): no tasks yet, so just poll the flag. */
uint8_t spi1_dma_send(const uint8_t *buf, uint16_t n) {
  uint8_t in_task = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
  uint8_t ok = 1;

  while (!(SPI1->SR & SPI_SR_TXE));             // last polled byte handed over
  DMA2->LIFCR   = SPI_DMA_FLAGS;
  SPI_DMA->M0AR = (uint32_t)buf;                // FROM
  SPI_DMA->NDTR = n;                            // HOW MANY
  spi_dma_err   = 0;

  if (in_task) {
    ulTaskNotifyTake(pdTRUE, 0);                // throw away any stale wake-up
    spi_dma_waiter = xTaskGetCurrentTaskHandle();
    SPI_DMA->CR |=  (DMA_SxCR_TCIE | DMA_SxCR_TEIE);
  } else {
    spi_dma_waiter = NULL;
    SPI_DMA->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_TEIE);   // no interrupt: we poll TCIF ourselves
  }

  SPI_DMA->CR |= DMA_SxCR_EN;                   // copier armed...
  SPI1->CR2   |= SPI_CR2_TXDMAEN;               // ...and SPI1 may now request bytes

  if (in_task) {
    // 512 bytes at 10.5 MHz ~ 0.4 ms. Sleep; other tasks get the CPU.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) ok = 0;
  } else {
    uint32_t t = 2000000;
    while (!(DMA2->LISR & (DMA_LISR_TCIF3 | DMA_LISR_TEIF3)) && --t);
    if (!(DMA2->LISR & DMA_LISR_TCIF3)) ok = 0;
    DMA2->LIFCR = SPI_DMA_FLAGS;
  }
  if (spi_dma_err) ok = 0;
  spi_dma_waiter = NULL;

  SPI1->CR2   &= ~SPI_CR2_TXDMAEN;
  SPI_DMA->CR &= ~DMA_SxCR_EN;
  while (SPI_DMA->CR & DMA_SxCR_EN);

  // "DMA done" only means the last byte was handed to SPI1, not that it left the pin.
  while (!(SPI1->SR & SPI_SR_TXE));
  while (SPI1->SR & SPI_SR_BSY);

  // SPI receives a byte for every byte it sends. Nobody read those 512 replies,
  // so RXNE and OVR (overrun) are set. Clear them: read DR, then read SR.
  (void)SPI1->DR;
  (void)SPI1->SR;
  return ok;
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

uint8_t sd_write_block(uint32_t block, const uint8_t *buf)
{
	uint8_t r = sd_send_cmd(24, block, 0x01);
	if (r != 0) {sd_deselect(); return 1; }

	spi_transfer(0xFF);
	spi_transfer(0xFE);                             // start token

	if (!spi1_dma_send(buf, 512)) { sd_deselect(); return 4; }   // the 512 data bytes, by DMA

	spi_transfer(0xFF);                             // CRC (ignored in SPI mode)
	spi_transfer(0xFF);
	for (int k = 0; k < 10; k++) {
	    r = spi_transfer(0xFF);
	    if (r != 0xFF) break;
	}

	if ((r & 0x1F) != 0x05) { sd_deselect(); return 2; }   // 0x05 = data accepted

	// The card holds MISO low (reads 0x00) while it programs its flash.
	// Usually short, sometimes 100+ ms. Spin briefly, then SLEEP 1 ms between checks
	// instead of burning the CPU.
	TickType_t t0 = xTaskGetTickCount();
	uint32_t spins = 0;
	while (spi_transfer(0xFF) == 0x00) {
		if (++spins < 100) continue;                 // ~0.15 ms of quick checks
		if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
			if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(500)) { sd_deselect(); return 3; }
			vTaskDelay(1);
		} else if (spins > 500000) { sd_deselect(); return 3; }
	}

	sd_deselect();
	return 0;
}
/* ---------------- MPU-6050 ---------------- */

#define MPU_ADDR  0x68      // 7-bit address
#define WHO_AM_I  0x75
#define PWR_MGMT_1  0x6B
#define ACCEL_XOUT_H 0x3B
#define MPU_CONFIG  0x1A    // DLPF_CFG in bits 2:0 (the sensor's built-in low-pass filter)
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

/* DLPF_CFG = 3: accel 44 Hz / gyro 42 Hz bandwidth, ~4.8 ms delay.
   We sample at 100 Hz, so anything above 50 Hz (motor/prop vibration on a drone)
   would alias - fold down and show up as a fake slow wobble. Cutting it off
   inside the sensor, before we sample, is the only place it can be removed.
   Side effect: the gyro's internal rate drops from 8 kHz to 1 kHz (still 10x ours). */
#define MPU_DLPF_44HZ 0x03

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
           mpu_write_reg(MPU_CONFIG, MPU_DLPF_44HZ) &&   // vibration filter (also re-applied after a reconnect)
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
/* ^ polling version, no longer used by the tasks. Kept to compare against the DMA
     version on the logic analyzer (PA8 high time). */

/* ---------------- I2C1 RX DMA (DMA1 Stream5, Channel 1) ----------------
   FROM = I2C1->DR (stays put), TO = buf (moves forward), HOW MANY = n.
   I2C1 raises a DMA request every time RXNE = 1 ("a byte just landed in DR"). */
#define I2C_DMA        DMA1_Stream5
#define I2C_DMA_FLAGS  (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | \
                        DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5)

static TaskHandle_t     i2c_dma_waiter = NULL;
static volatile uint8_t i2c_dma_err    = 0;

void i2c1_dma_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;           // DMA1 clock on
  I2C_DMA->CR &= ~DMA_SxCR_EN;
  while (I2C_DMA->CR & DMA_SxCR_EN);

  I2C_DMA->PAR = (uint32_t)&I2C1->DR;           // FROM: I2C data register (fixed)
  I2C_DMA->CR  = (1 << 25)                      // CHSEL = 1 -> this stream listens to I2C1_RX
               | (3 << 16)                      // PL = very high: ST errata says I2C DMA must never be kept waiting
               | DMA_SxCR_MINC                  // memory side moves forward (PINC stays off!)
               | DMA_SxCR_TCIE | DMA_SxCR_TEIE; // interrupt on done / error
                                                // DIR = 00: peripheral -> memory, bytes, direct mode
  NVIC_SetPriority(DMA1_Stream5_IRQn, 6);       // >= 5 so it may call FreeRTOS FromISR
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

void DMA1_Stream5_IRQHandler(void) {
  BaseType_t woke = pdFALSE;
  uint32_t isr = DMA1->HISR;
  DMA1->HIFCR = I2C_DMA_FLAGS;
  if (isr & DMA_HISR_TCIF5) I2C1->CR1 |= I2C_CR1_STOP;   // last byte is in: release the bus now
  if (isr & DMA_HISR_TEIF5) i2c_dma_err = 1;
  if (i2c_dma_waiter) vTaskNotifyGiveFromISR(i2c_dma_waiter, &woke);
  portYIELD_FROM_ISR(woke);
}

static void i2c_dma_stop(void) {
  I2C1->CR2   &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
  I2C_DMA->CR &= ~DMA_SxCR_EN;
  while (I2C_DMA->CR & DMA_SxCR_EN);
  i2c_dma_waiter = NULL;
  DMA1->HIFCR = I2C_DMA_FLAGS;
}

/* Burst-read n (>= 2) registers starting at reg. Task context only.
   The CPU sends the register address (1 byte, fast), then the DMA collects
   all n bytes while this task sleeps. Returns 1 = ok, 0 = fail. */
uint8_t mpu_read_burst_dma(uint8_t reg, uint8_t *buf, uint8_t n) {
  if (n < 2) return 0;

  // 1) Tell the sensor where to start reading (CPU, polling - it's 1 byte)
  if (!i2c_start() || !i2c_addr(MPU_ADDR, 0) || !i2c_write(reg)) return i2c_fail();

  // 2) Arm the copier: TO = buf, HOW MANY = n (FROM was set once in init)
  I2C_DMA->CR &= ~DMA_SxCR_EN;
  while (I2C_DMA->CR & DMA_SxCR_EN);
  DMA1->HIFCR   = I2C_DMA_FLAGS;
  I2C_DMA->M0AR = (uint32_t)buf;
  I2C_DMA->NDTR = n;
  i2c_dma_err   = 0;
  ulTaskNotifyTake(pdTRUE, 0);                  // throw away any stale wake-up
  i2c_dma_waiter = xTaskGetCurrentTaskHandle();
  I2C_DMA->CR |= DMA_SxCR_EN;

  // 3) Let I2C1 request the DMA. LAST = hardware NACKs the final byte by itself
  //    (with polling, we had to do the BTF / ACK-off dance by hand for this).
  I2C1->CR1 |= I2C_CR1_ACK;
  I2C1->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;

  // 4) Repeated START + address with read bit. Clearing ADDR (inside i2c_addr)
  //    starts the bytes flowing - from here on it's all hardware.
  if (!i2c_start() || !i2c_addr(MPU_ADDR, 1)) { i2c_dma_stop(); return i2c_fail(); }

  // 5) Sleep until the DMA interrupt wakes us. 14 bytes at 100 kHz ~ 1.3 ms.
  uint8_t ok = (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5)) != 0) && !i2c_dma_err;
  i2c_dma_stop();
  if (!ok) return i2c_fail();

  // STOP was requested in the interrupt; hardware clears the bit once it's sent.
  uint32_t t = 100000;
  while ((I2C1->CR1 & I2C_CR1_STOP) && --t);
  return 1;
}

/* ---------------- Task messages + queues ----------------*/
/* imu_task --[imu_q]--> filter_task --[log_q]--> sd_task --[full_q]--> sdw_task --> SD card
                                                      ^------[free_q]------'                */
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

/* ---------------- Ping-pong SD blocks ----------------
   Two 512-byte blocks. sd_task (the formatter) fills one with CSV text while
   sdw_task (the writer) DMAs the other one to the card. They never touch the
   same block: a block index is either in free_q (empty, formatter's), in full_q
   (waiting for the writer) or being worked on by exactly one task. Swapping
   = passing a 1-byte index through a queue. No 512-byte copies.
   Every write is a whole, block-aligned 512 bytes, so FatFS hands blk[i]
   straight to disk_write -> the DMA reads directly from our buffer. */
#define BLK 512
static uint8_t blk[2][BLK] __attribute__((aligned(4)));
static QueueHandle_t free_q;   // empty blocks (formatter takes from here)
static QueueHandle_t full_q;   // full blocks  (writer takes from here)

static volatile uint32_t blocks_written = 0;
static volatile uint32_t blk_waits = 0;    // formatter had to wait for an empty block (writer too slow)
static volatile uint32_t sd_errors = 0;    // failed f_write / f_sync

/* ---------------- Gyro bias, measured at power-on ----------------
   A still gyro should read 0, but each one has an offset (bias) that changes
   with temperature, so hard-coded numbers go stale. filter_task averages the
   first 2 s of samples. If the board moved during that window (any axis swung
   more than GYRO_CAL_MAX_SPAN counts), it tries again; after 5 tries it keeps
   the old hand-measured values. Logging runs normally the whole time. */
#define GYRO_CAL_SAMPLES   200   // 2 s at 100 Hz
#define GYRO_CAL_MAX_SPAN  50    // counts, ~3 deg/s at 16.4 counts per deg/s
#define GYRO_CAL_TRIES     5

static volatile int16_t gyro_bias[3] = { -101, -52, -1 };   // fallback: the old hard-coded values
static volatile uint8_t gyro_cal_state = 0;   // 0 = measuring, 1 = measured, 2 = kept moving -> fallback

static TaskHandle_t imu_h, filter_h, sd_h, sdw_h;   // so sd_task can ask each task how much stack it used

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
  i2c1_dma_init();
  spi1_init();
  spi1_dma_init();
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
  // (header row is now written by sd_task into the first ping-pong block,
  //  so every write to the file stays a whole, aligned 512-byte block)

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
  free_q = xQueueCreate(2, sizeof(uint8_t));
  full_q = xQueueCreate(2, sizeof(uint8_t));
  if (!imu_q || !log_q || !free_q || !full_q) sendStr("queue create FAIL\r\n");
  for (uint8_t i = 0; i < 2; i++) xQueueSend(free_q, &i, 0);   // both blocks start empty
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Stack sizes are in words (4 bytes), sized from measured high-water marks
  // (used: imu 44, filter 96, sd ~132 words) with 2.7-3.9x margin.
  // sd was split into sd (formatter, no FatFS now) + sdw (writer, all FatFS calls):
  // measured 10 Oct: sd used 78 words (256 = 3.3x), sdw used 119 (384 = 3.2x).
  // Priorities on CubeMX's 0-55 scale:
  // sampling must never be late, SD writing is allowed to be late (the queues absorb it).
  // Writer is lowest: it mostly sleeps (DMA / card busy), and it must never
  // starve the formatter that keeps log_q drained.
  if (xTaskCreate(imu_task,    "imu",    128,  NULL, osPriorityHigh,        &imu_h) != pdPASS ||  // 40
      xTaskCreate(filter_task, "filter", 256,  NULL, osPriorityAboveNormal, &filter_h) != pdPASS ||  // 32
      xTaskCreate(sd_task,     "sd",     256,  NULL, osPriorityBelowNormal, &sd_h) != pdPASS ||    // 16
      xTaskCreate(sdw_task,    "sdw",    384,  NULL, osPriorityLow,         &sdw_h) != pdPASS)     // 8
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

/* imu_task: read the MPU-6050 every 10 ms (100 Hz), hand the raw bytes on.
   The 14-byte read is done by DMA: this task sleeps ~1.3 ms while the bytes arrive. */
void imu_task(void *arg) {
  (void)arg;
  uint8_t restart = 1;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));   // wake exactly every 10 ms

    imu_msg_t m;
    GPIOA->BSRR = (1 << 8);                        // debug pin HIGH = reading
    uint8_t ok = mpu_read_burst_dma(ACCEL_XOUT_H, m.raw, 14) &&
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

  // gyro bias calibration window
  int32_t  cal_sum[3] = {0, 0, 0};
  int16_t  cal_min[3] = {0, 0, 0}, cal_max[3] = {0, 0, 0};
  uint16_t cal_n = 0;
  uint8_t  cal_tries = 0;

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

    // Calibrating: collect raw gyro readings (before any bias is removed)
    if (gyro_cal_state == 0) {
      int16_t g[3] = { gx, gy, gz };
      if (m.restart) cal_n = 0;                    // sensor just (re)connected: start the window over
      for (int k = 0; k < 3; k++) {
        if (cal_n == 0) { cal_sum[k] = 0; cal_min[k] = cal_max[k] = g[k]; }
        cal_sum[k] += g[k];
        if (g[k] < cal_min[k]) cal_min[k] = g[k];
        if (g[k] > cal_max[k]) cal_max[k] = g[k];
      }
      if (++cal_n == GYRO_CAL_SAMPLES) {
        uint8_t still = 1;
        for (int k = 0; k < 3; k++)
          if (cal_max[k] - cal_min[k] > GYRO_CAL_MAX_SPAN) still = 0;

        if (still) {
          for (int k = 0; k < 3; k++) {            // average, rounded to the nearest count
            int32_t s = cal_sum[k];
            gyro_bias[k] = (int16_t)((s >= 0 ? s + GYRO_CAL_SAMPLES/2 : s - GYRO_CAL_SAMPLES/2) / GYRO_CAL_SAMPLES);
          }
          gyro_cal_state = 1;
        } else if (++cal_tries >= GYRO_CAL_TRIES) {
          gyro_cal_state = 2;                      // never held still: keep the fallback values
        }
        cal_n = 0;                                 // (if moved, the next window starts now)
      }
    }

    // subtract gyroscope bias (fallback values until calibration finishes)
    gx -= gyro_bias[0];
    gy -= gyro_bias[1];
    gz -= gyro_bias[2];

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

/* ---- CSV formatting into the ping-pong blocks (sd_task only) ---- */
static uint8_t  fmt_cur;    // index of the block the formatter is filling
static uint16_t fmt_used;   // bytes already in it

static char *put_u32(char *p, uint32_t u) {
  char t[10];
  int i = 0;
  do { t[i++] = '0' + (u % 10); u /= 10; } while (u);
  while (i) *p++ = t[--i];
  return p;
}

static char *put_i16(char *p, int16_t v) {
  if (v < 0) { *p++ = '-'; return put_u32(p, (uint32_t)(-(int32_t)v)); }
  return put_u32(p, (uint32_t)v);
}

/* Add text to the current block. When it fills up: hand it to the writer and
   SWAP to the other block (no copying - just pass the index along). */
static void blk_append(const char *s, uint16_t len) {
  while (len) {
    uint16_t room = BLK - fmt_used;
    uint16_t k = (len < room) ? len : room;   // a row may be split across two blocks - that's fine,
    memcpy(&blk[fmt_cur][fmt_used], s, k);    // the file is just one long stream of bytes
    fmt_used += k; s += k; len -= k;

    if (fmt_used == BLK) {
      xQueueSend(full_q, &fmt_cur, portMAX_DELAY);          // full -> writer (never waits: only 2 blocks exist)
      if (xQueueReceive(free_q, &fmt_cur, 0) != pdPASS) {   // grab the other block
        blk_waits++;                                        // writer still busy with it: wait
        xQueueReceive(free_q, &fmt_cur, portMAX_DELAY);     // (rows pile up in log_q meanwhile)
      }
      fmt_used = 0;
    }
  }
}

/* sd_task (formatter): turn log rows into CSV text inside the ping-pong blocks.
   Never touches FatFS or SPI - that's sdw_task's job.
   Once per second of TIME (so it keeps reporting even if the sensor is gone): print stats.
   Every 10 s also print how much stack each task never touched. */
void sd_task(void *arg) {
  (void)arg;
  sendStr("tasks running\r\n");

  xQueueReceive(free_q, &fmt_cur, portMAX_DELAY);   // first empty block
  fmt_used = 0;
  static const char header[] = "ms,ax,ay,az,gx,gy,gz,roll_x100,pitch_x100\n";
  blk_append(header, sizeof(header) - 1);

  TickType_t next_report = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
  uint8_t reports = 0;
  uint8_t cal_reported = 0;
  log_msg_t r;
  char line[80];   // longest row is ~75 chars

  for (;;) {
    // Wait for a row, but never past the next report time
    int32_t wait = (int32_t)(next_report - xTaskGetTickCount());
    if (wait < 0) wait = 0;

    if (xQueueReceive(log_q, &r, (TickType_t)wait) == pdPASS) {
      uint32_t n = uxQueueMessagesWaiting(log_q) + 1;
      if (n > log_q_max) log_q_max = n;

      // same format as the old f_printf: "ms, ax, ay, az, gx, gy, gz, roll, pitch\n"
      int16_t v[8] = { r.ax, r.ay, r.az, r.gx, r.gy, r.gz, r.roll_x100, r.pitch_x100 };
      char *p = put_u32(line, r.ms);
      for (int i = 0; i < 8; i++) { *p++ = ','; *p++ = ' '; p = put_i16(p, v[i]); }
      *p++ = '\n';
      blk_append(line, (uint16_t)(p - line));
    }

    if ((int32_t)(xTaskGetTickCount() - next_report) >= 0) {   // 1 s passed
      next_report += pdMS_TO_TICKS(1000);

      sendStr("imuq ");  sendInt((int16_t)imu_q_max);
      sendStr(" logq "); sendInt((int16_t)log_q_max);
      sendStr(" drop "); sendInt((int16_t)imu_drops); sendStr("/"); sendInt((int16_t)log_drops);
      sendStr(" fault "); sendInt((int16_t)imu_faults);
      sendStr(" blk ");  sendU32(blocks_written);
      sendStr(" wait "); sendU32(blk_waits);
      sendStr(" sderr "); sendU32(sd_errors); sendStr("\r\n");

      if (!cal_reported && gyro_cal_state != 0) {  // once, when calibration ends
        cal_reported = 1;
        sendStr(gyro_cal_state == 1 ? "gyro bias measured: " : "gyro moved during calibration, using defaults: ");
        sendInt(gyro_bias[0]); sendStr(" "); sendInt(gyro_bias[1]); sendStr(" "); sendInt(gyro_bias[2]);
        sendStr("\r\n");
      }

      if (++reports >= 10) {                       // every 10 s
        reports = 0;
        // "High-water mark" = the LEAST free stack a task has ever had, in words.
        // Small number = that task came close to running out.
        sendStr("stack free (words): imu ");  sendInt((int16_t)uxTaskGetStackHighWaterMark(imu_h));
        sendStr(" filter ");                  sendInt((int16_t)uxTaskGetStackHighWaterMark(filter_h));
        sendStr(" sd ");                      sendInt((int16_t)uxTaskGetStackHighWaterMark(sd_h));
        sendStr(" sdw ");                     sendInt((int16_t)uxTaskGetStackHighWaterMark(sdw_h));
        sendStr(" hb ");                      sendInt((int16_t)uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle));
        sendStr(" | heap min free (bytes): "); sendInt((int16_t)xPortGetMinimumEverFreeHeapSize());
        sendStr("\r\n");
      }
    }
  }
}

/* sdw_task (writer): the ONLY task that touches FatFS / SPI.
   Takes a full block, writes it (FatFS -> disk_write -> SPI DMA straight from
   blk[i]), gives the block back as empty. Saves the file once a second.
   Lowest priority: while it sleeps on the DMA or the card's busy time,
   the formatter keeps filling the other block. */
void sdw_task(void *arg) {
  (void)arg;

  // PA9 (Arduino D8) = HIGH while writing a block or saving, for the logic analyzer
  GPIOA->MODER &= ~(3 << 18);
  GPIOA->MODER |=  (1 << 18);

  TickType_t next_sync = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
  uint8_t i;

  for (;;) {
    int32_t wait = (int32_t)(next_sync - xTaskGetTickCount());
    if (wait < 0) wait = 0;

    if (xQueueReceive(full_q, &i, (TickType_t)wait) == pdPASS) {
      UINT bw = 0;
      GPIOA->BSRR = (1 << 9);                      // D8 HIGH = writing
      FRESULT fr = f_write(&file, blk[i], BLK, &bw);
      GPIOA->BSRR = (1 << (9 + 16));               // D8 LOW
      if (fr != FR_OK || bw != BLK) sd_errors++;
      else blocks_written++;
      xQueueSend(free_q, &i, 0);                   // block is empty again -> formatter
    }

    if ((int32_t)(xTaskGetTickCount() - next_sync) >= 0) {   // 1 s passed
      next_sync += pdMS_TO_TICKS(1000);
      GPIOA->BSRR = (1 << 9);
      if (f_sync(&file) != FR_OK) sd_errors++;     // update file size on the card
      GPIOA->BSRR = (1 << (9 + 16));
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
