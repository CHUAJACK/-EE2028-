/**
  ******************************************************************************
  * @file    hci_tl_interface.c
  * @brief   SPI3 transport between the STM32 and the on-board SPBTLE-RF
  *          (BlueNRG-MS) Bluetooth LE module on the B-L4S5I-IOT01A.
  *
  *          Adapted from ST X-CUBE-BLE1 (SensorDemo_BLESensor-App), with the
  *          pins and SPI settings ST uses for this module on the IoT node
  *          (STM32CubeL4 B-L475E-IOT01A BLE examples).
  *
  *          Pins: SPI3 SCK/MISO/MOSI = PC10/PC11/PC12, CS = PD13,
  *                RESET = PA8, IRQ = PE6 (EXTI line 6)
  ******************************************************************************
  */
#include <string.h>
#include "hci_tl.h"

#define HEADER_SIZE       5U
#define MAX_BUFFER_SIZE   255U
#define TIMEOUT_DURATION  15U    /* ms to wait for the module to accept a write */
#define SPI_TIMEOUT_MS    10U
#define ISR_MAX_READS     20U    /* safety limit; BLE_Process() re-triggers if more is pending */

/* BlueNRG SPI protocol: first byte of the 5-byte header */
#define BNRG_WRITE        0x0A
#define BNRG_READ         0x0B
#define BNRG_READY        0x02

static SPI_HandleTypeDef hspi_ble;
static uint8_t dummy_tx[HCI_READ_PACKET_SIZE];   /* 0xFF bytes clocked out while reading */

static void cs_low(void)  { HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_SET); }

static int32_t HCI_TL_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}

int32_t HCI_TL_SPI_Init(void* pConf)
{
  (void)pConf;
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_SPI3_CLK_ENABLE();

  /* The Wi-Fi module and the sub-GHz radio share SPI3: keep them deselected */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin   = ISM43362_SPI3_CSN_Pin;
  HAL_GPIO_Init(ISM43362_SPI3_CSN_GPIO_Port, &gpio);
  gpio.Pin   = SPSGRF_915_SPI3_CSN_Pin;
  HAL_GPIO_Init(SPSGRF_915_SPI3_CSN_GPIO_Port, &gpio);

  /* BlueNRG chip select (idle high) */
  cs_high();
  gpio.Pin   = HCI_TL_SPI_CS_PIN;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(HCI_TL_SPI_CS_PORT, &gpio);

  /* BlueNRG reset line (held in reset until HCI_TL_SPI_Reset) */
  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_RESET);
  gpio.Pin   = HCI_TL_RST_PIN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HCI_TL_RST_PORT, &gpio);

  /* BlueNRG IRQ line: rising edge = data ready */
  gpio.Pin   = HCI_TL_SPI_EXTI_PIN;
  gpio.Mode  = GPIO_MODE_IT_RISING;
  gpio.Pull  = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(HCI_TL_SPI_EXTI_PORT, &gpio);

  /* SPI3 pins */
  gpio.Pin       = INTERNAL_SPI3_SCK_Pin | INTERNAL_SPI3_MISO_Pin | INTERNAL_SPI3_MOSI_Pin;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(INTERNAL_SPI3_SCK_GPIO_Port, &gpio);

  /* SPI mode 0, MSB first. The core runs at 4 MHz (MSI), so /2 gives a 2 MHz clock. */
  hspi_ble.Instance               = SPI3;
  hspi_ble.Init.Mode              = SPI_MODE_MASTER;
  hspi_ble.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi_ble.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi_ble.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi_ble.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi_ble.Init.NSS               = SPI_NSS_SOFT;
  hspi_ble.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi_ble.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi_ble.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi_ble.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi_ble.Init.CRCPolynomial     = 7;
  hspi_ble.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi_ble.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi_ble) != HAL_OK) {
    return -1;
  }

  memset(dummy_tx, 0xFF, sizeof(dummy_tx));

  /* IRQ line interrupt. The EXTI9_5 handler in stm32l4xx_it.c forwards pin 6
     to HAL_GPIO_EXTI_Callback in main.c. */
  __HAL_GPIO_EXTI_CLEAR_IT(HCI_TL_SPI_EXTI_PIN);
  HAL_NVIC_SetPriority(HCI_TL_SPI_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(HCI_TL_SPI_EXTI_IRQn);
  return 0;
}

int32_t HCI_TL_SPI_DeInit(void)
{
  HAL_NVIC_DisableIRQ(HCI_TL_SPI_EXTI_IRQn);
  HAL_SPI_DeInit(&hspi_ble);
  HAL_GPIO_DeInit(HCI_TL_SPI_EXTI_PORT, HCI_TL_SPI_EXTI_PIN);
  HAL_GPIO_DeInit(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN);
  HAL_GPIO_DeInit(HCI_TL_RST_PORT, HCI_TL_RST_PIN);
  return 0;
}

int32_t HCI_TL_SPI_Reset(void)
{
  cs_high();   /* avoid spurious commands while the module boots */
  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
  return 0;
}

/* Reads one packet from the module. Runs in the EXTI interrupt. */
int32_t HCI_TL_SPI_Receive(uint8_t* buffer, uint16_t size)
{
  uint8_t header_master[HEADER_SIZE] = {BNRG_READ, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];
  uint16_t len = 0;

  cs_low();
  HAL_SPI_TransmitReceive(&hspi_ble, header_master, header_slave, HEADER_SIZE, SPI_TIMEOUT_MS);

  if (header_slave[0] == BNRG_READY) {
    uint16_t byte_count = (uint16_t)((header_slave[4] << 8) | header_slave[3]);
    if (byte_count > size) {
      byte_count = size;   /* never overrun the caller's buffer */
    }
    while (len < byte_count) {
      uint16_t chunk = byte_count - len;
      if (chunk > sizeof(dummy_tx)) {
        chunk = sizeof(dummy_tx);
      }
      HAL_SPI_TransmitReceive(&hspi_ble, dummy_tx, buffer + len, chunk, SPI_TIMEOUT_MS);
      len += chunk;
    }
  }
  cs_high();
  return len;
}

/* Writes one HCI command to the module. Runs in the main loop. */
int32_t HCI_TL_SPI_Send(uint8_t* buffer, uint16_t size)
{
  int32_t result;
  uint8_t header_master[HEADER_SIZE] = {BNRG_WRITE, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];
  static uint8_t read_char_buf[MAX_BUFFER_SIZE];
  uint32_t tickstart = HAL_GetTick();

  /* Keep the receive interrupt off the bus while we write.
     An IRQ edge that arrives meanwhile stays pending and runs right after. */
  HAL_NVIC_DisableIRQ(HCI_TL_SPI_EXTI_IRQn);
  do {
    result = 0;
    cs_low();
    HAL_SPI_TransmitReceive(&hspi_ble, header_master, header_slave, HEADER_SIZE, SPI_TIMEOUT_MS);

    if (header_slave[0] == BNRG_READY) {
      if (header_slave[1] >= size) {   /* module's write buffer has room */
        HAL_SPI_TransmitReceive(&hspi_ble, buffer, read_char_buf, size, SPI_TIMEOUT_MS);
      } else {
        result = -2;
      }
    } else {
      result = -1;   /* module still waking up */
    }
    cs_high();

    if ((HAL_GetTick() - tickstart) > TIMEOUT_DURATION) {
      result = -3;
      break;
    }
  } while (result < 0);
  HAL_NVIC_EnableIRQ(HCI_TL_SPI_EXTI_IRQn);

  return result;
}

static int32_t IsDataAvailable(void)
{
  return (HAL_GPIO_ReadPin(HCI_TL_SPI_EXTI_PORT, HCI_TL_SPI_EXTI_PIN) == GPIO_PIN_SET);
}

void hci_tl_lowlevel_init(void)
{
  tHciIO fops = {0};
  fops.Init    = HCI_TL_SPI_Init;
  fops.DeInit  = HCI_TL_SPI_DeInit;
  fops.Send    = HCI_TL_SPI_Send;
  fops.Receive = HCI_TL_SPI_Receive;
  fops.Reset   = HCI_TL_SPI_Reset;
  fops.GetTick = HCI_TL_GetTick;
  hci_register_io_bus(&fops);
}

void hci_tl_lowlevel_isr(void)
{
  uint32_t reads = 0;
  while (IsDataAvailable() && reads++ < ISR_MAX_READS) {
    if (hci_notify_asynch_evt(NULL)) {
      return;   /* no free packet buffers; BLE_Process() will retry */
    }
  }
}
