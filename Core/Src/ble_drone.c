/**
  ******************************************************************************
  * @file    ble_drone.c
  * @brief   Bluetooth LE telemetry for the drone (SPBTLE-RF / BlueNRG-MS).
  *          See ble_drone.h for the GATT layout.
  *          Init and event handling follow ST X-CUBE-BLE1 SensorDemo.
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>

#include "ble_drone.h"
#include "main.h"

#include "hci.h"
#include "hci_le.h"
#include "hci_tl.h"
#include "hci_const.h"
#include "bluenrg_types.h"
#include "bluenrg_def.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_utils.h"
#include "link_layer.h"

#define BDADDR_LEN          6
#define ADV_INTERVAL_MIN    160    /* x 0.625 ms = 100 ms */
#define ADV_INTERVAL_MAX    320    /* x 0.625 ms = 200 ms */
#define ADV_RETRY_MS        1000
#define MAX_TIMEOUTS        3      /* module unresponsive this many times in a row -> BLE off */
#define FLUSH_MS            250    /* BLE_Shutdown: time for queued updates to reach the laptop */
#define DISCONNECT_WAIT_MS  500    /* BLE_Shutdown: max wait for the disconnect to finish */

#define SENSORS_LEN         13
#define STATUS_LEN          7
#define LASER_LEN           5

/* Last byte of the first UUID group: e20280xx-6c61-7365-7200-647230ee2028 */
#define UUID_ID_SERVICE     0x00
#define UUID_ID_SENSORS     0x01
#define UUID_ID_STATUS      0x02
#define UUID_ID_LASER       0x03

static volatile uint8_t ble_ready = 0;
static volatile uint8_t connected = 0;
static volatile uint8_t set_connectable = 0;
static uint32_t adv_retry_at = 0;
static uint16_t conn_handle;

static uint8_t  bdaddr[BDADDR_LEN];
static uint8_t  bdaddr_valid = 0;

static uint16_t serviceHandle;
static uint16_t sensorsCharHandle;
static uint16_t statusCharHandle;
static uint16_t laserCharHandle;

/* Builds e20280xx-6c61-7365-7200-647230ee2028 in the LSB-first order BlueNRG expects */
static void make_uuid(uint8_t out[16], uint8_t id)
{
  static const uint8_t msb_first[16] = {
    0xe2, 0x02, 0x80, 0x00, 0x6c, 0x61, 0x73, 0x65,
    0x72, 0x00, 0x64, 0x72, 0x30, 0xee, 0x20, 0x28
  };
  for (int i = 0; i < 16; i++) {
    out[i] = msb_first[15 - i];
  }
  out[12] = id;   /* 4th byte when written MSB first */
}

static void put_le16(uint8_t *buf, uint16_t v)
{
  buf[0] = (uint8_t)(v & 0xFF);
  buf[1] = (uint8_t)(v >> 8);
}

static int16_t scale_i16(float v, float scale)
{
  float x = v * scale;
  if (x != x) return 0;   /* NaN */
  if (x > 32767.0f)  return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)(x >= 0 ? x + 0.5f : x - 0.5f);
}

static uint16_t scale_u16(float v, float scale)
{
  float x = v * scale;
  if (x != x || x <= 0.0f) return 0;
  if (x > 65535.0f) return 65535;
  return (uint16_t)(x + 0.5f);
}

/* LED4 (blue, BLE) and LED3 (yellow, Wi-Fi) share PC9:
   low = LED4 on, floating = both off. */
static void ble_led(int on)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  gpio.Pin   = LED3_WIFI__LED4_BLE_Pin;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  if (on) {
    HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
  } else {
    gpio.Mode = GPIO_MODE_INPUT;
  }
  HAL_GPIO_Init(LED3_WIFI__LED4_BLE_GPIO_Port, &gpio);
}

static void ble_event_handler(void *pData)
{
  hci_uart_pckt *hci_pckt = pData;
  hci_event_pckt *event_pckt = (hci_event_pckt *)hci_pckt->data;

  if (hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }

  switch (event_pckt->evt) {
  case EVT_DISCONN_COMPLETE:
    connected = 0;
    set_connectable = 1;   /* advertise again so the laptop can reconnect */
    ble_led(0);
    break;

  case EVT_LE_META_EVENT: {
    evt_le_meta_event *evt = (void *)event_pckt->data;
    if (evt->subevent == EVT_LE_CONN_COMPLETE) {
      evt_le_connection_complete *cc = (void *)evt->data;
      if (cc->status == BLE_STATUS_SUCCESS) {
        connected = 1;
        conn_handle = cc->handle;
        ble_led(1);
      } else {
        set_connectable = 1;
      }
    }
    break;
  }

  default:
    break;
  }
}

static int start_advertising(void)
{
  char local_name[1 + sizeof(BLE_DEVICE_NAME) - 1];
  uint8_t scan_resp[2 + 16];

  local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(&local_name[1], BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1);

  /* The 128-bit service UUID does not fit next to the name, so it goes in the scan response */
  scan_resp[0] = 17;
  scan_resp[1] = AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST;
  make_uuid(&scan_resp[2], UUID_ID_SERVICE);
  hci_le_set_scan_resp_data(sizeof(scan_resp), scan_resp);

  return aci_gap_set_discoverable(ADV_IND, ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
                                  STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                  sizeof(local_name), local_name,
                                  0, NULL, 0, 0);
}

static int add_char(uint8_t id, uint8_t len, uint16_t *handle)
{
  uint8_t uuid[16];
  make_uuid(uuid, id);
  return aci_gatt_add_char(serviceHandle, UUID_TYPE_128, uuid, len,
                           CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,   /* reads return the stored value */
                           16, 0, handle);
}

/* Turns the radio off: holds the module in reset, after which every BLE_ call does nothing */
static void ble_shutdown(void)
{
  HAL_NVIC_DisableIRQ(HCI_TL_SPI_EXTI_IRQn);
  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_RESET);   /* hold module in reset */
  ble_ready = 0;
  connected = 0;
  set_connectable = 0;
  ble_led(0);
}

/* Handles events from the module (runs ble_event_handler for each one) */
static void poll_events(void)
{
  hci_user_evt_proc();

  /* If the IRQ line is still high (events waited for a free buffer), run the
     interrupt again so they are read. A missed edge would otherwise stall BLE. */
  if (HAL_GPIO_ReadPin(HCI_TL_SPI_EXTI_PORT, HCI_TL_SPI_EXTI_PIN) == GPIO_PIN_SET) {
    __HAL_GPIO_EXTI_GENERATE_SWIT(HCI_TL_SPI_EXTI_PIN);
  }
}

int BLE_Init(void)
{
  uint8_t  hwVersion = 0;
  uint16_t fwVersion = 0;
  uint8_t  len_out = 0;
  uint16_t gapService, devNameChar, appearanceChar;
  uint8_t  uuid[16];
  int      ok = 0;

  ble_ready = 0;
  connected = 0;
  ble_led(0);

  hci_init(ble_event_handler, NULL);   /* sets up SPI3 and resets the module */

  /* First command after reset: retry once in case the module is still booting */
  for (int attempt = 0; attempt < 2 && !ok; attempt++) {
    ok = (getBlueNRGVersion(&hwVersion, &fwVersion) == BLE_STATUS_SUCCESS);
    if (!ok) HAL_Delay(20);
  }
  if (!ok) { ble_shutdown(); return 1; }

  if (aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, BDADDR_LEN, &len_out, bdaddr) == BLE_STATUS_SUCCESS
      && len_out == BDADDR_LEN) {
    bdaddr_valid = 1;
  }

  if (aci_gatt_init() != BLE_STATUS_SUCCESS) { ble_shutdown(); return 2; }

  if (aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, sizeof(BLE_DEVICE_NAME) - 1,
                           &gapService, &devNameChar, &appearanceChar) != BLE_STATUS_SUCCESS) {
    ble_shutdown(); return 3;
  }

  if (aci_gatt_update_char_value(gapService, devNameChar, 0, sizeof(BLE_DEVICE_NAME) - 1,
                                 (const uint8_t *)BLE_DEVICE_NAME) != BLE_STATUS_SUCCESS) {
    ble_shutdown(); return 4;
  }

  /* 1 service record + 3 records (declaration, value, notify config) per characteristic */
  make_uuid(uuid, UUID_ID_SERVICE);
  if (aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 1 + 3 * 3, &serviceHandle) != BLE_STATUS_SUCCESS) {
    ble_shutdown(); return 5;
  }
  if (add_char(UUID_ID_SENSORS, SENSORS_LEN, &sensorsCharHandle) != BLE_STATUS_SUCCESS) { ble_shutdown(); return 6; }
  if (add_char(UUID_ID_STATUS,  STATUS_LEN,  &statusCharHandle)  != BLE_STATUS_SUCCESS) { ble_shutdown(); return 7; }
  if (add_char(UUID_ID_LASER,   LASER_LEN,   &laserCharHandle)   != BLE_STATUS_SUCCESS) { ble_shutdown(); return 8; }

  aci_hal_set_tx_power_level(1, 4);

  ble_ready = 1;
  set_connectable = 1;
  adv_retry_at = HAL_GetTick();
  BLE_Process();   /* start advertising now */
  return 0;
}

/* Sends a new characteristic value. Other failures (e.g. a full TX buffer) just
   drop this update. If the module stops answering, BLE is switched off so that
   the 1 s HCI timeouts can't slow down the drone's main loop. */
static void update_char(uint16_t charHandle, const uint8_t *buf, uint8_t len)
{
  static uint8_t timeouts_in_a_row = 0;

  if (aci_gatt_update_char_value(serviceHandle, charHandle, 0, len, buf) != BLE_STATUS_TIMEOUT) {
    timeouts_in_a_row = 0;
    return;
  }
  if (++timeouts_in_a_row >= MAX_TIMEOUTS) {
    ble_shutdown();
    printf("Bluetooth module not responding, BLE switched off\r\n");
  }
}

void BLE_Process(void)
{
  if (!ble_ready) {
    return;
  }

  poll_events();

  if (set_connectable && (int32_t)(HAL_GetTick() - adv_retry_at) >= 0) {
    if (start_advertising() == BLE_STATUS_SUCCESS) {
      set_connectable = 0;
    } else {
      adv_retry_at = HAL_GetTick() + ADV_RETRY_MS;
    }
  }
}

void BLE_Shutdown(void)
{
  uint32_t start;

  if (!ble_ready) {
    return;
  }

  if (connected) {
    /* Let the last updates go out, then hang up so the laptop sees the
       disconnect straight away instead of waiting for the link to time out */
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < FLUSH_MS) {
      poll_events();
    }
    if (connected && aci_gap_terminate(conn_handle, ERR_RMT_USR_TERM_CONN) == BLE_STATUS_SUCCESS) {
      start = HAL_GetTick();
      while (connected && (HAL_GetTick() - start) < DISCONNECT_WAIT_MS) {
        poll_events();
      }
    }
  }

  ble_shutdown();   /* also stops advertising */
}

void BLE_IrqHandler(void)
{
  hci_tl_lowlevel_isr();
}

int BLE_IsConnected(void)
{
  return connected;
}

void BLE_GetAddress(char *out)
{
  if (!bdaddr_valid) {
    strcpy(out, "unknown");
    return;
  }
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]);
}

void BLE_UpdateSensors(float tempC, float pressureHpa, float humidityRh,
                       float accelG, float gyroRadS, float magGauss,
                       uint8_t outOfRangeMask)
{
  uint8_t buf[SENSORS_LEN];
  if (!ble_ready) return;

  put_le16(&buf[0],  (uint16_t)scale_i16(tempC, 100.0f));
  put_le16(&buf[2],  scale_u16(pressureHpa, 10.0f));
  put_le16(&buf[4],  scale_u16(humidityRh, 100.0f));
  put_le16(&buf[6],  scale_u16(accelG, 1000.0f));
  put_le16(&buf[8],  scale_u16(gyroRadS, 1000.0f));
  put_le16(&buf[10], scale_u16(magGauss, 1000.0f));
  buf[12] = outOfRangeMask;

  update_char(sensorsCharHandle, buf, sizeof(buf));
}

void BLE_UpdateStatus(uint8_t mode, float rollDeg, float pitchDeg, float yawDeg)
{
  uint8_t buf[STATUS_LEN];
  if (!ble_ready) return;

  buf[0] = mode;
  put_le16(&buf[1], (uint16_t)scale_i16(rollDeg, 100.0f));
  put_le16(&buf[3], (uint16_t)scale_i16(pitchDeg, 100.0f));
  put_le16(&buf[5], (uint16_t)scale_i16(yawDeg, 100.0f));

  update_char(statusCharHandle, buf, sizeof(buf));
}

void BLE_UpdateLaser(uint8_t charge, uint8_t maxCharge, uint8_t cost, uint8_t chargePerPress,
                     uint8_t event)
{
  uint8_t buf[LASER_LEN] = {charge, maxCharge, cost, chargePerPress, event};
  if (!ble_ready) return;

  update_char(laserCharHandle, buf, sizeof(buf));
}
