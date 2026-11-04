/**
  ******************************************************************************
  * @file    hci_tl_interface.h
  * @brief   SPI3 transport between the STM32 and the on-board SPBTLE-RF
  *          (BlueNRG-MS) Bluetooth LE module on the B-L4S5I-IOT01A.
  *          Adapted from ST X-CUBE-BLE1 (SensorDemo_BLESensor-App).
  ******************************************************************************
  */
#ifndef HCI_TL_INTERFACE_H
#define HCI_TL_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* pin names for the SPBTLE-RF and SPI3 */

/* BlueNRG IRQ line: the module drives it high when it has data for us */
#define HCI_TL_SPI_EXTI_PORT  SPBTLE_RF_IRQ_EXTI6_GPIO_Port
#define HCI_TL_SPI_EXTI_PIN   SPBTLE_RF_IRQ_EXTI6_Pin
#define HCI_TL_SPI_EXTI_IRQn  SPBTLE_RF_IRQ_EXTI6_EXTI_IRQn

#define HCI_TL_SPI_CS_PORT    SPBTLE_RF_SPI3_CSN_GPIO_Port
#define HCI_TL_SPI_CS_PIN     SPBTLE_RF_SPI3_CSN_Pin

#define HCI_TL_RST_PORT       SPBTLE_RF_RST_GPIO_Port
#define HCI_TL_RST_PIN        SPBTLE_RF_RST_Pin

int32_t HCI_TL_SPI_Init    (void* pConf);
int32_t HCI_TL_SPI_DeInit  (void);
int32_t HCI_TL_SPI_Receive (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Send    (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Reset   (void);

/* Registers the SPI functions with the HCI layer and enables the IRQ line */
void hci_tl_lowlevel_init(void);

/* Call from the EXTI callback when the BlueNRG IRQ pin goes high */
void hci_tl_lowlevel_isr(void);

#ifdef __cplusplus
}
#endif
#endif /* HCI_TL_INTERFACE_H */
