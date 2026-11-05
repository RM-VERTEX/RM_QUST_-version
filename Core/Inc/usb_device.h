/**
  ******************************************************************************
  * @file    usb_device.h
  * @brief   USB Device initialisation for the Qust_version virtual COM port.
  ******************************************************************************
  */

#ifndef __USB_DEVICE_H
#define __USB_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usbd_def.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Exported functions --------------------------------------------------------*/
void MX_USB_DEVICE_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE_H */
