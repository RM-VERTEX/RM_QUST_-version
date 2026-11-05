/**
  ******************************************************************************
  * @file    usbd_desc.h
  * @brief   USB Device descriptors for the Qust_version virtual COM port.
  ******************************************************************************
  */

#ifndef __USBD_DESC_H
#define __USBD_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/* Exported constants --------------------------------------------------------*/
/* The upper computer (sp_vision) opens this port through the udev symlink
   /dev/gimbal, so the USB identity only has to be stable, not specific. */
#define USBD_VID 0x0483U /* STMicroelectronics */
#define USBD_PID 0x5740U /* Virtual COM Port */

#define USBD_LANGID_STRING 0x0409U /* en-US */
#define USBD_MANUFACTURER_STRING "QustVision"
#define USBD_PRODUCT_STRING_FS "Qust Gimbal CDC"
#define USBD_CONFIGURATION_STRING_FS "CDC Config"
#define USBD_INTERFACE_STRING_FS "CDC Interface"

#define USB_SIZ_STRING_SERIAL 0x1AU

/* Exported types ------------------------------------------------------------*/
extern USBD_DescriptorsTypeDef FS_Desc;

/* Exported functions --------------------------------------------------------*/
uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC_H */
