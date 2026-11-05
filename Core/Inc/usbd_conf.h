/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @brief   USB Device low level driver configuration for Qust_version.
  *
  *          This is a hand-maintained replacement for the CubeMX template, so
  *          that `MX_USB_DEVICE_Init()` (and therefore the class/interface
  *          files CubeMX generates alongside it) is drop-in compatible if the
  *          project is ever regenerated from Qust_version.ioc.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Device identifier passed to USBD_Init(). */
#ifndef DEVICE_FS
#define DEVICE_FS 0U
#endif
#ifndef DEVICE_HS
#define DEVICE_HS 1U
#endif

/* Common Config -------------------------------------------------------------*/
#define USBD_MAX_NUM_INTERFACES 1
#define USBD_MAX_NUM_CONFIGURATION 1

/* The `SP` telemetry frame (io::GimbalToVision) is 43 bytes and the command
   frame (io::VisionToGimbal) is 29 bytes, so the default 256 byte descriptor
   scratch space is plenty. */
#define USBD_MAX_STR_DESC_SIZ 0x100

#define USBD_SELF_POWERED 1
#define USBD_DEBUG_LEVEL 0

/* This board is self powered (24V robot supply), so the CDC class must not
   advertise the 100 mA bus-powered budget. */
#define USBD_CDC_MAX_CURRENT 0x32  /* 100 mA, only used when !USBD_SELF_POWERED */

/* Exported macro ------------------------------------------------------------*/
/* Memory management.
   USBD_malloc is only used by USBD_Init() for the configuration descriptor
   cache, so the standard library heap (`_Min_Heap_Size` in the linker script)
   is sufficient and much cheaper than pulling in a custom pool. */
#define USBD_malloc malloc
#define USBD_free free
#define USBD_memset memset
#define USBD_memcpy memcpy

/* Must not block: several USBD_LL_* entry points are called from the OTG_FS
   interrupt, and this project runs FreeRTOS (osDelay would trap there). */
#define USBD_Delay HAL_Delay

/* DEBUG macros --------------------------------------------------------------*/
#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...) \
  do {                   \
    printf(__VA_ARGS__); \
    printf("\n");        \
  } while (0)
#else
#define USBD_UsrLog(...) \
  do {                   \
  } while (0)
#endif

#if (USBD_DEBUG_LEVEL > 1)
#define USBD_ErrLog(...)      \
  do {                        \
    printf("ERROR: ");        \
    printf(__VA_ARGS__);      \
    printf("\n");             \
  } while (0)
#else
#define USBD_ErrLog(...) \
  do {                   \
  } while (0)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)      \
  do {                        \
    printf("DEBUG : ");       \
    printf(__VA_ARGS__);      \
    printf("\n");             \
  } while (0)
#else
#define USBD_DbgLog(...) \
  do {                   \
  } while (0)
#endif

/* Exported functions --------------------------------------------------------*/

/* The PCD handle lives in usbd_conf.c but its interrupt handler is declared in
   Core/Src/stm32f4xx_it.c, which is C. */
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
