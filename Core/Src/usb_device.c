/**
  ******************************************************************************
  * @file    usb_device.c
  * @brief   USB Device initialisation for the Qust_version virtual COM port.
  *
  *          Provides `MX_USB_DEVICE_Init()`, which is called from `main()` and
  *          defines the global USB device handle that `usbd_cdc_if.c` uses to
  *          drive the CDC class.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_conf.h"
#include "usb_device.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef hUsbDeviceFS;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialise the USB device stack as a CDC (virtual COM port).
  * @retval None
  * @note   Must be called after the clocks are configured and before
  *         `osKernelStart()`, so the OTG_FS interrupt is live from boot.
  */
void MX_USB_DEVICE_Init(void)
{
  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_fops) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
    Error_Handler();
  }
}
