/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @brief   USB Device descriptors for the Qust_version virtual COM port.
  *
  *          Layout follows the STM32CubeF4 V1.28.3 CDC_Standalone example; the
  *          identity strings and the serial number source were changed so the
  *          port is recognisable and the serial is stable per MCU.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"

#include "usbd_conf.h"
#include "usbd_core.h"

/* Private define ------------------------------------------------------------*/
#define USBD_DEVICE_DESC_SIZE 0x12U
#define USBD_LANGID_DESC_SIZE 0x04U

/* Sixteen-bit unicode string descriptor header plus 2 bytes per character */
#define USBD_STR_DESC_LEN(_str) (uint8_t)(2U + (2U * (sizeof(_str) - 1U)))

/* Private variables ---------------------------------------------------------*/
USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor,
};

/* USB Standard Device Descriptor.
   bDeviceClass = 0x02 (CDC) / bDeviceSubClass = 0x00, i.e. class information
   is carried by the interface descriptor, which is what the Linux cdc_acm
   driver expects when it binds to /dev/ttyACM0. */
__ALIGN_BEGIN static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  USBD_DEVICE_DESC_SIZE,      /* bLength */
  USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
  0x00,                       /* bcdUSB = 2.00 */
  0x02,
  0x02,                       /* bDeviceClass: CDC */
  0x00,                       /* bDeviceSubClass */
  0x00,                       /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
  LOBYTE(USBD_VID),           /* idVendor */
  HIBYTE(USBD_VID),
  LOBYTE(USBD_PID),           /* idProduct */
  HIBYTE(USBD_PID),
  0x00,                       /* bcdDevice = 2.00 */
  0x02,
  USBD_IDX_MFC_STR,           /* iManufacturer */
  USBD_IDX_PRODUCT_STR,       /* iProduct */
  USBD_IDX_SERIAL_STR,        /* iSerialNumber */
  USBD_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
};

/* USB Standard language ID descriptor (en-US) */
__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USBD_LANGID_DESC_SIZE] __ALIGN_END = {
  USBD_LANGID_DESC_SIZE,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING),
};

/* USB serial number descriptor, derived from the MCU 96-bit unique ID */
__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

/* UTF-16LE encoded identity strings. USBD_GetString() writes straight into
   these buffers and reports the length back, so no shared scratch buffer is
   needed. */
__ALIGN_BEGIN static uint8_t USBD_StringMfc[USBD_STR_DESC_LEN(USBD_MANUFACTURER_STRING)] __ALIGN_END;
__ALIGN_BEGIN static uint8_t USBD_StringProduct[USBD_STR_DESC_LEN(USBD_PRODUCT_STRING_FS)] __ALIGN_END;
__ALIGN_BEGIN static uint8_t USBD_StringConfig[USBD_STR_DESC_LEN(USBD_CONFIGURATION_STRING_FS)]
  __ALIGN_END;
__ALIGN_BEGIN static uint8_t USBD_StringInterface[USBD_STR_DESC_LEN(USBD_INTERFACE_STRING_FS)]
  __ALIGN_END;

/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len);
static void Get_SerialNum(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Convert an integer to a unicode string.
  * @param  value: The integer to be converted
  * @param  pbuf: The destination buffer
  * @param  len: Number of unicode characters to produce
  * @retval None
  */
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++) {
    if (((value >> 28)) < 0xA) {
      pbuf[2 * idx] = (uint8_t)((value >> 28) + '0');
    } else {
      pbuf[2 * idx] = (uint8_t)((value >> 28) + 'A' - 10);
    }

    value = value << 4;

    /* The last two characters of the serial are always zero */
    pbuf[2 * idx + 1] = 0;
  }
}

/**
  * @brief  Create the serial number string descriptor from the MCU UID.
  * @retval None
  */
static void Get_SerialNum(void)
{
  /* UID_BASE (0x1FFF7A10) is the STM32F4 96-bit unique device ID: the three
     words are the wafer X/Y coordinates and the lot number. The F4 CMSIS
     headers only expose the base address, not DEVICE_ID1..3. */
  uint32_t deviceserial0 = *(uint32_t *)(UID_BASE + 0x00U);
  uint32_t deviceserial1 = *(uint32_t *)(UID_BASE + 0x04U);
  uint32_t deviceserial2 = *(uint32_t *)(UID_BASE + 0x08U);

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0) {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Return the device descriptor.
  */
uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

/**
  * @brief  Return the LangID string descriptor.
  */
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
  * @brief  Return the manufacturer string descriptor.
  */
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StringMfc, length);
  return USBD_StringMfc;
}

/**
  * @brief  Return the product string descriptor.
  */
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StringProduct, length);
  return USBD_StringProduct;
}

/**
  * @brief  Return the serial number string descriptor.
  */
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;

  /* Re-read the UID every time: it is cheap and keeps the descriptor valid
     without relying on an init call ordering. */
  Get_SerialNum();

  return USBD_StringSerial;
}

/**
  * @brief  Return the configuration string descriptor.
  */
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StringConfig, length);
  return USBD_StringConfig;
}

/**
  * @brief  Return the interface string descriptor.
  */
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StringInterface, length);
  return USBD_StringInterface;
}
