/**
  ******************************************************************************
  * @file    usbd_cdc_if.c
  * @brief   CDC interface for the Qust_version virtual COM port.
  *
  *          Differences from the ST template:
  *            - no UART bridging or TIM/DMA plumbing, this is a pure USB pipe
  *              between the upper computer (sp_vision) and the application;
  *            - the receive path feeds an ISR-safe ring buffer instead of
  *              touching application state from the interrupt;
  *            - the transmit path is strictly non-blocking and reports
  *              back-pressure, so it is safe to call from a FreeRTOS task.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

#include <stddef.h>
#include <string.h>

#include "usbd_conf.h"
#include "usbd_core.h"
#include "usb_device.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Receive ring buffer. Written only by the OTG_FS ISR, read only by the task,
   so no critical section is required: each index has a single writer. */
static uint8_t rx_ring_[CDC_RX_RING_SIZE];
static volatile uint16_t rx_head_ = 0; /* ISR writer */
static volatile uint16_t rx_tail_ = 0; /* task reader */

/* Endpoint buffer handed to the USB device core. Must be at least
   CDC_DATA_FS_OUT_PACKET_SIZE (64) bytes. */
__ALIGN_BEGIN static uint8_t cdc_rx_packet_[CDC_DATA_FS_OUT_PACKET_SIZE] __ALIGN_END;

/* Frame currently owned by the USB device core (i.e. being transmitted). The
   caller's buffer stays referenced until TransmitCplt fires, so the caller
   must not reuse it before CDC_Transmit_Ready() reports free again. */
static uint8_t * tx_frame_ = NULL;
static volatile uint16_t tx_frame_len_ = 0;

/* Guard so a USBD_BUSY must be resolved before another frame is accepted. */
static volatile uint8_t tx_busy_ = 0;

/* Set once the host has issued SET_CONFIGURATION. */
static volatile uint8_t cdc_configured_ = 0;

/* Bumped whenever the host drops the port, so the parser can resynchronise. */
static volatile uint32_t cdc_disconnect_count_ = 0;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t * pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t * Buf, uint32_t * Len);
static int8_t CDC_Itf_TransmitCplt(uint8_t * Buf, uint32_t * Len, uint8_t epnum);

/* Exported variables --------------------------------------------------------*/
USBD_CDC_ItfTypeDef USBD_CDC_fops = {
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive,
  CDC_Itf_TransmitCplt,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer.
  * @param  None
  * @retval USBD_OK
  */
static int8_t CDC_Itf_Init(void)
{
  rx_head_ = 0;
  rx_tail_ = 0;
  tx_frame_ = NULL;
  tx_frame_len_ = 0;
  tx_busy_ = 0;
  cdc_configured_ = 0;

  /* Hand our OUT endpoint buffer to the class driver. Without this the core
     would receive into a stale pointer from the previous configuration. */
  if (USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_packet_) != USBD_OK) {
    return (USBD_FAIL);
  }

  return (USBD_OK);
}

/**
  * @brief  DeInitializes the CDC media low layer.
  * @param  None
  * @retval USBD_OK
  */
static int8_t CDC_Itf_DeInit(void)
{
  cdc_configured_ = 0;
  tx_busy_ = 0;
  tx_frame_ = NULL;
  cdc_disconnect_count_++;
  return (USBD_OK);
}

/**
  * @brief  Handle the CDC class control requests.
  * @param  cmd: command code
  * @param  pbuf: pointer to the request payload
  * @param  length: payload length
  * @retval USBD_OK to accept, USBD_FAIL to stall
  * @note   Runs in the OTG_FS ISR context; must not block.
  */
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t * pbuf, uint16_t length)
{
  (void)pbuf;

  switch (cmd) {
    case CDC_SET_LINE_CODING:
      /* The host may pick any baud rate (sp_vision uses the default); the
         virtual port ignores it because there is no physical UART behind it. */
      break;

    case CDC_GET_LINE_CODING:
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* bit0 = DTR, bit1 = RTS. DTR dropping is how a Linux host signals that
         it closed the port, which invalidates the byte stream. */
      if (length >= 2U && (pbuf[0] & 0x01U) == 0U) {
        CDC_RxFlush();
        cdc_disconnect_count_++;
      } else if (length >= 2U) {
        cdc_configured_ = 1U;
      }
      break;

    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
      break;

    default:
      return (USBD_FAIL);
  }

  return (USBD_OK);
}

/**
  * @brief  Data received over the CDC OUT endpoint.
  * @param  Buf: received bytes
  * @param  Len: number of bytes received
  * @retval USBD_OK
  * @note   Runs in the OTG_FS ISR context. Only ring indices are touched, and
  *         only the ISR writes them, so this stays lock-free.
  */
static int8_t CDC_Itf_Receive(uint8_t * Buf, uint32_t * Len)
{
  uint32_t len = *Len;
  uint16_t head = rx_head_;

  for (uint32_t i = 0; i < len; i++) {
    uint16_t next = (uint16_t)((head + 1U) & CDC_RX_RING_MASK);

    if (next == rx_tail_) {
      /* Buffer full: drop the whole burst and resynchronise. The frame in
         flight is incomplete either way, and the parser re-syncs on the next
         'S','P' header, so a partial tail would only add latency. */
      break;
    }

    rx_ring_[head] = Buf[i];
    head = next;
  }

  rx_head_ = head;

  return (USBD_OK);
}

/**
  * @brief  Transmit complete callback.
  * @param  Buf: buffer that was sent
  * @param  Len: number of bytes sent
  * @param  epnum: endpoint number
  * @retval USBD_OK
  * @note   Runs in the OTG_FS ISR context.
  */
static int8_t CDC_Itf_TransmitCplt(uint8_t * Buf, uint32_t * Len, uint8_t epnum)
{
  (void)Buf;
  (void)Len;
  (void)epnum;

  tx_frame_len_ = 0;
  tx_frame_ = NULL;
  tx_busy_ = 0;

  return (USBD_OK);
}

/* Exported functions --------------------------------------------------------*/

uint8_t CDC_Transmit_FS(uint8_t * buf, uint16_t len)
{
  USBD_HandleTypeDef * pdev = &hUsbDeviceFS;

  if (buf == NULL || len == 0U) {
    return (USBD_FAIL);
  }

  if (cdc_configured_ == 0U || pdev->dev_state != USBD_STATE_CONFIGURED) {
    return (USBD_FAIL);
  }

  if (tx_busy_ != 0U) {
    return (USBD_BUSY);
  }

  /* Claim the slot before handing the buffer to the core. */
  tx_frame_ = buf;
  tx_frame_len_ = len;
  tx_busy_ = 1U;

  if (USBD_CDC_SetTxBuffer(pdev, buf, len) != USBD_OK) {
    tx_busy_ = 0U;
    tx_frame_ = NULL;
    tx_frame_len_ = 0;
    return (USBD_FAIL);
  }

  if (USBD_CDC_TransmitPacket(pdev) != USBD_OK) {
    tx_busy_ = 0U;
    tx_frame_ = NULL;
    tx_frame_len_ = 0;
    return (USBD_BUSY);
  }

  return (USBD_OK);
}

uint8_t CDC_Transmit_Flush(void) { return (tx_busy_ != 0U) ? USBD_BUSY : USBD_OK; }

uint8_t CDC_Transmit_Ready(void)
{
  USBD_HandleTypeDef * pdev = &hUsbDeviceFS;

  if (cdc_configured_ == 0U || pdev->dev_state != USBD_STATE_CONFIGURED) {
    return 0U;
  }

  return (tx_busy_ == 0U) ? 1U : 0U;
}

uint8_t CDC_IsConnected(void)
{
  USBD_HandleTypeDef * pdev = &hUsbDeviceFS;

  if (cdc_configured_ == 0U) {
    return 0U;
  }

  return (pdev->dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}

uint16_t CDC_RxAvailable(void)
{
  return (uint16_t)((rx_head_ - rx_tail_) & CDC_RX_RING_MASK);
}

uint16_t CDC_RxRead(uint8_t * dst, uint16_t max_len)
{
  uint16_t copied = 0;
  uint16_t tail = rx_tail_;

  if (dst == NULL) {
    return 0U;
  }

  while (copied < max_len && tail != rx_head_) {
    dst[copied++] = rx_ring_[tail];
    tail = (uint16_t)((tail + 1U) & CDC_RX_RING_MASK);
  }

  rx_tail_ = tail;

  return copied;
}

void CDC_RxFlush(void)
{
  rx_tail_ = rx_head_;
}

void CDC_NotifyDisconnect(void) { cdc_disconnect_count_++; }
