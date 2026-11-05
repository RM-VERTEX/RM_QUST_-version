/**
  ******************************************************************************
  * @file    usbd_cdc_if.h
  * @brief   CDC interface for the Qust_version virtual COM port.
  *
  *          The receive path is an ISR-safe ring buffer so the OTG_FS interrupt
  *          never blocks; the transmit path is explicitly non-blocking and
  *          reports back-pressure through `CDC_Transmit_Ready()`.
  ******************************************************************************
  */

#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "usbd_cdc.h"

/* Exported constants --------------------------------------------------------*/
/* Power of two so the index wrap is a mask. Must fit several 64 byte USB
   packets, since one interrupt can conceivably land more than one packet. */
#define CDC_RX_RING_SIZE 512U
#define CDC_RX_RING_MASK (CDC_RX_RING_SIZE - 1U)

extern USBD_CDC_ItfTypeDef USBD_CDC_fops;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Queue a frame for transmission on the CDC IN endpoint.
  * @param  buf: frame to send; must stay valid until the transfer completes
  * @param  len: frame length in bytes
  * @retval USBD_OK      transfer started (or the frame is copied internally)
  * @retval USBD_BUSY    the previous transfer has not completed yet
  * @retval USBD_FAIL    bad arguments
  * @note   Safe to call from a task context only.
  */
uint8_t CDC_Transmit_FS(uint8_t * buf, uint16_t len);

/**
  * @brief  Non-blocking flush of the internal transmit buffer.
  * @note   Present for CubeMX signature compatibility; this implementation
  *         never buffers a frame of its own, so it starts any frame that was
  *         previously rejected with USBD_BUSY if the caller re-armed it.
  * @retval USBD_OK / USBD_BUSY / USBD_FAIL
  */
uint8_t CDC_Transmit_Flush(void);

/**
  * @brief  Whether a new frame may be queued right now.
  * @retval 1 if free, 0 if the previous transfer is still in flight
  */
uint8_t CDC_Transmit_Ready(void);

/**
  * @brief  Whether the host has configured the device (port is open).
  * @retval 1 if configured, 0 otherwise
  */
uint8_t CDC_IsConnected(void);

/**
  * @brief  Number of received bytes waiting to be read.
  * @retval byte count
  */
uint16_t CDC_RxAvailable(void);

/**
  * @brief  Read received bytes out of the ISR ring buffer.
  * @param  dst: destination buffer
  * @param  max_len: maximum number of bytes to copy
  * @retval number of bytes actually copied (0 if empty)
  * @note   Safe to call from a task context only.
  */
uint16_t CDC_RxRead(uint8_t * dst, uint16_t max_len);

/**
  * @brief  Discard everything in the receive ring buffer.
  * @retval None
  */
void CDC_RxFlush(void);

/**
  * @brief  Signal that the host closed the port, invalidating the RX stream.
  * @note   Called from the control-request handler when the line coding is
  *         dropped; the vision task uses this to reset its byte-stream parser.
  * @retval None
  */
void CDC_NotifyDisconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */
