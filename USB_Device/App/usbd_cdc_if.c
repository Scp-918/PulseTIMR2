/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v3.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "usart.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// 注意：APP_TX_DATA_SIZE 在 usbd_cdc_if.h 中定义，通常为 2048
volatile uint32_t UserTxBufPtrIn = 0;  // 写入指针 (Head)
volatile uint32_t UserTxBufPtrOut = 0; // 读出/发送指针 (Tail)
volatile uint8_t  UserTxBufBusy = 0;   // 发送忙标志

uint8_t  USB_Rx_Flag = 0;
uint8_t  USB_Rx_Buffer[256];
uint32_t USB_Rx_Len = 0;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* Default line coding for virtual COM port: bitrate 961200, 1 stop bit, no parity, 8 data bits */
USBD_CDC_LineCodingTypeDef linecoding =
{
  921600, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
uint8_t CDC_Transmit_FS2(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  uint32_t primask;
  /* USER CODE BEGIN 7 */
  primask = __get_PRIMASK();
  __disable_irq();

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  
  // 1. 检查参数
  if (hcdc == NULL || (hcdc->TxState != 0 && UserTxBufBusy == 0)) {
      result = USBD_BUSY;
      goto exit;
  }

  // 2. 将用户数据填入环形缓冲区
  for (int i = 0; i < Len; i++) {
      UserTxBufferFS[UserTxBufPtrIn] = Buf[i];
      UserTxBufPtrIn++;
      // 处理环形回绕
      if (UserTxBufPtrIn == APP_TX_DATA_SIZE) {
          UserTxBufPtrIn = 0;
      }
      
      // 检查缓冲区溢出（可选：如果追上输出指针，则丢弃或返回错误）
      if (UserTxBufPtrIn == UserTxBufPtrOut) {
          result = USBD_FAIL; // 缓冲区满
          goto exit;
      }
  }

  // 3. 如果当前 USB 空闲，立即启动发送
  if (UserTxBufBusy == 0) {
      uint32_t size_to_send = 0;
      
      // 计算本次需要发送的长度（处理环形缓冲区的回绕问题）
      if (UserTxBufPtrIn > UserTxBufPtrOut) {
          size_to_send = UserTxBufPtrIn - UserTxBufPtrOut;
      } else if (UserTxBufPtrIn < UserTxBufPtrOut) {
          // 如果回绕，先发送从 Out 到 缓冲区末尾 的部分
          size_to_send = APP_TX_DATA_SIZE - UserTxBufPtrOut;
      } else {
          // 指针相等，无数据
          goto exit;
      }

      // 设置 USB 发送缓冲区指针指向环形缓冲区的当前读取位置
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &UserTxBufferFS[UserTxBufPtrOut], size_to_send);
      
      // 标记为忙，并启动发送
      UserTxBufBusy = 1;
      result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  }
exit:
  __set_PRIMASK(primask);
  /* USER CODE END 7 */
  return result;
}
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
      linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | \
                                         (pbuf[2] << 16) | (pbuf[3] << 24));
      linecoding.format     = pbuf[4];
      linecoding.paritytype = pbuf[5];
      linecoding.datatype   = pbuf[6];

      /* If needed, apply new settings to UART hardware here */
    break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(linecoding.bitrate);
      pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
      pbuf[4] = linecoding.format;
      pbuf[5] = linecoding.paritytype;
      pbuf[6] = linecoding.datatype;
    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  // USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  // USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  // return (USBD_OK);
  // 1. 【新增代码】将接收到的数据回传给上位机
  // 利用您已经实现的 CDC_Transmit_FS2 函数（环形缓冲区版本）发送数据
  // Buf 包含了接收到的数据，*Len 是接收到的数据长度
  // // 1. 将数据拷贝到临时缓冲区，供主循环处理
  // USB_Rx_Len = (*Len < 256) ? *Len : 256;
  // memcpy(USB_Rx_Buffer, Buf, USB_Rx_Len);
  // USB_Rx_Flag = 1; // 触发标志位
  CDC_Transmit_FS2(Buf, *Len);
  HAL_UART_Transmit(&huart1, Buf, *Len, 100);

  // 2. 【原有代码】准备下一次接收
  // 这一步非常重要，必须告知 USB 驱动重新准备好接收缓冲区，否则将无法接收后续数据
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  
  // 1. 检查参数
  if (hcdc == NULL || hcdc->TxState != 0 && UserTxBufBusy == 0) {
      return USBD_BUSY;
  }

  // 2. 将用户数据填入环形缓冲区
  for (int i = 0; i < Len; i++) {
      UserTxBufferFS[UserTxBufPtrIn] = Buf[i];
      UserTxBufPtrIn++;
      // 处理环形回绕
      if (UserTxBufPtrIn == APP_TX_DATA_SIZE) {
          UserTxBufPtrIn = 0;
      }
      
      // 检查缓冲区溢出（可选：如果追上输出指针，则丢弃或返回错误）
      if (UserTxBufPtrIn == UserTxBufPtrOut) {
          return USBD_FAIL; // 缓冲区满
      }
  }

  // 3. 如果当前 USB 空闲，立即启动发送
  if (UserTxBufBusy == 0) {
      uint32_t size_to_send = 0;
      
      // 计算本次需要发送的长度（处理环形缓冲区的回绕问题）
      if (UserTxBufPtrIn > UserTxBufPtrOut) {
          size_to_send = UserTxBufPtrIn - UserTxBufPtrOut;
      } else if (UserTxBufPtrIn < UserTxBufPtrOut) {
          // 如果回绕，先发送从 Out 到 缓冲区末尾 的部分
          size_to_send = APP_TX_DATA_SIZE - UserTxBufPtrOut;
      } else {
          // 指针相等，无数据
          return USBD_OK;
      }

      // 设置 USB 发送缓冲区指针指向环形缓冲区的当前读取位置
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &UserTxBufferFS[UserTxBufPtrOut], size_to_send);
      
      // 标记为忙，并启动发送
      UserTxBufBusy = 1;
      result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  }
  // 2. 退出临界区：恢复中断
  // __enable_irq();
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  uint32_t primask;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  primask = __get_PRIMASK();
  __disable_irq();

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc == NULL)
  {
    result = USBD_FAIL;
    goto exit;
  }

  // 1. 更新读出指针 (UserTxBufPtrOut)
  // 注意：此时 hcdc->TxLength 存储了上一次发送的长度
  UserTxBufPtrOut += hcdc->TxLength;
  if (UserTxBufPtrOut >= APP_TX_DATA_SIZE) {
      UserTxBufPtrOut -= APP_TX_DATA_SIZE;
  }

  // 2. 检查是否还有剩余数据需要发送
  if (UserTxBufPtrIn != UserTxBufPtrOut) {
      uint32_t size_to_send = 0;

      if (UserTxBufPtrIn > UserTxBufPtrOut) {
          size_to_send = UserTxBufPtrIn - UserTxBufPtrOut;
      } else {
          // 遇到回绕，先发尾部
          size_to_send = APP_TX_DATA_SIZE - UserTxBufPtrOut;
      }

      // 再次设置发送缓冲区并启动下一次传输
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &UserTxBufferFS[UserTxBufPtrOut], size_to_send);
      result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
      
      // 保持忙状态 UserTxBufBusy = 1;
  } else {
      // 数据全部发完，释放忙标志
      UserTxBufBusy = 0;
  }
exit:
  __set_PRIMASK(primask);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
