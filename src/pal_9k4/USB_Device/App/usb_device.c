/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usb_device.c
 * @version        : v1.0_Cube
 * @brief          : This file implements the USB Device
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */
#include "usbd_mtp.h"
#include "usbd_mtp_if.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void Error_Handler(void);
/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceHS;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
static uint8_t cdc_ep_addr[3] = {CDC_IN_EP, CDC_OUT_EP, CDC_CMD_EP};
static uint8_t mtp_ep_addr[3] = {MTP_IN_EP, MTP_OUT_EP, MTP_CMD_EP};
/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS) != USBD_OK)
  {
    Error_Handler();
  }

  if (USBD_RegisterClassComposite(&hUsbDeviceHS, &USBD_CDC, CLASS_TYPE_CDC,
                                  cdc_ep_addr) != USBD_OK) {
      Error_Handler();
  }

  if (USBD_RegisterClassComposite(&hUsbDeviceHS, &USBD_MTP, CLASS_TYPE_MTP,
                                  mtp_ep_addr) != USBD_OK) {
      Error_Handler();
  }

  if (USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_HS) !=
      USBD_OK) {
      Error_Handler();
  }

  if (USBD_MTP_RegisterInterface(&hUsbDeviceHS, &USBD_MTP_fops) != USBD_OK) {
      Error_Handler();
  }

  if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

