/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v3.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
                0x06, 0x00, 0xff,// USAGE_PAGE (Vendor Defined Page 1)  <---- (1)

                0x09, 0x01,// USAGE (Vendor Usage 1)              <---- (1)

                0xa1, 0x01,// COLLECTION (Application)            <---- (2)

                // ------ common globals ------

                0x15, 0x00,//   LOGICAL_MINIMUM (0)               <---- (3)

                0x26, 0xff, 0x00,//   LOGICAL_MAXIMUM (255)             <---- (3)

                0x75, 0x08,//   REPORT_SIZE (8)   - bits          <---- (4)

                // ------ report 1 ----------

                0x85, 0x01,

                // ------ input report ------

                0x95, 0x03,//   REPORT_COUNT      - bytes         <---- (5)

                0x09, 0x01,//   USAGE (Vendor Usage 1)            <---- (6)

                0x81, 0x02,//   INPUT (Data,Var,Abs)              <---- (7)

                // ------ output report ------

                0x95, 0x03,//   REPORT_COUNT      - bytes         <---- (5)

                0x09, 0x01,//   USAGE (Vendor Usage 1)            <---- (6)

                0x91, 0x02,//   OUTPUT (Data,Var,Abs)             <---- (7)

                // ------ report 2 ----------

                0x85, 0x02,

                0x95, 0x03,//   REPORT_COUNT      - bytes         <---- (5)

                0x09, 0x01,//   USAGE (Vendor Usage 1)            <---- (6)

                0x81, 0x02,//   INPUT (Data,Var,Abs)              <---- (7)

                // ------ output report ------

                0x95, 0x03,//   REPORT_COUNT      - bytes         <---- (5)

                0x09, 0x01,//   USAGE (Vendor Usage 1)            <---- (6)

                0x91, 0x02,//   OUTPUT (Data,Var,Abs)             <---- (7)

  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
    return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
    return (USBD_OK);
  /* USER CODE END 5 */
}


extern uint16_t DT;
extern int8_t motorSpeeds[3];
extern int8_t motor_new;

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
    UNUSED(event_idx);
    UNUSED(state);

    USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef *) hUsbDeviceFS.pClassData;
    //for(unsigned char i=0;i<USBD_CUSTOMHID_OUTREPORT_BUF_SIZE;i++)
    //    USB_RX_Buffer[i] = hhid->Report_buf[i];

    if (hhid->Report_buf[0] == 0x01) {
        DT = hhid->Report_buf[1] << 8 | hhid->Report_buf[2];
    }else if (hhid->Report_buf[0] == 0x02) {
        for(int i = 0; i < 3; i++)
            motorSpeeds[i] = hhid->Report_buf[i+1];
        motor_new = 1;
    }
    /*update the TX buffer with the corect response*/
    //ProcessMultipleRequest();

    /*send back the answers for the requests*/
    //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, hhid->Report_buf, 4);
    /* Start next USB packet transfer once data processing is completed */
    USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS);

    return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */

static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len) {
    return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}

/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

