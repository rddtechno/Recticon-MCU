/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DO_RLY_VIN_Pin GPIO_PIN_13
#define DO_RLY_VIN_GPIO_Port GPIOC
#define DO_SYS_RUN_Pin GPIO_PIN_14
#define DO_SYS_RUN_GPIO_Port GPIOC
#define DO_SYS_ALM_Pin GPIO_PIN_15
#define DO_SYS_ALM_GPIO_Port GPIOC
#define DO_BMS_EN_Pin GPIO_PIN_0
#define DO_BMS_EN_GPIO_Port GPIOH
#define DO_BMS1_Pin GPIO_PIN_1
#define DO_BMS1_GPIO_Port GPIOH
#define AI_NTC_TEMP_Pin GPIO_PIN_0
#define AI_NTC_TEMP_GPIO_Port GPIOC
#define AI_BTS_CUR_Pin GPIO_PIN_1
#define AI_BTS_CUR_GPIO_Port GPIOC
#define AI_VSAT_CUR_Pin GPIO_PIN_2
#define AI_VSAT_CUR_GPIO_Port GPIOC
#define DO_BMS2_Pin GPIO_PIN_3
#define DO_BMS2_GPIO_Port GPIOC
#define DO_BMS3_Pin GPIO_PIN_0
#define DO_BMS3_GPIO_Port GPIOA
#define DO_USART2_MUX_Pin GPIO_PIN_1
#define DO_USART2_MUX_GPIO_Port GPIOA
#define DO_RS485_1_DE_Pin GPIO_PIN_4
#define DO_RS485_1_DE_GPIO_Port GPIOA
#define DO_LCD_A0_Pin GPIO_PIN_4
#define DO_LCD_A0_GPIO_Port GPIOC
#define DO_LCD_RST_Pin GPIO_PIN_5
#define DO_LCD_RST_GPIO_Port GPIOC
#define DI_BTN_OK_PM1_ZX_Pin GPIO_PIN_0
#define DI_BTN_OK_PM1_ZX_GPIO_Port GPIOB
#define DI_BTN_BCK_Pin GPIO_PIN_1
#define DI_BTN_BCK_GPIO_Port GPIOB
#define DI_BTN_UP_PM2_ZX_Pin GPIO_PIN_10
#define DI_BTN_UP_PM2_ZX_GPIO_Port GPIOB
#define DI_BTN_DN_Pin GPIO_PIN_12
#define DI_BTN_DN_GPIO_Port GPIOB
#define DO_LCD_CS_Pin GPIO_PIN_13
#define DO_LCD_CS_GPIO_Port GPIOB
#define DO_RS232_MUXA_Pin GPIO_PIN_14
#define DO_RS232_MUXA_GPIO_Port GPIOB
#define DO_RS232_MUXB_DI_OPI_TICK_Pin GPIO_PIN_15
#define DO_RS232_MUXB_DI_OPI_TICK_GPIO_Port GPIOB
#define DO_RS485_2_DE_Pin GPIO_PIN_8
#define DO_RS485_2_DE_GPIO_Port GPIOC
#define DO_ADC_CS_Pin GPIO_PIN_9
#define DO_ADC_CS_GPIO_Port GPIOC
#define DI_ADC_DRDY_Pin GPIO_PIN_8
#define DI_ADC_DRDY_GPIO_Port GPIOA
#define DI_ADC_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define DO_OPI_PWR_Pin GPIO_PIN_11
#define DO_OPI_PWR_GPIO_Port GPIOA
#define DO_IOEXP_DO_RST_Pin GPIO_PIN_12
#define DO_IOEXP_DO_RST_GPIO_Port GPIOA
#define DO_IOEXP_DO_CS_Pin GPIO_PIN_15
#define DO_IOEXP_DO_CS_GPIO_Port GPIOA
#define DI_GST_RUN_SPI3_SCK_Pin GPIO_PIN_10
#define DI_GST_RUN_SPI3_SCK_GPIO_Port GPIOC
#define DI_GST_ALM_SPI3_MISO_Pin GPIO_PIN_11
#define DI_GST_ALM_SPI3_MISO_GPIO_Port GPIOC
#define DO_GST_EN_SPI3_MOSI_Pin GPIO_PIN_12
#define DO_GST_EN_SPI3_MOSI_GPIO_Port GPIOC
#define DO_IOEXP_DI_CS_Pin GPIO_PIN_2
#define DO_IOEXP_DI_CS_GPIO_Port GPIOD
#define DO_IOEXP_DI_RST_Pin GPIO_PIN_3
#define DO_IOEXP_DI_RST_GPIO_Port GPIOB
#define DI_MCB_BTS_Pin GPIO_PIN_4
#define DI_MCB_BTS_GPIO_Port GPIOB
#define DI_MCB_VSAT_Pin GPIO_PIN_5
#define DI_MCB_VSAT_GPIO_Port GPIOB
#define DO_EEPROM_WP_Pin GPIO_PIN_8
#define DO_EEPROM_WP_GPIO_Port GPIOB
#define DO_ADC_RST_Pin GPIO_PIN_9
#define DO_ADC_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
