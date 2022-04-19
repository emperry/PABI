/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern volatile uint32_t hrtim_count;
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
struct MODULECOMMAND_t{
	uint16_t address;
	uint8_t packageNum;
	uint16_t command;
	uint8_t paramLen;
	uint8_t params[32];
};
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ETH_SCK_Pin GPIO_PIN_2
#define ETH_SCK_GPIO_Port GPIOE
#define PG_Pin GPIO_PIN_3
#define PG_GPIO_Port GPIOE
#define ETH_NSS_Pin GPIO_PIN_4
#define ETH_NSS_GPIO_Port GPIOE
#define ETH_MISO_Pin GPIO_PIN_5
#define ETH_MISO_GPIO_Port GPIOE
#define ETH_MOSI_Pin GPIO_PIN_6
#define ETH_MOSI_GPIO_Port GPIOE
#define PWM_EXPANDER_SDA_Pin GPIO_PIN_0
#define PWM_EXPANDER_SDA_GPIO_Port GPIOF
#define PWM_EXPANDER_SCL_Pin GPIO_PIN_1
#define PWM_EXPANDER_SCL_GPIO_Port GPIOF
#define LCD2_NSS_Pin GPIO_PIN_4
#define LCD2_NSS_GPIO_Port GPIOA
#define LCD1_SCK_Pin GPIO_PIN_5
#define LCD1_SCK_GPIO_Port GPIOA
#define LCD1_MISO_Pin GPIO_PIN_6
#define LCD1_MISO_GPIO_Port GPIOA
#define USER_LED_G_Pin GPIO_PIN_0
#define USER_LED_G_GPIO_Port GPIOB
#define ETH_RSTN_Pin GPIO_PIN_2
#define ETH_RSTN_GPIO_Port GPIOB
#define STAT1_Pin GPIO_PIN_0
#define STAT1_GPIO_Port GPIOG
#define STAT2_Pin GPIO_PIN_1
#define STAT2_GPIO_Port GPIOG
#define ETH_INTn_Pin GPIO_PIN_9
#define ETH_INTn_GPIO_Port GPIOE
#define USER_LED_R_Pin GPIO_PIN_14
#define USER_LED_R_GPIO_Port GPIOB
#define LCD2_DCX_Pin GPIO_PIN_2
#define LCD2_DCX_GPIO_Port GPIOG
#define LCD2_RSVD_Pin GPIO_PIN_3
#define LCD2_RSVD_GPIO_Port GPIOG
#define LCD1_DCX_Pin GPIO_PIN_8
#define LCD1_DCX_GPIO_Port GPIOC
#define LCD1_RSVD_Pin GPIO_PIN_9
#define LCD1_RSVD_GPIO_Port GPIOC
#define LCD1_NSS_Pin GPIO_PIN_15
#define LCD1_NSS_GPIO_Port GPIOA
#define LCD2_SCK_Pin GPIO_PIN_10
#define LCD2_SCK_GPIO_Port GPIOC
#define LCD2_MISO_Pin GPIO_PIN_11
#define LCD2_MISO_GPIO_Port GPIOC
#define LCD2_MOSI_Pin GPIO_PIN_12
#define LCD2_MOSI_GPIO_Port GPIOC
#define LCD1_MOSI_Pin GPIO_PIN_5
#define LCD1_MOSI_GPIO_Port GPIOB
#define GPIO_EXPANDER_SCL_Pin GPIO_PIN_8
#define GPIO_EXPANDER_SCL_GPIO_Port GPIOB
#define GPIO_EXPANDER_SDA_Pin GPIO_PIN_9
#define GPIO_EXPANDER_SDA_GPIO_Port GPIOB
#define USER_LED_Y_Pin GPIO_PIN_1
#define USER_LED_Y_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
