/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
 double kalmanFilter(double input);
 typedef struct 
{
    float LastP;//上次估算协方�? 初始化�?�为0.02
    float Now_P;//当前估算协方�? 初始化�?�为0
    float out;//卡尔曼滤波器输出 初始化�?�为0
    float Kg;//卡尔曼增�? 初始化�?�为0
    float Q;//过程噪声协方�? 初始化�?�为0.001
    float R;//观测噪声协方�? 初始化�?�为0.543
} KFP;//Kalman Filter parameter
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
void OTG_FS_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
/* USER CODE BEGIN EFP */
extern unsigned short distance_to_left;
extern unsigned short distance_to_right;
extern unsigned short flag;
extern uint16_t round_loop;

/* USER CODE END EFP */



#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
