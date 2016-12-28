/**
  ******************************************************************************
  * @file    stm32f4xx_nucleo_ihm01a1.h
  * @author  IPC Rennes, modified by Markus Petermann
  * @version V1.7.0_MP
  * @date    December 17th, 2016
  * @brief   Header for BSP driver for x-nucleo-ihm01a1 Nucleo extension board
  *  (based on L6474), modified to work with ChibiOS/RT 3.1.4
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef __STM32F4XX_NUCLEO_IHM01A1_H
#define __STM32F4XX_NUCLEO_IHM01A1_H


#include <ch.h>
#include <hal.h>


/// Interrupt line used for L6474 FLAG
#define EXTI_MCU_LINE_IRQn           (EXTI15_10_IRQn)


/// 4 MHz PWM clock frequency
#define PWM_CLK			4000000
/// Initial PWM period
#define PWM_PERIOD		10000


/// ChibiOS &PWMDx ressource PWMx mapping 
#define BSP_MOTOR_CONTROL_BOARD_PWM1_DRV    PWMD3
#define BSP_MOTOR_CONTROL_BOARD_PWM1_CH     1U
#define BSP_MOTOR_CONTROL_BOARD_PWM2_DRV    PWMD2
#define BSP_MOTOR_CONTROL_BOARD_PWM2_CH     1U
#define BSP_MOTOR_CONTROL_BOARD_PWM3_DRV    PWMD4
#define BSP_MOTOR_CONTROL_BOARD_PWM3_CH     1U


/// GPIO Pin used for the L6474 direction pin of device 0
#define BSP_MOTOR_CONTROL_BOARD_DIR_1_PIN   GPIOA_PIN8_DIR1
/// GPIO port used for the L6474 direction pin of device 0
#define BSP_MOTOR_CONTROL_BOARD_DIR_1_PORT  GPIOA

/// GPIO Pin used for the L6474 direction pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_DIR_2_PIN   GPIOB_PIN5_DIR2
/// GPIO port used for the L6474 direction pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_DIR_2_PORT  GPIOB

/// GPIO Pin used for the L6474 direction pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_DIR_3_PIN   GPIOB_PIN4_DIR3
/// GPIO port used for the L6474 direction pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_DIR_3_PORT  GPIOB

/// GPIO Pin used as softPWM pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN   GPIOB_PIN10_TIM4_SOFTPWM3
/// GPIO port used as softPWM pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT  GPIOB

/// GPIO Pin used for the L6474 reset pin
#define BSP_MOTOR_CONTROL_BOARD_RESET_PIN  GPIOA_PIN9_RESET
/// GPIO port used for the L6474 reset pin
#define BSP_MOTOR_CONTROL_BOARD_RESET_PORT GPIOA

/// GPIO Pin used for the L6474 SPI chip select pin
#define BSP_MOTOR_CONTROL_BOARD_CS_PIN  GPIOB_PIN6_SPI1_CS
/// GPIO port used for the L6474 SPI chip select  pin
#define BSP_MOTOR_CONTROL_BOARD_CS_PORT GPIOB


/// Used SPI
#define SPIDx   SPID1


#endif /* __STM32F4XX_NUCLEO_IHM01A1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
