/**
  ******************************************************************************
  * @file    stm32f4xx_nucleo_ihm01a1.c
  * @author  IPC Rennes, modified by Markus Petermann
  * @version V1.7.0_MP
  * @date    December 17th, 2016
  * @brief   BSP driver for x-nucleo-ihm01a1 Nucleo extension board
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

#include <ch.h>
#include <hal.h>

#include "stm32f4xx_nucleo_ihm01a1.h"
#include "controller.h"


/// SPI Maximum Timeout values for flags waiting loops
#define SPIx_TIMEOUT_MAX                      ((uint32_t)0x1000)


/*
 * L6474 SPI (Master, 5MHz, CPHA=1, CPOL=1, MSb first, 8 bit).
 */
static SPIConfig spicfg = {
	NULL,								/* Operation complete cb */
	GPIOB,						        /* CS Port */
	GPIOB_PIN6_SPI1_CS,					/* CS PIN  */
	SPI_CR1_CPHA | SPI_CR1_CPOL | \
    SPI_CR1_BR_2 | SPI_CR1_BR_0 | \
    SPI_CR1_MSTR                        /* CR1 Config Register */
};


/*
 * PWM configuration structure for TIM3
 */
static PWMConfig pwm1cfg = {
	PWM_CLK,								/* PWM clock frequency.						*/
	PWM_PERIOD,								/* PWM period								*/
	pwm1cb,									/* PWM callback pointer						*/
	{										/* PWMChannelConfig Struct					*/
		{PWM_OUTPUT_DISABLED, NULL},		/* pwmmode_t, pwmcallback_t	for CH1			*/
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* pwmmode_t, pwmcallback_t	for CH2			*/
		{PWM_OUTPUT_DISABLED, NULL},		/* pwmmode_t, pwmcallback_t	for CH3			*/
		{PWM_OUTPUT_DISABLED, NULL}		    /* pwmmode_t, pwmcallback_t	for CH4			*/
	},
	0,										/* TIM CR2 register 						*/
	0										/* TIM DIER register 						*/
};

/*
 * PWM configuration structure for TIM2
 */
static PWMConfig pwm2cfg = {
	PWM_CLK,								/* PWM clock frequency.						*/
	PWM_PERIOD,								/* PWM period								*/
	pwm2cb,									/* PWM callback pointer						*/
	{										/* PWMChannelConfig Struct					*/
		{PWM_OUTPUT_DISABLED, NULL},		/* pwmmode_t, pwmcallback_t	for CH1			*/
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* pwmmode_t, pwmcallback_t	for CH2			*/
		{PWM_OUTPUT_DISABLED, NULL},		/* pwmmode_t, pwmcallback_t	for CH3			*/
		{PWM_OUTPUT_DISABLED, NULL}		    /* pwmmode_t, pwmcallback_t	for CH4			*/
	},
	0,										/* TIM CR2 register 						*/
	0										/* TIM DIER register 						*/
};

/*
 * PWM configuration structure for TIM4, only uses soft PWM via callback function
 */
static PWMConfig pwm3cfg = {
	PWM_CLK,								/* PWM clock frequency.						*/
	PWM_PERIOD,								/* PWM period								*/
	pwm3cb,									/* PWM callback pointer						*/
	{										/* PWMChannelConfig Struct					*/
		{PWM_OUTPUT_DISABLED, NULL},		/* pwmmode_t, pwmcallback_t	for CH1			*/
		{PWM_OUTPUT_DISABLED, NULL},		/* pwmmode_t, pwmcallback_t	for CH2			*/
		{PWM_OUTPUT_DISABLED, NULL},		/* pwmmode_t, pwmcallback_t	for CH3			*/
		{PWM_OUTPUT_DISABLED, NULL}		    /* pwmmode_t, pwmcallback_t	for CH4			*/
	},
	0,										/* TIM CR2 register 						*/
	0										/* TIM DIER register 						*/
};


void L6474_Board_Delay(uint32_t delay);                                 /* Delay of the requested number of milliseconds */
void L6474_Board_DisableIrq(void);                                      /* Disable Irq */
void L6474_Board_EnableIrq(void);                                       /* Enable Irq */
void L6474_Board_GpioInit(uint8_t deviceId);                            /* Initialise GPIOs used for L6474s */
void L6474_Board_Pwm1SetFreq(uint32_t newFreq);                         /* Set PWM1 frequency and start it */
void L6474_Board_Pwm2SetFreq(uint32_t newFreq);                         /* Set PWM2 frequency and start it */
void L6474_Board_Pwm3SetFreq(uint32_t newFreq);                         /* Set PWM3 frequency and start it */
void L6474_Board_PwmInit(uint8_t deviceId);                             /* Init the PWM of the specified device */
void L6474_Board_PwmStop(uint8_t deviceId);                             /* Stop the PWM of the specified device */
void L6474_Board_ReleaseReset(uint8_t deviceId);                        /* Reset the L6474 reset pin */
void L6474_Board_Reset(uint8_t deviceId);                               /* Set the L6474 reset pin */
void L6474_Board_SetDirectionGpio(uint8_t deviceId, uint8_t gpioState); /* Set direction GPIO */
uint8_t L6474_Board_SpiInit(void);                                                                      /* Initialise the SPI used for L6474s */
uint8_t L6474_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices); /* Write bytes to the L6474s via SPI */


/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay time length in milliseconds
 * @retval None
 **********************************************************/
void L6474_Board_Delay(uint32_t delay)
{
    chThdSleepMilliseconds(delay);
}

/******************************************************//**
 * @brief This function disable the interruptions
  * @retval None
 **********************************************************/
void L6474_Board_DisableIrq(void)
{
    chSysLock();
}

/******************************************************//**
 * @brief This function enable the interruptions
 * @retval None
 **********************************************************/
void L6474_Board_EnableIrq(void)
{
    chSysUnlock();
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the L6474s
 * @param[in] deviceId (from 0 to 2)
 * @retval None
  **********************************************************/
void L6474_Board_GpioInit(uint8_t deviceId)
{
    if (deviceId==0)
    {
        L6474_Board_Reset(0);
    }

}

/******************************************************//**
 * @brief  Sets the frequency of PWM1 used by device 0
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void L6474_Board_Pwm1SetFreq(uint32_t newFreq)
{
    pwmcnt_t period = PWM_CLK / newFreq;
    pwmEnableChannel(&BSP_MOTOR_CONTROL_BOARD_PWM1_DRV, BSP_MOTOR_CONTROL_BOARD_PWM1_CH, period>>1);
    pwmChangePeriod(&BSP_MOTOR_CONTROL_BOARD_PWM1_DRV, period);
}

/******************************************************//**
 * @brief  Sets the frequency of PWM2 used by device 1
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void L6474_Board_Pwm2SetFreq(uint32_t newFreq)
{
    pwmcnt_t period = PWM_CLK / newFreq;
    pwmEnableChannel(&BSP_MOTOR_CONTROL_BOARD_PWM2_DRV, BSP_MOTOR_CONTROL_BOARD_PWM2_CH, period>>1);
    pwmChangePeriod(&BSP_MOTOR_CONTROL_BOARD_PWM2_DRV, period);
}

/******************************************************//**
 * @brief  Sets the frequency of PWM3 used by device 2
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void L6474_Board_Pwm3SetFreq(uint32_t newFreq)
{
    /* double frequency, because we use softPWM need to toggle the pin twice per period */
    newFreq = newFreq*2;
    pwmcnt_t period = PWM_CLK / newFreq;
    pwmEnableChannel(&BSP_MOTOR_CONTROL_BOARD_PWM3_DRV, BSP_MOTOR_CONTROL_BOARD_PWM3_CH, period>>1);
    pwmChangePeriod(&BSP_MOTOR_CONTROL_BOARD_PWM3_DRV, period);
}

/******************************************************//**
 * @brief  Initialises the PWM uses by the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 * @note Device 0 uses PWM1 based on TIM3
 * Device 1 uses PWM2 based on TIM2
 * Device 2 uses softPWM based on TIM4
 **********************************************************/
void L6474_Board_PwmInit(uint8_t deviceId)
{
    switch (deviceId)
    {
        case 0:
            pwmStart(&BSP_MOTOR_CONTROL_BOARD_PWM1_DRV,&pwm1cfg);
            pwmEnablePeriodicNotification(&BSP_MOTOR_CONTROL_BOARD_PWM1_DRV);
            break;
        case  1:
            pwmStart(&BSP_MOTOR_CONTROL_BOARD_PWM2_DRV,&pwm2cfg);
            pwmEnablePeriodicNotification(&BSP_MOTOR_CONTROL_BOARD_PWM2_DRV);
            break;
        case 2:
            pwmStart(&BSP_MOTOR_CONTROL_BOARD_PWM3_DRV,&pwm3cfg);
            pwmEnablePeriodicNotification(&BSP_MOTOR_CONTROL_BOARD_PWM3_DRV);
            break;
    }
}

/******************************************************//**
 * @brief  Stops the PWM uses by the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_Board_PwmStop(uint8_t deviceId)
{
    switch (deviceId)
    {
        case 0:
            pwmDisableChannel(&BSP_MOTOR_CONTROL_BOARD_PWM1_DRV,BSP_MOTOR_CONTROL_BOARD_PWM1_CH);
            break;
        case  1:
            pwmDisableChannel(&BSP_MOTOR_CONTROL_BOARD_PWM2_DRV,BSP_MOTOR_CONTROL_BOARD_PWM2_CH);
            break;
        case 2:
            pwmDisableChannel(&BSP_MOTOR_CONTROL_BOARD_PWM3_DRV,BSP_MOTOR_CONTROL_BOARD_PWM3_CH);
            break;
        default:
            break;
    }
}

/******************************************************//**
 * @brief  Releases the L6474 reset (pin set to High) of all devices
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_Board_ReleaseReset(uint8_t deviceId)
{
    (void)deviceId;
    palSetPad(BSP_MOTOR_CONTROL_BOARD_RESET_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_PIN);
}

/******************************************************//**
 * @brief  Resets the L6474 (reset pin set to low) of all devices
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_Board_Reset(uint8_t deviceId)
{
    (void)deviceId;
    palClearPad(BSP_MOTOR_CONTROL_BOARD_RESET_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_PIN);
}

/******************************************************//**
 * @brief  Set the GPIO used for the direction
 * @param[in] deviceId (from 0 to 2)
 * @param[in] gpioState state of the direction gpio (0 to reset, 1 to set)
 * @retval None
 **********************************************************/
void L6474_Board_SetDirectionGpio(uint8_t deviceId, uint8_t gpioState)
{
    switch (deviceId)
    {
        case 2:
            palWritePad(BSP_MOTOR_CONTROL_BOARD_DIR_3_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_3_PIN, gpioState);
            break;
        case 1:
            palWritePad(BSP_MOTOR_CONTROL_BOARD_DIR_2_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_2_PIN, gpioState);
            break;
        case 0:
            palWritePad(BSP_MOTOR_CONTROL_BOARD_DIR_1_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_1_PIN, gpioState);
            break;
        default:
        ;
    }
}

/******************************************************//**
 * @brief  Initialise the SPI used by L6474
 * @retval 0
 **********************************************************/
uint8_t L6474_Board_SpiInit(void)
{
    spiStart(&SPIDx,&spicfg);
    return 0;
}

/******************************************************//**
 * @brief  Write and read SPI byte to the L6474
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @param[in] nbDevices Number of device in the SPI chain
 * @retval 0
 **********************************************************/
uint8_t L6474_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices)
{
    spiSelect(&SPIDx);
    spiExchange(&SPIDx, nbDevices, pByteToTransmit, pReceivedByte);
    spiUnselect(&SPIDx);
    return 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
