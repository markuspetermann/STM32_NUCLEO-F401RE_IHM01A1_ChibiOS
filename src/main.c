/*
 * main.c
 *
 * Guess what this does.
 *
 *  Created on: 12.11.2016
 *      Author: mp
 */

#include <ch.h>
#include <hal.h>

#include "MotorControl/motorcontrol.h"
#include "Components/l6474/l6474.h"
#include "STM32F4xx-Nucleo/stm32f4xx_nucleo_ihm01a1.h"
#include "controller.h"
#include "uartshell.h"


/*
 * ALL Thread Working Area Definitions
 */
static THD_WORKING_AREA(uartshellThreadWA,256);




int main(void) {

    halInit();
    chSysInit();
    
    chRegSetThreadName("Main Thread");
    
    
    controllerInit();

    
    chThdCreateStatic(uartshellThreadWA,sizeof(uartshellThreadWA),NORMALPRIO+2,uartshellThread,NULL);

    
    /* Reduce Priority of the main Thread */
	chThdSetPriority(LOWPRIO);

    while(1) {
        chThdSleepMilliseconds(500);
        palTogglePad(GPIOA, GPIOA_PIN1);
    };
}
















