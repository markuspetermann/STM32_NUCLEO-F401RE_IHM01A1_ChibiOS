/*
 * system.c
 *
 * General system functions, such as reboot
 *
 * 	Created on: 19.09.2016
 *      Author: mp
 *
 */

#include <ch.h>
#include <hal.h>

#include "system.h"


void reboot(uint32_t mode) {
    (void)mode;
    NVIC_SystemReset();
}
