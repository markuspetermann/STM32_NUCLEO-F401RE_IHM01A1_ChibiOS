/*
 * system.h
 *
 * General system functions, such as reboot
 *
 * 	Created on: 19.09.2016
 *      Author: mp
 *
 */
 
#ifndef __SYSTEM_H
#define __SYSTEM_H


#define REBOOT_SYSTEM             0x00000000    /* Reboot System */


void reboot(uint32_t mode);


#endif /* __SYSTEM_H */
