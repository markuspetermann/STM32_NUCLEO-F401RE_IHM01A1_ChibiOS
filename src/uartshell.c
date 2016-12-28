/*
 * uartshell.c
 *
 * Implements an UART shell.
 *
 *  Created on: 21.12.2016
 *      Author: mp
 */

#include <ch.h>
#include <hal.h>
#include <shell.h>
#include <chprintf.h>
#include <stdlib.h>
#include <string.h>

#include "uartshell.h"
#include "controller.h"
#include "system.h"


#define SDx             SD2

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(4096)






/*
 * Shell command: mem
 */
static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
 	size_t n, size;

 	(void)argv;
 	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}


/*
 * Shell command: threads
 */
static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
	static const char *states[] = {CH_STATE_NAMES};
	thread_t *tp;

	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: threads\r\n");
		return;
	}
	chprintf(chp, "    addr    stack prio refs     state time\r\n");
	tp = chRegFirstThread();
	do {
		chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %8u ",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state]
			   
#if (CH_DBG_THREADS_PROFILING == TRUE)
			   , chThdGetTicksX(tp)
#endif
			   
		);
		chprintf(chp, tp->p_name);
		chprintf(chp, "\r\n");
		tp = chRegNextThread(tp);
	} while (tp != NULL);
}


/*
 * Shell command: reboot
 */
static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]) {

  if ((argc != 1)) {
    chprintf(chp, "Usage: reboot 0\r\n");
    return;
  }
  
  uint32_t mode = atoi(argv[0]);
  if(mode != 0) {
    chprintf(chp, "Invalid mode, use 0 for system reboot\r\n");
    return;
  }
  
  /* FIXME: message is not being sent */
  chprintf(chp, "Rebooting...\r\n");
  /* Disable hardware to avoid fire */
  //PTGlobalOnOff(PT_GLOBAL_OFF);
  /* Reboot */
  reboot(mode);
}


/*
 * Shell commands
 */
static const ShellCommand commands[] = {
    {"mem", cmd_mem},
	{"threads", cmd_threads},
    {"reboot", cmd_reboot},
    {"run", cmd_run},
    {"move", cmd_move},
    {"get", cmd_get},
    {"set", cmd_set},
    {"stop", cmd_stop},
    {"reset", cmd_reset},
    {NULL, NULL}
};


/*
 * Shell config struct
 */
static const ShellConfig shell_cfg = {
    (BaseSequentialStream *)&SDx,
    commands
};


/*
 * Initialize UARTShell
 */
void uartshellInit(void) {
    sdStart(&SDx, NULL);
    shellInit();
}


/*
 * Setup Shell whenever neccessary
 */
void uartshellThread(void *arg) {
    (void)arg;
    
    chRegSetThreadName("UARTShell Thread");
	thread_t *shelltp = NULL;
    uartshellInit();
    
    chThdSleepMilliseconds(1000);
    
    while(1) {
        if (!shelltp)
            shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO-1);
        else if (chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);    			/* Recovers memory of the previous shell.   */
            shelltp = NULL;           			/* Triggers spawning of a new shell.        */
        }
        chThdSleepMilliseconds(1000);
    }
    
    return;
}


















