/*
 * controller.h
 *
 * Controller initialization and shell commands.
 *
 * 	Created on: 21.12.2016
 *      Author: mp
 *
 */

#ifndef __CONTROLLER_H
#define __CONTROLLER_H





void pwm1cb(PWMDriver *pwmp);
void pwm2cb(PWMDriver *pwmp);
void pwm3cb(PWMDriver *pwmp);


void controllerInit(void);

void cmd_run(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_move(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_get(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_set(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_stop(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[]);





#endif /* __CONTROLLER_H */
