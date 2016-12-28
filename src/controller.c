/*
 * controller.c
 *
 * Controller initialization and shell commands.
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

#include "MotorControl/motorcontrol.h"
#include "Components/l6474/l6474.h"
#include "STM32F4xx-Nucleo/stm32f4xx_nucleo_ihm01a1.h"
#include "controller.h"


static volatile uint16_t gLastError;


/* L6474 Board init parameters */
L6474_Init_t gL6474InitParams =
{
    16000,                             /// Acceleration rate in step/s2. Range: (0..+inf).
    16000,                             /// Deceleration rate in step/s2. Range: (0..+inf). 
    10000,                             /// Maximum speed in step/s. Range: (30..10000].
    1000,                              /// Minimum speed in step/s. Range: [30..10000).
    400,                               /// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
    1000,                              /// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
    L6474_CONFIG_OC_SD_ENABLE,         /// Overcurrent shutwdown (OC_SD field of CONFIG register). 
    L6474_CONFIG_EN_TQREG_TVAL_USED,   /// Torque regulation method (EN_TQREG field of CONFIG register).
    L6474_STEP_SEL_1_16,               /// Step selection (STEP_SEL field of STEP_MODE register).
    L6474_SYNC_SEL_1_2,                /// Sync selection (SYNC_SEL field of STEP_MODE register).
    L6474_FAST_STEP_12us,              /// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
    L6474_TOFF_FAST_8us,               /// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
    3,                                 /// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
    21,                                /// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
    L6474_CONFIG_TOFF_044us,           /// Target Swicthing Period (field TOFF of CONFIG register).
    L6474_CONFIG_SR_320V_us,           /// Slew rate (POW_SR field of CONFIG register).
    L6474_CONFIG_INT_16MHZ,            /// Clock setting (OSC_CLK_SEL field of CONFIG register).
    (L6474_ALARM_EN_OVERCURRENT      |
     L6474_ALARM_EN_THERMAL_SHUTDOWN |
     L6474_ALARM_EN_THERMAL_WARNING  |
     L6474_ALARM_EN_UNDERVOLTAGE     |
     L6474_ALARM_EN_SW_TURN_ON       |
     L6474_ALARM_EN_WRONG_NPERF_CMD)    /// Alarm (ALARM_EN register).
};


/*
 * Callback handler for PWM controlling device 0
 */
void pwm1cb(PWMDriver *pwmp) {
    (void)pwmp;
    if (BSP_MotorControl_GetDeviceState(0) != INACTIVE) {
        BSP_MotorControl_StepClockHandler(0);
    }
    palTogglePad(GPIOA, GPIOA_PIN0);
}

/*
 * Callback handler for PWM controlling device 1
 */
void pwm2cb(PWMDriver *pwmp) {
    (void)pwmp;
    if (BSP_MotorControl_GetDeviceState(1) != INACTIVE) {
        BSP_MotorControl_StepClockHandler(1);
    }
}

/*
 * Callback handler for PWM controlling device 2, uses softPWM
 */
void pwm3cb(PWMDriver *pwmp) {
    (void)pwmp;
    palTogglePad(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN);
    if ( (BSP_MotorControl_GetDeviceState(2) != INACTIVE) && 
         (palReadPad(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN) == PAL_HIGH) ) {
            BSP_MotorControl_StepClockHandler(2);
    }
}


/*
 * Callback handler for FLAG pin interrupt
 */
void FlagInterruptHandler(void) {
    /* Get the value of the status register via the L6474 command GET_STATUS */
    uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);
  
    /* Check HIZ flag: if set, power brigdes are disabled */
    if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ) {
        // HIZ state           
    }

    /* Check direction bit */
    if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR) {
        // Forward direction is set           
    } else {
        // Backward direction is set         
    }  

    /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
    /* This often occures when a command is sent to the L6474 */
    /* while it is in HIZ state */
    if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD) {
        // Command received by SPI can't be performed          
    }  

    /* Check WRONG_CMD flag: if set, the command does not exist */
    if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD) {
        //command received by SPI does not exist        
    }  

    /* Check UVLO flag: if not set, there is an undervoltage lock-out */
    if ((statusRegister & L6474_STATUS_UVLO) == 0) {
        //undervoltage lock-out 
    }  

    /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
    if ((statusRegister & L6474_STATUS_TH_WRN) == 0) {
        //thermal warning threshold is reached
    }    

    /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
    if ((statusRegister & L6474_STATUS_TH_SD) == 0) {
        //thermal shut down threshold is reached 
    }    

    /* Check OCD  flag: if not set, there is an overcurrent detection */
    if ((statusRegister & L6474_STATUS_OCD) == 0) {
        //overcurrent detection 
    }
}


/*
 * Error handler
 */
void ErrorHandler(uint16_t error) {
    /* Backup error number */
    gLastError = error;
  
    /* Infinite loop */
    while(1) {
    }
}


/*
 * Initializes the motor controller
 */
void controllerInit(void) {
    BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
    BSP_MotorControl_AttachFlagInterrupt(FlagInterruptHandler);
    BSP_MotorControl_AttachErrorHandler(ErrorHandler);
}


/*
 * Shell command: run
 */
void cmd_run(BaseSequentialStream *chp, int argc, char *argv[]) {
    uint8_t id;
    uint16_t speed;
    motorDir_t direction;
    
    if(!(argc==3)) {
        chprintf(chp, "Usage: run [ID] (F|B) [SPEED]\r\n");
        return;
    }
    
    // FIXME: Set limit to actual number of present devices!!!!!!!!!!!!
    id = (uint8_t)atoi(argv[0]) - 1;
    if(id != 0) {
        chprintf(chp, "Given ID out of range (1)\r\n");
        return;
    }
    
    if(strcmp(argv[1],"F") == 0) {
        direction = FORWARD;
    } else if(strcmp(argv[1],"B") == 0) {
        direction = BACKWARD;
    } else {
        chprintf(chp, "Direction invalid  (F|B)\r\n");
        return;
    }
    
    speed = atoi(argv[2]);
    if(speed>10000) {
        chprintf(chp, "Given speed out of range (30..10000)\r\n");
        return;
    }
    
    BSP_MotorControl_SetMaxSpeed(id,speed);
    BSP_MotorControl_Run(id,direction);
}


/*
 * Shell command: move
 */
void cmd_move(BaseSequentialStream *chp, int argc, char *argv[]) {
    uint8_t id;
    uint16_t speed;
    uint32_t steps;
    motorDir_t direction;
    
    if(!(argc==4)) {
        chprintf(chp, "Usage: move [ID] (F|B) [STEPS] [SPEED]\r\n");
        return;
    }
    
    // FIXME: Set limit to actual number of present devices!!!!!!!!!!!!
    id = (uint8_t)atoi(argv[0]) - 1;
    if(id != 0) {
        chprintf(chp, "Given ID out of range (1)\r\n");
        return;
    }
    
    if(strcmp(argv[1],"F") == 0) {
        direction = FORWARD;
    } else if(strcmp(argv[1],"B") == 0) {
        direction = BACKWARD;
    } else {
        chprintf(chp, "Direction invalid  (F|B)\r\n");
        return;
    }
    
    steps = atol(argv[2]);
    
    speed = atoi(argv[3]);
    if(speed>10000) {
        chprintf(chp, "Given speed out of range (30..10000)\r\n");
        return;
    }
    
    BSP_MotorControl_SetMaxSpeed(id,speed);
    BSP_MotorControl_Move(id,direction,steps);
}


/*
 * Shell command: get
 */
void cmd_get(BaseSequentialStream *chp, int argc, char *argv[]) {
    uint8_t id;
    
    if(!(argc==2)) {
        chprintf(chp, "Usage: get [ID] (state)\r\n");
        return;
    }
    
    // FIXME: Set limit to actual number of present devices!!!!!!!!!!!!
    id = (uint8_t)atoi(argv[0]) - 1;
    if(id != 0) {
        chprintf(chp, "Given ID out of range (1)\r\n");
        return;
    }
    
    if(strcmp(argv[1],"state") == 0) {
        motorState_t state = BSP_MotorControl_GetDeviceState(id);
        chprintf(chp, "State: %d\r\n", state);
//    } else if(strcmp(argv[1],"xxx") == 0) {
//        ...;
    } else {
        chprintf(chp, "Parameter invalid  (state)\r\n");
        return;
    }
}


/*
 * Shell command: set
 */
void cmd_set(BaseSequentialStream *chp, int argc, char *argv[]) {
    uint8_t id;
    int32_t value;
    
    if(!(argc==3)) {
        chprintf(chp, "Usage: set [ID] (step_mode|max_current) [value]\r\n");
        return;
    }
    
    // FIXME: Set limit to actual number of present devices!!!!!!!!!!!!
    id = (uint8_t)atoi(argv[0]) - 1;
    if(id != 0) {
        chprintf(chp, "Given ID out of range (1)\r\n");
        return;
    }
    
    uint32_t parameter = 0;
    if(strcmp(argv[1],"step_mode") == 0) { parameter = 1; }
    else if(strcmp(argv[1],"max_current") == 0) { parameter = 2; }
    if(parameter == 0) {
        chprintf(chp, "Invalid parameter (step_mode|max_current)\r\n");
        return;
    }
    
    /* We may now assume that we have a valid channel and parameter, so let's check the value and set it */
    switch(parameter) {
        
        case 1: /* set step_mode */
            value = atol(argv[2]);
            switch(value) {
                case 1:
                    BSP_MotorControl_SelectStepMode(id,STEP_MODE_FULL);
                    break;
                case 2:
                    BSP_MotorControl_SelectStepMode(id,STEP_MODE_HALF);
                    break;
                case 4:
                    BSP_MotorControl_SelectStepMode(id,STEP_MODE_1_4);
                    break;
                case 8:
                    BSP_MotorControl_SelectStepMode(id,STEP_MODE_1_8);
                    break;
                default:
                    BSP_MotorControl_SelectStepMode(id,STEP_MODE_1_16);
                    break;
            }
            break;
            
        case 2: /* set max_current */
            value = atol(argv[2]);
            chprintf(chp, "Not implemented!\r\n");
            break;
            
    }
}


/*
 * Shell command: stop
 */
void cmd_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
    uint8_t id;
    
    if(!(argc==2)) {
        chprintf(chp, "Usage: stop [ID] (H|S|Z)\r\n");
        return;
    }
    
    // FIXME: Set limit to actual number of present devices!!!!!!!!!!!!
    id = (uint8_t)atoi(argv[0]) - 1;
    if(id != 0) {
        chprintf(chp, "Given ID out of range (1)\r\n");
        return;
    }
    
    if(strcmp(argv[1],"H") == 0) {
        BSP_MotorControl_HardStop(id);
    } else if(strcmp(argv[1],"S") == 0) {
        BSP_MotorControl_SoftStop(id);
    } else if(strcmp(argv[1],"Z") == 0) {
        BSP_MotorControl_CmdHardHiZ(id);
    } else {
        chprintf(chp, "Mode invalid  (H|S|Z)\r\n");
        return;
    }
}


/*
 * Shell command: reset /////////////// FIXME ///////////////
 */
void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    uint8_t id;
    
    if(!(argc==0)) {
        chprintf(chp, "Usage: reset\r\n");
        return;
    }
    
    BSP_MotorControl_CmdHardHiZ(id);
    chThdSleepMilliseconds(10);
    BSP_MotorControl_Reset(id);
    chThdSleepMilliseconds(10);
    BSP_MotorControl_ReleaseReset(id);
    chThdSleepMilliseconds(10);
    controllerInit();
}

















