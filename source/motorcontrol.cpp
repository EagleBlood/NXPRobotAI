/*
 * motorcontrol.cpp
 *
 *  Created on: 7 sty 2025
 *      Author: dawid
 */

#include "motorcontrol.h"
#include "fsl_debug_console.h"

/* Function to set motor power and direction */
void Motor_SetPower(int32_t powerLeft, int32_t powerRight, bool brake) {
    if (brake) {
        // Apply brake to both motors
        GPIO_PinWrite(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, 1);
        GPIO_PinWrite(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, 1);
        GPIO_PinWrite(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, 1);
        GPIO_PinWrite(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, 1);

        // Stop PWM
        CTIMER_UpdatePwmDutycycle(CTIMER4_PERIPHERAL, (ctimer_match_t)3, CTIMER4_PWM_0_CHANNEL, 0);
        CTIMER_UpdatePwmDutycycle(CTIMER4_PERIPHERAL, (ctimer_match_t)3, CTIMER4_PWM_1_CHANNEL, 0);
    } else {
        // Debug print statements
        //PRINTF("Scaled Power Left: %d, Scaled Power Right: %d\r\n", abs(powerLeft), abs(powerRight));

        // Left motor control
        if (powerLeft > 0) {
            GPIO_PinWrite(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, 1);
            GPIO_PinWrite(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, 0);
        } else if (powerLeft < 0) {
            GPIO_PinWrite(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, 0);
            GPIO_PinWrite(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, 1);
        } else {
            GPIO_PinWrite(MOTOR_AIN1_GPIO, MOTOR_AIN1_PIN, 0);
            GPIO_PinWrite(MOTOR_AIN2_GPIO, MOTOR_AIN2_PIN, 0);
        }

        // Right motor control
        if (powerRight > 0) {
            GPIO_PinWrite(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, 1);
            GPIO_PinWrite(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, 0);
        } else if (powerRight < 0) {
            GPIO_PinWrite(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, 0);
            GPIO_PinWrite(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, 1);
        } else {
            GPIO_PinWrite(MOTOR_BIN1_GPIO, MOTOR_BIN1_PIN, 0);
            GPIO_PinWrite(MOTOR_BIN2_GPIO, MOTOR_BIN2_PIN, 0);
        }

        CTIMER_UpdatePwmDutycycle(CTIMER4_PERIPHERAL, (ctimer_match_t)3, CTIMER4_PWM_0_CHANNEL, abs(powerLeft));
        CTIMER_UpdatePwmDutycycle(CTIMER4_PERIPHERAL, (ctimer_match_t)3, CTIMER4_PWM_1_CHANNEL, abs(powerRight));
    }
}

/* Callback function for motor PWM control */
//void cbMotor_PWM(uint32_t flags) {
//    //motorControlFlag = true;
//    PRINTF("works");
//}
