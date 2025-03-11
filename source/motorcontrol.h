/*
 * motorcontrol.h
 *
 *  Created on: 7 sty 2025
 *      Author: dawid
 */

#ifndef _MOTORCONTROL_H_
#define _MOTORCONTROL_H_

#include <stdbool.h>
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include <stdio.h>
#include "board.h"

/* Motor control pin definitions */
#define MOTOR_AIN1_GPIO BOARD_INITMOTOR_AIN1_GPIO
#define MOTOR_AIN1_PIN  BOARD_INITMOTOR_AIN1_PIN
#define MOTOR_AIN2_GPIO BOARD_INITMOTOR_AIN2_GPIO
#define MOTOR_AIN2_PIN  BOARD_INITMOTOR_AIN2_PIN
#define MOTOR_BIN1_GPIO BOARD_INITMOTOR_BIN1_GPIO
#define MOTOR_BIN1_PIN  BOARD_INITMOTOR_BIN1_PIN
#define MOTOR_BIN2_GPIO BOARD_INITMOTOR_BIN2_GPIO
#define MOTOR_BIN2_PIN  BOARD_INITMOTOR_BIN2_PIN


/* Global speed multiplier */
extern volatile float SpeedMultiplier;

/* Function prototypes */
void Motor_SetPower(int32_t powerLeft, int32_t powerRight, bool brake);
void cbMotor_PWM(uint32_t flags);

#endif /* _MOTORCONTROL_H_ */
