/*
 * Copyright 2020-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board_init.h"
#include "fsl_debug_console.h"
#include "image.h"
#include "image_utils.h"
#include "model.h"
#include "output_postproc.h"
#include "timer.h"
#include "video.h"
#include "ov7670.h"
#include "servo_motor_control.h"
int main(void)
{
	BOARD_Init();
    TIMER_Init();

    PRINTF("Camera Init\r\n");
    Ov7670_Init();

    display_init(); // without camera doesn't detect anything

    PRINTF("\r\nSmartDMA Init\r\n");
    ezh_start();

    face_det();

    while(1)
    {

    }
}
