/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board_init.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "peripherals.h"

void BOARD_Init()
{
    BOARD_InitPins();
    BOARD_InitLCDFXIOPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    BOARD_InitBootPeripherals();
    BOARD_InitMotor();


    /* Enable caching of flash memory */
    SYSCON->LPCAC_CTRL = SYSCON->LPCAC_CTRL & ~SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK;
    SYSCON->NVM_CTRL = SYSCON->NVM_CTRL & ~SYSCON_NVM_CTRL_DIS_FLASH_DATA_MASK;

    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
}

