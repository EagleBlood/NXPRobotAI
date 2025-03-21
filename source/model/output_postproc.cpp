/*
 * Copyright 2020-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <config.h>
#include "fsl_debug_console.h"
#include "output_postproc.h"
#include "get_top_n.h"

#ifdef EIQ_GUI_PRINTF
#include "chgui.h"
#endif

status_t MODEL_ProcessOutput(const uint8_t* data, const tensor_dims_t* dims,
                             tensor_type_t type, int inferenceTime)
{

    return kStatus_Success;
}
