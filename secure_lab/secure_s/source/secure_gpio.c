/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <secure_gpio.h>
#include "board.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

#include "fsl_inputmux.h"
#include "fsl_pint.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_SECURE_SW1_GPIO 	   SECGPIO
#define BOARD_SECURE_SW1_GPIO_PORT 0U
#define BOARD_SECURE_SW1_GPIO_PIN  5U

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
* Prototypes
*******************************************************************************/
void sec_pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status);

/*******************************************************************************
 * Code
 ******************************************************************************/
void sec_pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    if (pintr & kPINT_SecPinInt0)
    {
    	PRINTF("\f\r\nSecure PINT S1 Interrupt event detected.");
    }
}


void SecureGPIO_Init (void)
{
    /* Initialize secure GPIO for button S1. */
    CLOCK_EnableClock(kCLOCK_Gpio_Sec);
    RESET_PeripheralReset(kGPIOSEC_RST_SHIFT_RSTn);
    GPIO_PinInit(BOARD_SECURE_SW1_GPIO, BOARD_SECURE_SW1_GPIO_PORT, BOARD_SECURE_SW1_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

    /* Connect INPUTMUX trigger sources for button S1 */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, kINPUTMUX_GpioPort0Pin5ToPintsel);
    INPUTMUX_Deinit(INPUTMUX);

    /* Initialize secure PINT for button S1 */
    PINT_Init(SECPINT);
    PINT_PinInterruptConfig(SECPINT, kPINT_PinInt0, kPINT_PinIntEnableFallEdge, sec_pint_intr_callback);
    PINT_EnableCallbackByIndex(SECPINT, kPINT_PinInt0);
}

void SecureGPIO_Mask (uint32_t iomask, bool secure)
{
	if (secure)
	{
		AHB_SECURE_CTRL->SEC_GPIO_MASK0 = AHB_SECURE_CTRL->SEC_GPIO_MASK0 & ~iomask;
	}
	else
	{
		AHB_SECURE_CTRL->SEC_GPIO_MASK0 = AHB_SECURE_CTRL->SEC_GPIO_MASK0 | iomask;
	}
}
