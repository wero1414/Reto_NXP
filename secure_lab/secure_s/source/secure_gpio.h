/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SECURE_GPIO_H_
#define _SECURE_GPIO_H_

#include "board.h"

void SecureGPIO_Init (void);
void SecureGPIO_Mask (uint32_t iomask, bool secure);

#endif /* _SECURE_GPIO_H_*/
