/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v5.0
processor: LPC55S69
package_id: LPC55S69JBD100
mcu_data: ksdk2_0
processor_version: 0.0.6
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '92', peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/SD1_D2/CTIMER2_MAT3/SCT0_OUT8/CMP0_OUT/PLU_OUT2/SECURE_GPIO0_29,
    mode: inactive, slew_rate: standard, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '94', peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/SD1_D3/CTIMER0_MAT0/SCT0_OUT9/SECURE_GPIO0_30, mode: inactive,
    slew_rate: standard, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '94', peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/SD1_D3/CTIMER0_MAT0/SCT0_OUT9/SECURE_GPIO0_30, mode: inactive,
    slew_rate: standard, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '92', peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/SD1_D2/CTIMER2_MAT3/SCT0_OUT8/CMP0_OUT/PLU_OUT2/SECURE_GPIO0_29,
    mode: inactive, slew_rate: standard, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '90', peripheral: FLEXCOMM7, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_19/FC4_RTS_SCL_SSEL1/UTICK_CAP0/CTIMER0_MAT2/SCT0_OUT2/FC7_TXD_SCL_MISO_WS/PLU_IN4/SECURE_GPIO0_19,
    mode: pullUp, slew_rate: standard, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '74', peripheral: FLEXCOMM7, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_20/FC3_CTS_SDA_SSEL0/CTIMER1_MAT1/CT_INP15/SCT_GPI2/FC7_RXD_SDA_MOSI_DATA/HS_SPI_SSEL0/PLU_IN5/SECURE_GPIO0_20/FC4_TXD_SCL_MISO_WS,
    mode: pullUp, slew_rate: standard, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '76', peripheral: FLEXCOMM7, signal: SCK, pin_signal: PIO0_21/FC3_RTS_SCL_SSEL1/UTICK_CAP3/CTIMER3_MAT3/SCT_GPI3/FC7_SCK/PLU_CLKIN/SECURE_GPIO0_21,
    mode: pullUp, slew_rate: standard, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '4', peripheral: FLEXCOMM7, signal: RTS_SCL_SSEL1, pin_signal: PIO1_20/FC7_RTS_SCL_SSEL1/CT_INP14/FC4_TXD_SCL_MISO_WS/PLU_OUT2, mode: pullUp, slew_rate: standard,
    invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '62', peripheral: FLEXCOMM8, signal: HS_SPI_MISO, pin_signal: PIO1_3/SCT0_OUT4/HS_SPI_MISO/USB0_PORTPWRN/PLU_OUT6, mode: inactive, slew_rate: fast,
    invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '60', peripheral: FLEXCOMM8, signal: HS_SPI_MOSI, pin_signal: PIO0_26/FC2_RXD_SDA_MOSI_DATA/CLKOUT/CT_INP14/SCT0_OUT5/USB0_IDVALUE/FC0_SCK/HS_SPI_MOSI/SECURE_GPIO0_26,
    mode: inactive, slew_rate: fast, invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '61', peripheral: FLEXCOMM8, signal: HS_SPI_SCK, pin_signal: PIO1_2/CTIMER0_MAT3/SCT_GPI6/HS_SPI_SCK/USB1_PORTPWRN/PLU_OUT5, mode: inactive, slew_rate: fast,
    invert: disabled, digi_mode: digital, open_drain: disabled}
  - {pin_num: '59', peripheral: FLEXCOMM8, signal: HS_SPI_SSEL1, pin_signal: PIO1_1/FC3_RXD_SDA_MOSI_DATA/CT_INP3/SCT_GPI5/HS_SPI_SSEL1/USB1_OVERCURRENTN/PLU_OUT4,
    mode: inactive, slew_rate: fast, invert: disabled, digi_mode: digital, open_drain: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void BOARD_InitPins(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                         IOCON_PIO_FUNC1 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

    const uint32_t port1_pin10_config = (/* Pin is configured as PIO1_4 */
                                           IOCON_PIO_FUNC0 |
                                           /* Selects pull-up function */
   										IOCON_PIO_MODE_INACT |
                                           /* Standard mode, output slew rate control is enabled */
                                           IOCON_PIO_SLEW_STANDARD |
                                           /* Input function is not inverted */
                                           IOCON_PIO_INV_DI |
                                           /* Enables digital function */
                                           IOCON_PIO_DIGITAL_EN |
                                           /* Open drain is disabled */
                                           IOCON_PIO_OPENDRAIN_DI);
       /* PORT1 PIN4 (coords: 1) is configured as PIO1_4 */
       IOCON_PinMuxSet(IOCON, 1U, 10U, port1_pin10_config);

       const uint32_t port1_pin9_config = (/* Pin is configured as PIO1_4 */
                                           IOCON_PIO_FUNC0 |
                                           /* Selects pull-up function */
   										IOCON_PIO_MODE_INACT |
                                           /* Standard mode, output slew rate control is enabled */
                                           IOCON_PIO_SLEW_STANDARD |
                                           /* Input function is not inverted */
                                           IOCON_PIO_INV_DI |
                                           /* Enables digital function */
                                           IOCON_PIO_DIGITAL_EN |
                                           /* Open drain is disabled */
                                           IOCON_PIO_OPENDRAIN_DI);
       /* PORT1 PIN4 (coords: 1) is configured as PIO1_4 */
       IOCON_PinMuxSet(IOCON, 1U, 9U, port1_pin9_config);

       const uint32_t port0_pin15_config = (/* Pin is configured as PIO1_4 */
                                           IOCON_PIO_FUNC0 |
                                           /* Selects pull-up function */
                                           IOCON_PIO_MODE_PULLUP |
                                           /* Standard mode, output slew rate control is enabled */
                                           IOCON_PIO_SLEW_STANDARD |
                                           /* Input function is not inverted */
                                           IOCON_PIO_INV_DI |
                                           /* Enables digital function */
                                           IOCON_PIO_DIGITAL_EN |
                                           /* Open drain is disabled */
                                           IOCON_PIO_OPENDRAIN_DI);
       /* PORT1 PIN4 (coords: 1) is configured as PIO1_4 */
       IOCON_PinMuxSet(IOCON, 0U, 15U, port0_pin15_config);

       const uint32_t port1_pin4_config = (/* Pin is configured as PIO1_4 */
                                           IOCON_PIO_FUNC0 |
                                           /* Selects pull-up function */
                                           IOCON_PIO_MODE_PULLUP |
                                           /* Standard mode, output slew rate control is enabled */
                                           IOCON_PIO_SLEW_STANDARD |
                                           /* Input function is not inverted */
                                           IOCON_PIO_INV_DI |
                                           /* Enables digital function */
                                           IOCON_PIO_DIGITAL_EN |
                                           /* Open drain is disabled */
                                           IOCON_PIO_OPENDRAIN_DI);
       /* PORT1 PIN4 (coords: 1) is configured as PIO1_4 */
       IOCON_PinMuxSet(IOCON, 1U, 4U, port1_pin4_config);

       const uint32_t port0_pin19_config = (/* Pin is configured as FC7_TXD_SCL_MISO_WS */
                                            IOCON_PIO_FUNC7 |
                                            /* Selects pull-up function */
                                            IOCON_PIO_MODE_PULLUP |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
       /* PORT0 PIN19 (coords: 90) is configured as FC7_TXD_SCL_MISO_WS */
       IOCON_PinMuxSet(IOCON, 0U, 19U, port0_pin19_config);

       const uint32_t port0_pin20_config = (/* Pin is configured as FC7_RXD_SDA_MOSI_DATA */
                                            IOCON_PIO_FUNC7 |
                                            /* Selects pull-up function */
                                            IOCON_PIO_MODE_PULLUP |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
       /* PORT0 PIN20 (coords: 74) is configured as FC7_RXD_SDA_MOSI_DATA */
       IOCON_PinMuxSet(IOCON, 0U, 20U, port0_pin20_config);

       const uint32_t port0_pin21_config = (/* Pin is configured as FC7_SCK */
                                            IOCON_PIO_FUNC7 |
                                            /* Selects pull-up function */
                                            IOCON_PIO_MODE_PULLUP |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
       /* PORT0 PIN21 (coords: 76) is configured as FC7_SCK */
       IOCON_PinMuxSet(IOCON, 0U, 21U, port0_pin21_config);


       const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                                            IOCON_PIO_FUNC1 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
       /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
       IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);

       const uint32_t port1_pin20_config = (/* Pin is configured as FC7_RTS_SCL_SSEL1 */
                                            IOCON_PIO_FUNC1 |
                                            /* Selects pull-up function */
                                            IOCON_PIO_MODE_PULLUP |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
       /* PORT1 PIN20 (coords: 4) is configured as FC7_RTS_SCL_SSEL1 */
       IOCON_PinMuxSet(IOCON, 1U, 20U, port1_pin20_config);


       //FLEXCOM8

       const uint32_t port0_pin26_config = (/* Pin is configured as HS_SPI_MOSI */
                                                IOCON_PIO_FUNC9 |
                                                /* No addition pin function */
                                                IOCON_PIO_MODE_INACT |
                                                /* Fast mode, slew rate control is disabled */
                                                IOCON_PIO_SLEW_FAST |
                                                /* Input function is not inverted */
                                                IOCON_PIO_INV_DI |
                                                /* Enables digital function */
                                                IOCON_PIO_DIGITAL_EN |
                                                /* Open drain is disabled */
                                                IOCON_PIO_OPENDRAIN_DI);
           /* PORT0 PIN26 (coords: 60) is configured as HS_SPI_MOSI */
           IOCON_PinMuxSet(IOCON, 0U, 26U, port0_pin26_config);

           const uint32_t port1_pin1_config = (/* Pin is configured as HS_SPI_SSEL1 */
                                               IOCON_PIO_FUNC5 |
                                               /* No addition pin function */
                                               IOCON_PIO_MODE_INACT |
                                               /* Fast mode, slew rate control is disabled */
                                               IOCON_PIO_SLEW_FAST |
                                               /* Input function is not inverted */
                                               IOCON_PIO_INV_DI |
                                               /* Enables digital function */
                                               IOCON_PIO_DIGITAL_EN |
                                               /* Open drain is disabled */
                                               IOCON_PIO_OPENDRAIN_DI);
           /* PORT1 PIN1 (coords: 59) is configured as HS_SPI_SSEL1 */
           IOCON_PinMuxSet(IOCON, 1U, 1U, port1_pin1_config);

           const uint32_t port1_pin2_config = (/* Pin is configured as HS_SPI_SCK */
                                               IOCON_PIO_FUNC6 |
                                               /* No addition pin function */
                                               IOCON_PIO_MODE_INACT |
                                               /* Fast mode, slew rate control is disabled */
                                               IOCON_PIO_SLEW_FAST |
                                               /* Input function is not inverted */
                                               IOCON_PIO_INV_DI |
                                               /* Enables digital function */
                                               IOCON_PIO_DIGITAL_EN |
                                               /* Open drain is disabled */
                                               IOCON_PIO_OPENDRAIN_DI);
           /* PORT1 PIN2 (coords: 61) is configured as HS_SPI_SCK */
           IOCON_PinMuxSet(IOCON, 1U, 2U, port1_pin2_config);

           const uint32_t port1_pin3_config = (/* Pin is configured as HS_SPI_MISO */
                                               IOCON_PIO_FUNC6 |
                                               /* No addition pin function */
                                               IOCON_PIO_MODE_INACT |
                                               /* Fast mode, slew rate control is disabled */
                                               IOCON_PIO_SLEW_FAST |
                                               /* Input function is not inverted */
                                               IOCON_PIO_INV_DI |
                                               /* Enables digital function */
                                               IOCON_PIO_DIGITAL_EN |
                                               /* Open drain is disabled */
                                               IOCON_PIO_OPENDRAIN_DI);
           /* PORT1 PIN3 (coords: 62) is configured as HS_SPI_MISO */
           IOCON_PinMuxSet(IOCON, 1U, 3U, port1_pin3_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
