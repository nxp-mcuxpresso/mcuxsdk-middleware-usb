/*
 * Copyright 2018 ,2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v9.0
processor: MIMXRT1062xxxxB
package_id: MIMXRT1062DVL6B
mcu_data: ksdk2_0
processor_version: 9.0.1
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: G14, peripheral: GPIO1, signal: 'gpio_io, 05', pin_signal: GPIO_AD_B0_05, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
  - {pin_num: D13, peripheral: GPIO2, signal: 'gpio_io, 28', pin_signal: GPIO_B1_12, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: L14, peripheral: LPUART1, signal: RX, pin_signal: GPIO_AD_B0_13, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
  - {pin_num: K14, peripheral: LPUART1, signal: TX, pin_signal: GPIO_AD_B0_12, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
  - {pin_num: J3, peripheral: USDHC1, signal: usdhc_clk, pin_signal: GPIO_SD_B0_01, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Disable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: J4, peripheral: USDHC1, signal: usdhc_cmd, pin_signal: GPIO_SD_B0_00, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: J1, peripheral: USDHC1, signal: 'usdhc_data, 0', pin_signal: GPIO_SD_B0_02, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: K1, peripheral: USDHC1, signal: 'usdhc_data, 1', pin_signal: GPIO_SD_B0_03, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: H2, peripheral: USDHC1, signal: 'usdhc_data, 2', pin_signal: GPIO_SD_B0_04, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: J2, peripheral: USDHC1, signal: 'usdhc_data, 3', pin_signal: GPIO_SD_B0_05, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: C14, peripheral: USDHC1, signal: usdhc_vselect, pin_signal: GPIO_B1_14, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_4, slew_rate: Fast}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           

  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_12_GPIO2_IO28, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_14_USDHC1_VSELECT, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3, 0U); 
  IOMUXC_GPR->GPR26 = ((IOMUXC_GPR->GPR26 &
    (~(BOARD_INITPINS_IOMUXC_GPR_GPR26_GPIO_MUX1_GPIO_SEL_MASK))) 
      | IOMUXC_GPR_GPR26_GPIO_MUX1_GPIO_SEL(0x00U) 
    );
  IOMUXC_GPR->GPR27 = ((IOMUXC_GPR->GPR27 &
    (~(BOARD_INITPINS_IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL_MASK))) 
      | IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL(0x00U) 
    );
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, 0x10B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0x10B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0x10B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_12_GPIO2_IO28, 0x017089U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_14_USDHC1_VSELECT, 0x0170A1U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD, 0x017089U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK, 0x014089U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0, 0x017089U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1, 0x017089U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2, 0x017089U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3, 0x017089U); 
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/