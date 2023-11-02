/*
 * Copyright (c) 2023-2024 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_CH32V_SYSCON_REGS_CH57X_H__
#define __SOC_CH32V_SYSCON_REGS_CH57X_H__

#define CH32V_SYS_R16_CLK_SYS_CFG_REG    (CH32V_SYS_BASE + 0x08)
#define CH32V_SYS_R8_HFCK_PWR_CTRL_REG   (CH32V_SYS_BASE + 0x0A)
#define CH32V_SYS_R8_SLP_CLK_OFF_REG(n)  (CH32V_SYS_BASE + 0x0C + n)
#define CH32V_SYS_R8_SLP_CLK_OFF_CNT     (2)
#define CH32V_SYS_R8_SLP_WAKE_CTRL_REG   (CH32V_SYS_BASE + 0x0E)
#define CH32V_SYS_R8_SLP_POWER_CTRL_REG  (CH32V_SYS_BASE + 0x0F)
#define CH32V_SYS_R16_PIN_ALTERNATE_REG  (CH32V_SYS_BASE + 0x18)
#define CH32V_SYS_R16_PIN_ANALOG_IE_ENG  (CH32V_SYS_BASE + 0x1A)
#define CH32V_SYS_R16_POWER_PLAN_REG     (CH32V_SYS_BASE + 0x20)
#define CH32V_SYS_R16_AUX_POWER_ADJ_REG  (CH32V_SYS_BASE + 0x22)
#define CH32V_SYS_R8_BAT_DET_CTRL_REG    (CH32V_SYS_BASE + 0x24)
#define CH32V_SYS_R8_BAT_DET_CFG_REG     (CH32V_SYS_BASE + 0x25)
#define CH32V_SYS_R8_BAT_STATUS_REG      (CH32V_SYS_BASE + 0x26)
#define CH32V_SYS_R16_INT32K_TUNE_REG    (CH32V_SYS_BASE + 0x2C)
#define CH32V_SYS_R8_XT32K_TUNE_REG      (CH32V_SYS_BASE + 0x2E)
#define CH32V_SYS_R8_CK32K_CONFIG_REG    (CH32V_SYS_BASE + 0x2F)
#define CH32V_SYS_R8_RTC_FLAG_CTRL_REG   (CH32V_SYS_BASE + 0x30)
#define CH32V_SYS_R8_RTC_MODE_CTRL_REG   (CH32V_SYS_BASE + 0x31)
#define CH32V_SYS_R32_RTC_TRIG_REG       (CH32V_SYS_BASE + 0x34)
#define CH32V_SYS_R16_RTC_CNT_32K_REG    (CH32V_SYS_BASE + 0x38)
#define CH32V_SYS_R16_RTC_CNT_2S_REG     (CH32V_SYS_BASE + 0x3A)
#define CH32V_SYS_R32_RTC_CNT_DAY_REG    (CH32V_SYS_BASE + 0x3C)
#define CH32V_SYS_R8_SAFE_ACCESS_SIG_REG (CH32V_SYS_BASE + 0x40)
#define CH32V_SYS_R8_CHIP_ID_REG         (CH32V_SYS_BASE + 0x41)
#define CH32V_SYS_R8_SAFE_ACCESS_ID_REG  (CH32V_SYS_BASE + 0x42)
#define CH32V_SYS_R8_WDOG_COUNT_REG      (CH32V_SYS_BASE + 0x43)
#define CH32V_SYS_R8_GLOB_ROM_CFG_REG    (CH32V_SYS_BASE + 0x44)
#define CH32V_SYS_R8_GLOB_CFG_INFO_REG   (CH32V_SYS_BASE + 0x45)
#define CH32V_SYS_R8_RST_WDOG_CTRL_REG   (CH32V_SYS_BASE + 0x46)
#define CH32V_SYS_R8_GLOB_RESET_KEEP_REG (CH32V_SYS_BASE + 0x47)
#define CH32V_SYS_R8_PLL_CONFIG_REG      (CH32V_SYS_BASE + 0x4B)
#define CH32V_SYS_R8_XT32M_TUNE_REG      (CH32V_SYS_BASE + 0x4E)
#define CH32V_SYS_R16_OSC_CAL_CNT_REG    (CH32V_SYS_BASE + 0x50)
#define CH32V_SYS_R8_OSC_CAL_CTRL_REG    (CH32V_SYS_BASE + 0x52)

#endif /* __SOC_CH32V_SYSCON_REGS_CH57X_H__ */
