/*
 * Copyright (c) 2023-2024 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <soc.h>

/* CH32V_SYS_R8_SAFE_ACCESS_SIG_REG */
#define SAFE_ACCESS_SIG_KEY_1 (0x57)
#define SAFE_ACCESS_SIG_KEY_2 (0xA8)

// SW reset
#define RB_SOFTWARE_RESET 0x01

// unlock/relock functions are used by clock control, which is a ramfunction
// Consider bringing them into inline version
__ramfunc
void ch32v_sys_unlock(void)
{
	sys_write8(SAFE_ACCESS_SIG_KEY_1, CH32V_SYS_R8_SAFE_ACCESS_SIG_REG);
	sys_write8(SAFE_ACCESS_SIG_KEY_2, CH32V_SYS_R8_SAFE_ACCESS_SIG_REG);
}

__ramfunc
void ch32v_sys_relock(void)
{
	sys_write8(0x00, CH32V_SYS_R8_SAFE_ACCESS_SIG_REG);
}

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	ch32v_sys_unlock();
	sys_write8(RB_SOFTWARE_RESET, CH32V_SYS_R8_RST_WDOG_CTRL_REG);
}
