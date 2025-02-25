/*
 * Copyright (c) 2025 Armen Soghoyan <asoghoyan@yahoo.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT wch_ch5xx_pwm

LOG_MODULE_REGISTER(pwm_ch5xx, LOG_LEVEL_INF);


#define R32_PWM_CONTROL     0x40005000 // RW, PWM control
#define R8_PWM_OUT_EN       0x40005000 // RW, PWM output enable control
#define R8_PWM_POLAR        0x40005001 // RW, PWM output polarity control
#define R8_PWM_CONFIG       0x40005002 // RW, PWM configuration
#define  RB_PWM_CYCLE_SEL   0x01                      // RW, PWM cycle selection: 0=256/128/64/32 clocks, 1=255/127/63/31 clocks
#define  RB_PWM_STAG_ST     0x02                      // RO, PWM stagger cycle status
#define  RB_PWM_CYC_MOD     0x0C                      // RW, PWM data width mode: 00=8 bits data, 01=7 bits data, 10=6 bits data, 11=5 bits data
#define   PWM_CYC_MOD_8BIT	0x00
#define   PWM_CYC_MOD_7BIT	0x01
#define   PWM_CYC_MOD_6BIT	0x02
#define   PWM_CYC_MOD_5BIT	0x03
#define  RB_PWM4_5_STAG_EN  0x10                      // RW, PWM4/5 stagger output enable: 0=independent output, 1=stagger output
#define  RB_PWM6_7_STAG_EN  0x20                      // RW, PWM6/7 stagger output enable: 0=independent output, 1=stagger output
#define  RB_PWM8_9_STAG_EN  0x40                      // RW, PWM8/9 stagger output enable: 0=independent output, 1=stagger output
#define  RB_PWM10_11_STAG_EN 0x80                      // RW, PWM10/11 stagger output enable: 0=independent output, 1=stagger output
#define R8_PWM_CLOCK_DIV    0x40005003 // RW, PWM clock divisor
#define R32_PWM4_7_DATA     0x40005004 // RW, PWM4-7 data holding
#define R8_PWM4_DATA        0x40005004 // RW, PWM4 data holding
#define R8_PWM5_DATA        0x40005005 // RW, PWM5 data holding
#define R8_PWM6_DATA        0x40005006 // RW, PWM6 data holding
#define R8_PWM7_DATA        0x40005007 // RW, PWM7 data holding
#define R32_PWM8_11_DATA    0x40005008 // RW, PWM8-11 data holding
#define R8_PWM8_DATA        0x40005008 // RW, PWM8 data holding
#define R8_PWM9_DATA        0x40005009 // RW, PWM9 data holding
#define R8_PWM10_DATA       0x4000500A // RW, PWM10 data holding
#define R8_PWM11_DATA       0x4000500B // RW, PWM11 data holding
#define R8_PWM_INT_CTRL     0x4000500C // RW, PWM interrupt control
#define  RB_PWM_IE_CYC      0x01                      // RW, enable interrupt for PWM cycle end
#define  RB_PWM_CYC_PRE     0x02                      // RW, select PWM cycle interrupt point: 0=after count 0xFE (0x7E for 7 bits mode...), 1=after count 0xF0 (0x70 for 7 bits mode...)
#define  RB_PWM_IF_CYC      0x80                      // RW1, interrupt flag for PWM cycle end

#define PWM_CYC_MOD_8BIT	0x00
#define PWM_CYC_MOD_7BIT	0x01
#define PWM_CYC_MOD_6BIT	0x02
#define PWM_CYC_MOD_5BIT	0x03

#define CH5XX_PWM_CHAN_MIN 4
#define CH5XX_PWM_CHAN_MAX 11
#define CH5XX_PWM_CHAN_NUM 8


struct ch5xx_pwm_config {
    const struct pinctrl_dev_config *pcfg;
	uint32_t sys_clk_freq;
	uint32_t prescaler;
	uint32_t period;
};

static int ch5xx_pwm_set_cycles(const struct device *dev, uint32_t channel,
                                uint32_t period_cycles, uint32_t pulse_cycles,
                                pwm_flags_t flags) {
	uint8_t val;

	/* check pwm channel */
	if (channel < CH5XX_PWM_CHAN_MIN || channel > CH5XX_PWM_CHAN_MAX) {
		LOG_ERR("Invalid channel %d", channel);
		return -EINVAL;
	}

	if(period_cycles){
		if(pulse_cycles > period_cycles){
			LOG_ERR("Invalid pulse cycles %d", pulse_cycles);
			return -EINVAL;
		}
		switch(period_cycles){
		case 256:
			val = PWM_CYC_MOD_8BIT;
			break;
		case 255:
			val = PWM_CYC_MOD_8BIT | RB_PWM_CYCLE_SEL;
			break;
		case 128:
			val = PWM_CYC_MOD_7BIT;
			break;
		case 127:
			val = PWM_CYC_MOD_7BIT | RB_PWM_CYCLE_SEL;
			break;
		case 64:
			val = PWM_CYC_MOD_6BIT;
			break;
		case 63:
			val = PWM_CYC_MOD_6BIT | RB_PWM_CYCLE_SEL;
			break;
		case 32:
			val = PWM_CYC_MOD_5BIT;
			break;
		case 31:
			val = PWM_CYC_MOD_5BIT | RB_PWM_CYCLE_SEL;
			break;
		default:
			LOG_ERR("Invalid period cycles %d", period_cycles);
			return -EINVAL;
		}
		/* set period */
		sys_write8(val, R8_PWM_CONFIG);
	}

	/* set pulse */
	sys_write8(pulse_cycles, R8_PWM4_DATA + (channel - CH5XX_PWM_CHAN_MIN));

	/* set polarity */
	val = sys_read8(R8_PWM_POLAR);
	if (flags & PWM_POLARITY_INVERTED) {
		val |= BIT(channel - CH5XX_PWM_CHAN_MIN);
	} else {
		val &= ~BIT(channel - CH5XX_PWM_CHAN_MIN);
	}
	sys_write8(val, R8_PWM_POLAR);

	/* enable channel */
	val = sys_read8(R8_PWM_OUT_EN);
	val |= BIT(channel - CH5XX_PWM_CHAN_MIN);
	sys_write8(val, R8_PWM_OUT_EN);

	return 0;

}

/* API implementation: get_cycles_per_sec */
static int ch5xx_get_cycles_per_sec(const struct device *dev,
    uint32_t channel, uint64_t *cycles)
{
    const struct ch5xx_pwm_config *config = dev->config;

	/* check pwm channel */
	if (channel < CH5XX_PWM_CHAN_MIN || channel > CH5XX_PWM_CHAN_MAX) {
		LOG_ERR("Invalid channel %d", channel);
		return -EINVAL;
	}

	*cycles = config->prescaler ? config->sys_clk_freq / config->prescaler : config->sys_clk_freq;

    return 0;
}

static int ch5xx_pwm_init(const struct device *dev) {
    const struct ch5xx_pwm_config *config = dev->config;
	uint8_t val;
    int32_t status = 0;

	sys_write8(config->prescaler, R8_PWM_CLOCK_DIV);

	switch(config->period){
		case 256:
			val = PWM_CYC_MOD_8BIT;
			break;
		case 255:
			val = PWM_CYC_MOD_8BIT | RB_PWM_CYCLE_SEL;
			break;
		case 128:
			val = PWM_CYC_MOD_7BIT;
			break;
		case 127:
			val = PWM_CYC_MOD_7BIT | RB_PWM_CYCLE_SEL;
			break;
		case 64:
			val = PWM_CYC_MOD_6BIT;
			break;
		case 63:
			val = PWM_CYC_MOD_6BIT | RB_PWM_CYCLE_SEL;
			break;
		case 32:
			val = PWM_CYC_MOD_5BIT;
			break;
		case 31:
			val = PWM_CYC_MOD_5BIT | RB_PWM_CYCLE_SEL;
			break;
		default:
			LOG_ERR("Invalid period cycles %d", config->period);
			return -EINVAL;
	}
	/* set period */
	sys_write8(val, R8_PWM_CONFIG);
LOG_INF("Set period %d, reg %x", config->period, val);	
    status = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (status < 0) {
		return status;
	}

	return 0;
}

static const struct pwm_driver_api ch5xx_pwm_api = {
    .set_cycles = ch5xx_pwm_set_cycles,
    .get_cycles_per_sec = ch5xx_get_cycles_per_sec,

};

#define PWM_INIT(n)             \
	PINCTRL_DT_INST_DEFINE(n);  \
								\
	static const struct ch5xx_pwm_config pwm_ch5xx_config_##n = {   \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		       \
		.sys_clk_freq = DT_INST_PROP_BY_PHANDLE(n, clocks, clock_frequency), \
		.prescaler = DT_INST_PROP(n, prescaler),	       \
		.period = DT_INST_PROP(n, period),	       \
	};								       \
                                                            \
    DEVICE_DT_INST_DEFINE(n, ch5xx_pwm_init,				       \
                    NULL, NULL, &pwm_ch5xx_config_##n,			       \
                    POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,	       \
                    &ch5xx_pwm_api);
          

DT_INST_FOREACH_STATUS_OKAY(PWM_INIT)
