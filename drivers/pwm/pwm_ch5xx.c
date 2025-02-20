//#define DT_DRV_COMPAT wch582f_pwm_led

#define DT_DRV_COMPAT wch_ch582_pwm



#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_ch582, LOG_LEVEL_INF);

struct ch582_pwm_config {
    const struct pinctrl_dev_config *pcfg;
    uint32_t channels;
    uint32_t clk_freq;
    uint8_t clk32k_ch_enable;
};

struct ch582_pwm_data {
    uint32_t duty_cycle;
};

static int ch582_pwm_set_cycles(const struct device *dev, uint32_t channel,
                                uint32_t period_cycles, uint32_t pulse_cycles,
                                pwm_flags_t flags) {
    struct ch582_pwm_config *config = dev->config;
	/* check pwm channel */
	if (channel >= config->channels) {
		return -EINVAL;
	}

	/* check size of pulse and period (2 bytes) */
	if ((period_cycles > 0xFFFFu) ||
	    (pulse_cycles  > 0xFFFFu)) {
		return -EINVAL;
	}

	/* set polarity */
	if (flags & PWM_POLARITY_INVERTED) {
		pwm_invert_en(channel);
	} else {
		pwm_invert_dis(channel);
	}

	/* set pulse and period */
	pwm_set_tcmp(channel, pulse_cycles);
	pwm_set_tmax(channel, period_cycles);

	/* start pwm */
	pwm_start(channel);

	return 0;

}

/* API implementation: get_cycles_per_sec */
static int ch582_get_cycles_per_sec(const struct device *dev,
    uint32_t channel, uint64_t *cycles)
{
    struct ch582_pwm_config *config = dev->config;

    /* check pwm channel */
    if (channel >= config->channels) {
    return -EINVAL;
    }

    if ((config->clk32k_ch_enable & BIT(channel)) != 0U) {
    *cycles = 32000u;
    } else {
    *cycles = sys_clk.pclk * 1000 * 1000 / (reg_pwm_clkdiv + 1);
    }

    return 0;
}

static int ch582_pwm_init(const struct device *dev) {
    struct ch582_pwm_config *config = dev->config;
    
    int32_t status = 0;
	uint8_t clk_32k_en = 0;
	uint32_t pwm_clk_div = 0;
    

    /* Calculate and check PWM clock divider */
	pwm_clk_div = sys_clk.pclk * 1000 * 1000 / config->clk_freq - 1;
	if (pwm_clk_div > 255) {
		return -EINVAL;
	}

	/* Set PWM Peripheral clock */
	pwm_set_clk((unsigned char) (pwm_clk_div & 0xFF));

	/* Set PWM 32k Channel clock if enabled */
	clk_32k_en |= (config->clk32k_ch_enable & BIT(0)) ? PWM_CLOCK_32K_CHN_PWM0 : 0;
	clk_32k_en |= (config->clk32k_ch_enable & BIT(1)) ? PWM_CLOCK_32K_CHN_PWM1 : 0;
	clk_32k_en |= (config->clk32k_ch_enable & BIT(2)) ? PWM_CLOCK_32K_CHN_PWM2 : 0;
	clk_32k_en |= (config->clk32k_ch_enable & BIT(3)) ? PWM_CLOCK_32K_CHN_PWM3 : 0;
	clk_32k_en |= (config->clk32k_ch_enable & BIT(4)) ? PWM_CLOCK_32K_CHN_PWM4 : 0;
	clk_32k_en |= (config->clk32k_ch_enable & BIT(5)) ? PWM_CLOCK_32K_CHN_PWM5 : 0;
	pwm_32k_chn_en(clk_32k_en);

    status = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (status < 0) {
		return status;
	}

	return 0;
}

static const struct pwm_driver_api ch582_pwm_api = {
    .set_cycles = ch582_pwm_set_cycles,
    .get_cycles_per_sec = ch582_get_cycles_per_sec,

};

#define PWM_INIT(n)                                                 \
    static const struct ch582_pwm_config pwm_ch582_config_##n = {   \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		       \
		.clk_freq = DT_INST_PROP(n, clk_freq),	       \
		.channels = DT_INST_PROP(n, channels),			       \
		.clk32k_ch_enable =					       \
			((DT_INST_PROP(n, clk32k_ch0_enable) << 0U) |	       \
			 (DT_INST_PROP(n, clk32k_ch1_enable) << 1U) |	       \
			 (DT_INST_PROP(n, clk32k_ch2_enable) << 2U) |	       \
			 (DT_INST_PROP(n, clk32k_ch3_enable) << 3U) |	       \
			 (DT_INST_PROP(n, clk32k_ch4_enable) << 4U) |	       \
			 (DT_INST_PROP(n, clk32k_ch5_enable) << 5U)),	       \
	};								       \
                                                            \
    DEVICE_DT_INST_DEFINE(n, ch582_pwm_init,				       \
                    NULL, NULL, &pwm_ch582_config_##n,			       \
                    POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,	       \
                    &ch582_pwm_api);
          

DT_INST_FOREACH_STATUS_OKAY(PWM_INIT)
