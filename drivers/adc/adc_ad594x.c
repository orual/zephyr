#define DT_DRV_COMPAT adi_ad594x_adc

#include <zephyr/drivers/adc.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/mfd/ad5594x.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_ad594x, CONFIG_ADC_LOG_LEVEL);

struct adc_ad594x_config {
	const struct device *mfd_dev;
};

struct adc_ad594x_data {
	struct adc_context ctx;
	const struct device *dev;
	uint8_t adc_conf;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t channels;
	struct k_thread thread;
	struct k_sem sem;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_AD594X_ACQUISITION_THREAD_STACK_SIZE);
};


static int adc_ad594x_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_ad594x_config *config = dev->config;
	struct adc_ad594x_data *data = dev->data;

	if (channel_cfg->channel_id >= AD594X_PIN_MAX) {
		LOG_ERR("invalid channel id %d", channel_cfg->channel_id);
		return -EINVAL;
	}

	data->adc_conf |= BIT(channel_cfg->channel_id);

	return mfd_ad594x_write_reg(config->mfd_dev, AD594X_REG_ADC_CONFIG, data->adc_conf);
}

static const struct adc_driver_api adc_ad594x_api = {
	.channel_setup = adc_ad594x_channel_setup,
	.read = adc_ad594x_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_ad594x_read_async,
#endif
};

#define ADC_AD594X_DEFINE(inst)							\
	static const struct adc_ad594x_config adc_ad594x_config##inst = {	\
		.mfd_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),			\
	};									\
										\
	static struct adc_ad594x_data adc_ad594x_data##inst;			\
										\
	DEVICE_DT_INST_DEFINE(inst, adc_ad594x_init, NULL,			\
			      &adc_ad594x_data##inst, &adc_ad594x_config##inst, \
			      POST_KERNEL, CONFIG_MFD_INIT_PRIORITY,		\
			      &adc_ad594x_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_AD594X_DEFINE)
