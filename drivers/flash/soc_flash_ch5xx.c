/*
 * Copyright (c) 2024 Armen Soghoyan <asoghoyan@yahoo.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_flash_controller

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/sys/barrier.h>
#include <soc.h>

#ifdef CONFIG_SOC_CH56X
#include <wch/ISPEM569.h>
#else
#include <wch/ISP5xx.h>
#endif//CONFIG_SOC_CH56X

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_ch5xx, CONFIG_FLASH_LOG_LEVEL);

#define NV_FLASH_NODE DT_INST(0, wch_nv_flash)

#define CODE_FLASH_ADDR			DT_REG_ADDR_BY_NAME(NV_FLASH_NODE, code_flash)
#define CODE_FLASH_OFF			0
#define CODE_FLASH_SZ			DT_REG_SIZE_BY_NAME(NV_FLASH_NODE, code_flash)
#define CODE_FLASH_WRITE_BLK_SZ	DT_PROP(NV_FLASH_NODE, write_block_size)
#define CODE_FLASH_ERASE_BLK_SZ	DT_PROP(NV_FLASH_NODE, erase_block_size)

#define DATA_FLASH_ADDR			DT_REG_ADDR_BY_NAME(NV_FLASH_NODE, data_flash)
#define DATA_FLASH_OFF			(DATA_FLASH_ADDR - CODE_FLASH_ADDR)
#define DATA_FLASH_SZ			DT_REG_SIZE_BY_NAME(NV_FLASH_NODE, data_flash)
#define DATA_FLASH_WRITE_BLK_SZ	DT_PROP(NV_FLASH_NODE, data_write_block_size)
#define DATA_FLASH_ERASE_BLK_SZ	DT_PROP(NV_FLASH_NODE, data_erase_block_size)

BUILD_ASSERT(CODE_FLASH_ADDR + CODE_FLASH_SZ <= DATA_FLASH_ADDR,
	"Data Flash area must follow Code Flash area");

#define FLASH_SEM_TIMEOUT (k_is_in_isr() ? K_NO_WAIT : K_FOREVER)

struct flash_ch5xx_dev_data {
#ifdef CONFIG_MULTITHREADING
	struct k_sem sem;
#endif
};

static inline bool is_code_flash_area(off_t offset, size_t len){
	return offset >= CODE_FLASH_OFF &&
		offset < CODE_FLASH_OFF + CODE_FLASH_SZ &&
		len <= CODE_FLASH_SZ &&
		offset + len <= CODE_FLASH_OFF + CODE_FLASH_SZ;
}
static inline bool is_data_flash_area(off_t offset, size_t len){
	return offset >= DATA_FLASH_OFF &&
		offset < DATA_FLASH_OFF + DATA_FLASH_SZ &&
		len <= DATA_FLASH_SZ &&
		offset + len <= DATA_FLASH_OFF + DATA_FLASH_SZ;
}

#ifdef CONFIG_MULTITHREADING
static inline void flash_ch5xx_sem_take(const struct device *dev)
{
	struct flash_ch5xx_dev_data *data = dev->data;

	k_sem_take(&data->sem, FLASH_SEM_TIMEOUT);
}

static inline void flash_ch5xx_sem_give(const struct device *dev)
{
	struct flash_ch5xx_dev_data *data = dev->data;

	k_sem_give(&data->sem);
}
#else

#define flash_ch5xx_sem_take(dev) do {} while (0)
#define flash_ch5xx_sem_give(dev) do {} while (0)

#endif /* CONFIG_MULTITHREADING */

static int flash_ch5xx_erase(const struct device *dev, off_t offset,
			       size_t len)
{
	int ret = 0;
	unsigned int irq_lock_key;
	
	flash_ch5xx_sem_take(dev);
	if(is_code_flash_area(offset, len)){
		if(!(offset % CODE_FLASH_ERASE_BLK_SZ || len % CODE_FLASH_ERASE_BLK_SZ)){
			irq_lock_key = irq_lock();
			ret = FLASH_ROMA_ERASE(CODE_FLASH_ADDR + offset, len);
			irq_unlock(irq_lock_key);
			if(ret){
				LOG_ERR("Code flash erase failed. Offset: %ld, len: %zu, ret: %d",
					(long) offset, len, ret);
				ret = -EIO;
			}
		}else{
			LOG_ERR("Code flash erase range not aligned. Offset: %ld, len: %zu",
				(long) offset, len);
			ret = -EINVAL;
		}
	}else if(is_data_flash_area(offset, len)){
		if(!(offset % DATA_FLASH_ERASE_BLK_SZ || len % DATA_FLASH_ERASE_BLK_SZ)){
	LOG_DBG("Erase eeprom offset: %ld, len: %zu", (long) offset - DATA_FLASH_OFF, len);
			irq_lock_key = irq_lock();
			ret = EEPROM_ERASE(offset - DATA_FLASH_OFF, len);
			irq_unlock(irq_lock_key);
			if(ret){
				LOG_ERR("Data flash erase failed. Offset: %ld, len: %zu, ret: %d",
					(long) offset, len, ret);
				ret = -EIO;
			}
		}else{
			LOG_ERR("Data flash erase range not aligned. Offset: %ld, len: %zu",
				(long) offset, len);
			ret = -EINVAL;
		}
	}else{
		LOG_ERR("Erase range invalid. Offset: %ld, len: %zu",
			(long) offset, len);
		ret = -EINVAL;
	}
	flash_ch5xx_sem_give(dev);
	LOG_DBG("Erase offset: %ld, len: %zu", (long) offset, len);
	return ret;
}

static int flash_ch5xx_write(const struct device *dev, off_t offset,
			       const void *data, size_t len)
{
	int ret = 0;
	unsigned int irq_lock_key;
	
	flash_ch5xx_sem_take(dev);
	if(is_code_flash_area(offset, len)){
		if(!(offset % CODE_FLASH_WRITE_BLK_SZ || len % CODE_FLASH_WRITE_BLK_SZ)){
			irq_lock_key = irq_lock();
			ret = FLASH_ROMA_WRITE(CODE_FLASH_ADDR + offset, (void*)data, len);
			irq_unlock(irq_lock_key);
			if(ret){
				LOG_ERR("Code flash write failed. Offset: %ld, len: %zu, ret: %d",
					(long) offset, len, ret);
				ret = -EIO;
			}
		}else{
			LOG_ERR("Code flash write range not aligned. Offset: %ld, len: %zu",
				(long) offset, len);
			ret = -EINVAL;
		}
	}else if(is_data_flash_area(offset, len)){
		if(!(offset % DATA_FLASH_WRITE_BLK_SZ || len % DATA_FLASH_WRITE_BLK_SZ)){
			irq_lock_key = irq_lock();
			ret = EEPROM_WRITE(offset - DATA_FLASH_OFF, (void*)data, len);
			irq_unlock(irq_lock_key);
			if(ret){
				LOG_ERR("Data flash write failed. Offset: %ld, len: %zu, ret: %d",
					(long) offset, len, ret);
				ret = -EIO;
			}
		}else{
			LOG_ERR("Data flash write range not aligned. Offset: %ld, len: %zu",
				(long) offset, len);
			ret = -EINVAL;
		}
	}else{
		LOG_ERR("Write range invalid. Offset: %ld, len: %zu",
			(long) offset, len);
		ret = -EINVAL;
	}
	flash_ch5xx_sem_give(dev);
	LOG_DBG("Write offset: %ld, len: %zu", (long) offset, len);
	return ret;
}

static int flash_ch5xx_read(const struct device *dev, off_t offset,
			      void *data,
			      size_t len)
{
	int ret = 0;
	unsigned int irq_lock_key;
	
	flash_ch5xx_sem_take(dev);
	if(is_code_flash_area(offset, len)){
		memcpy(data, (void*)(CODE_FLASH_ADDR + offset), len);
	}else if(is_data_flash_area(offset, len)){
		irq_lock_key = irq_lock();
#ifndef CONFIG_SOC_CH56X
		ret = 
#endif
			EEPROM_READ(offset - DATA_FLASH_OFF, data, len);
		irq_unlock(irq_lock_key);
		if(ret){
			LOG_ERR("Data flash read failed. Offset: %ld, len: %zu, ret: %d",
				(long) offset, len, ret);
			ret = -EIO;
		}
	}else{
		LOG_ERR("Read range invalid. Offset: %ld, len: %zu",
			(long) offset, len);
		ret = -EINVAL;
	}
	flash_ch5xx_sem_give(dev);
	LOG_DBG("Read offset: %ld, len: %zu", (long) offset, len);
	return ret;
}


static const struct flash_parameters *
flash_ch5xx_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	static const struct flash_parameters flash_ch5xx_parameters = {
		.write_block_size = CODE_FLASH_WRITE_BLK_SZ,
		.erase_value = 0xff,
	};

	return &flash_ch5xx_parameters;
}

	
static const struct flash_pages_layout flash_ch5xx_pages_layout[] = {
	{
		.pages_size = CODE_FLASH_ERASE_BLK_SZ,
		.pages_count = CODE_FLASH_SZ / CODE_FLASH_ERASE_BLK_SZ,
	},
#if(CODE_FLASH_ADDR + CODE_FLASH_SZ != DATA_FLASH_ADDR)
	{
		.pages_size = DATA_FLASH_ADDR - (CODE_FLASH_ADDR + CODE_FLASH_SZ),
		.pages_count = 1,
	},
#endif
	{
		.pages_size = DATA_FLASH_ERASE_BLK_SZ,
		.pages_count = DATA_FLASH_SZ / DATA_FLASH_ERASE_BLK_SZ,
	},
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_ch5xx_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	ARG_UNUSED(dev);
	*layout_size = ARRAY_SIZE(flash_ch5xx_pages_layout);
	*layout = flash_ch5xx_pages_layout;
}
#endif //CONFIG_FLASH_PAGE_LAYOUT

static const struct flash_driver_api flash_ch5xx_api = {
	.erase = flash_ch5xx_erase,
	.write = flash_ch5xx_write,
	.read = flash_ch5xx_read,
	.get_parameters = flash_ch5xx_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_ch5xx_page_layout,
#endif
};

static int ch5xx_flash_init(const struct device *dev)
{
	struct flash_ch5xx_dev_data *data = dev->data;

#ifdef CONFIG_MULTITHREADING
	k_sem_init(&data->sem, 1, 1);
#endif /* CONFIG_MULTITHREADING */

	LOG_DBG("Flash initialized. Code BS: %zu, Data BS: %zu",
		CODE_FLASH_WRITE_BLK_SZ, DATA_FLASH_WRITE_BLK_SZ);

#if ((CONFIG_FLASH_LOG_LEVEL >= LOG_LEVEL_DBG) && CONFIG_FLASH_PAGE_LAYOUT)
	const struct flash_pages_layout *layout;
	size_t layout_size;

	flash_ch5xx_page_layout(dev, &layout, &layout_size);
	for (size_t i = 0; i < layout_size; i++) {
		LOG_DBG("Block %zu: bs: %zu count: %zu", i,
			layout[i].pages_size, layout[i].pages_count);
	}
#endif
	//FLASH_ROM_PWR_UP(); Do we need this?
	return 0;
}

static struct flash_ch5xx_dev_data ch5xx_dev_data;


DEVICE_DT_INST_DEFINE(0, ch5xx_flash_init, NULL,
		    &ch5xx_dev_data, NULL, POST_KERNEL,
		    CONFIG_FLASH_INIT_PRIORITY, &flash_ch5xx_api);
