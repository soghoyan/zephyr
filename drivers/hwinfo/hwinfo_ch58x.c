/*
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <string.h>
 #include <soc.h>
#include <zephyr/drivers/hwinfo.h>
#include <wch/ISP5xx.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	int rc;
	uint32_t buf[2];
	unsigned int irq_lock_key;

	irq_lock_key = irq_lock();
	rc = GET_UNIQUE_ID(buf);
	irq_unlock(irq_lock_key);

	if(rc < 0)
		return rc;

	length = length > sizeof(buf) ? sizeof(buf) : length;
	memcpy(buffer, buf, length);

	return length;
}


int z_impl_hwinfo_get_device_eui64(uint8_t *buffer)
{
	int rc;
	uint32_t buf[2];
	unsigned int irq_lock_key;

	irq_lock_key = irq_lock();
	rc = GET_UNIQUE_ID(buf);
	irq_unlock(irq_lock_key);

	memcpy(buffer, buf, sizeof(buf));

	return rc;
}
