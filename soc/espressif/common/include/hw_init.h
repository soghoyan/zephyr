/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_ESPRESSIF_COMMON_HW_INIT_H_
#define _SOC_ESPRESSIF_COMMON_HW_INIT_H_

struct rom_segments {
	unsigned int irom_map_addr;     /* Mapped address (VMA) for IROM region */
	unsigned int irom_flash_offset; /* Flash offset (LMA) for IROM region */
	unsigned int irom_size;         /* Size of IROM region */
	unsigned int drom_map_addr;     /* Mapped address (VMA) for DROM region */
	unsigned int drom_flash_offset; /* Flash offset (LMA) for DROM region */
	unsigned int drom_size;         /* Size of DROM region */
};

void map_rom_segments(int core, struct rom_segments *map);

int hardware_init(void);

/**
 * @brief Slot index MCUboot selected for this boot.
 *
 * In DirectXIP modes MCUboot boots the firmware from either slot pair
 * (slot0/slot0_appcpu or slot1/slot1_appcpu) and publishes the selection in
 * its shared bootinfo. Returns 1 for the secondary pair, otherwise 0 (also
 * when no bootinfo is configured or a non-DirectXIP mode is used).
 */
int esp_mcuboot_boot_slot(void);

#endif /* _SOC_ESPRESSIF_COMMON_HW_INIT_H_ */
