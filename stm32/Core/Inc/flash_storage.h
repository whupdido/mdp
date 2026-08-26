#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "main.h"

#define CALIB_FLASH_ADDR    0x080E0000u  /* Sector 11 start address */
#define CALIB_MAGIC_KEY     0xABCD1234u  /* Validates saved data */

typedef struct {
    uint32_t magic;
    uint16_t centre_us;
    uint16_t left_us;
    uint16_t right_us;
    uint32_t turn_counts_fl;
    uint32_t turn_counts_fr;
} FlashCalibData_t;

extern FlashCalibData_t active_calib;

void flash_calib_init(void);
uint8_t flash_calib_save(const FlashCalibData_t *data);

#endif
