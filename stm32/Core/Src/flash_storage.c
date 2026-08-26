#include "flash_storage.h"
#include "calib.h"
#include <string.h>

FlashCalibData_t active_calib;

void flash_calib_init(void)
{
    FlashCalibData_t *stored = (FlashCalibData_t *)CALIB_FLASH_ADDR;

    /* Check if valid calibration was previously flashed */
    if (stored->magic == CALIB_MAGIC_KEY) {
        memcpy(&active_calib, stored, sizeof(FlashCalibData_t));
    } else {
        /* Fallback to default constants from calib.h */
        active_calib.magic          = CALIB_MAGIC_KEY;
        active_calib.centre_us      = SERVO_CENTRE;
        active_calib.left_us        = SERVO_LEFT;
        active_calib.right_us       = SERVO_RIGHT;
        active_calib.turn_counts_fl = TURN_COUNTS_FL;
        active_calib.turn_counts_fr = TURN_COUNTS_FR;
    }
}

uint8_t flash_calib_save(const FlashCalibData_t *data)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;

    HAL_FLASH_Unlock();

    /* Erase Sector 11 */
    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7V - 3.6V */
    erase_init.Sector       = FLASH_SECTOR_11;
    erase_init.NbSectors    = 1;

    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }

    /* Write data 32-bits (word) at a time */
    uint32_t *src = (uint32_t *)data;
    uint32_t dest = CALIB_FLASH_ADDR;
    uint32_t words = (sizeof(FlashCalibData_t) + 3) / 4;

    for (uint32_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
        dest += 4;
    }

    HAL_FLASH_Lock();
    memcpy(&active_calib, data, sizeof(FlashCalibData_t));
    return 1;
}
