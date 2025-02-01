#ifndef __SD_CARD_H__
#define __SD_CARD_H__

#include "sdmmc_cmd.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#define MOUNT_POINT "/sdcard"
#define EXAMPLE_MAX_CHAR_SIZE   256 

esp_err_t s_example_write_file(const char *path, char *data);
void sd_card_initialize(sdmmc_card_t *card, sdmmc_host_t *host);
void sd_card_deinitialize(const char *mount_point, sdmmc_card_t *card, sdmmc_host_t *host);
void sd_card_test(sdmmc_card_t *card, sdmmc_host_t *host);

#endif //__SD_CARD_H__
