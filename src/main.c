#include "driver/sdmmc_host.h"

#include "sdmmc_common.h"

#include "freertos/task.h"

#include "sdmmc_cmd.h"
#include "esp_log.h"

static const char TAG[] = "sdmmc_host";

#if 1

#define SDMMC_INIT_STEP(condition, function) \
    do { \
        if ((condition)) { \
            ESP_LOGW(TAG, "%s: running %s", __func__, #function); \
            esp_err_t err = (function)(card); \
            if (err != ESP_OK) { \
                ESP_LOGD(TAG, "%s: %s returned 0x%x", __func__, #function, err); \
                return err; \
            } \
        } \
    } while(0);

esp_err_t mmc_init_ocr(sdmmc_card_t* card)
{
    esp_err_t err;
    /* In SPI mode, READ_OCR (CMD58) command is used to figure out which voltage
     * ranges the card can support. This step is skipped since 1.8V isn't
     * supported on the ESP32.
     */

    uint32_t host_ocr = get_host_ocr(card->host.io_voltage);
    if ((card->ocr & SD_OCR_SDHC_CAP) != 0) {
        host_ocr |= SD_OCR_SDHC_CAP;
    }

    /* If time-out, re-try send_op_cond as MMC */
    if (!host_is_spi(card)) {
        ESP_LOGD(TAG, "trying MMC");
        card->is_mmc = 1;
        //err = sdmmc_send_cmd_send_op_cond(card, host_ocr, &card->ocr);

    sdmmc_command_t cmd = {
                .arg = &card->ocr,
                .flags = SCF_CMD_BCR | SCF_RSP_R3,
                .opcode = SD_APP_OP_COND
        };

        int nretries = SDMMC_SEND_OP_COND_MAX_RETRIES;
        int err_cnt = SDMMC_SEND_OP_COND_MAX_ERRORS;
        for (; nretries != 0; --nretries)  {
            bzero(&cmd, sizeof cmd);
            cmd.arg = &card->ocr;
            cmd.flags = SCF_CMD_BCR | SCF_RSP_R3;
                cmd.arg &= ~MMC_OCR_ACCESS_MODE_MASK;
                cmd.arg |= MMC_OCR_SECTOR_MODE;
                cmd.opcode = MMC_SEND_OP_COND;
                err = sdmmc_send_cmd(card, &cmd);

            if (err != ESP_OK) {
                return err;
            }

            if ((MMC_R3(cmd.response) & MMC_OCR_MEM_READY)) {
                break;
            }

        }
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: send_op_cond (1) returned 0x%x", __func__, err);
        return err;
    }
    if (host_is_spi(card)) {
        err = sdmmc_send_cmd_read_ocr(card, &card->ocr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: read_ocr returned 0x%x", __func__, err);
            return err;
        }
    }
    ESP_LOGD(TAG, "host_ocr=0x%" PRIx32 " card_ocr=0x%" PRIx32, host_ocr, card->ocr);

    /* Clear all voltage bits in host's OCR which the card doesn't support.
     * Don't touch CCS bit because in SPI mode cards don't report CCS in ACMD41
     * response.
     */
    host_ocr &= (card->ocr | (~SD_OCR_VOL_MASK));
    ESP_LOGD(TAG, "sdmmc_card_init: host_ocr=%08" PRIx32 ", card_ocr=%08" PRIx32, host_ocr, card->ocr);
    return ESP_OK;
}

esp_err_t mmc_card_init(const sdmmc_host_t* config, sdmmc_card_t* card)
{
    memset(card, 0, sizeof(*card));
    memcpy(&card->host, config, sizeof(*config));
    const bool is_spi = host_is_spi(card);
    const bool always = true;
    const bool io_supported = true;

    /* Check if host flags are compatible with slot configuration. */
    SDMMC_INIT_STEP(!is_spi, sdmmc_fix_host_flags);

    /* Reset SDIO (CMD52, RES) before re-initializing IO (CMD5). */
    //SDMMC_INIT_STEP(io_supported, sdmmc_io_reset);

    /* GO_IDLE_STATE (CMD0) command resets the card */
    SDMMC_INIT_STEP(always, sdmmc_send_cmd_go_idle_state);

    SDMMC_INIT_STEP(always, sdmmc_send_cmd_go_idle_state);

    /* SEND_IF_COND (CMD8) command is used to identify SDHC/SDXC cards. */
    SDMMC_INIT_STEP(always, sdmmc_init_sd_if_cond);

    /* IO_SEND_OP_COND(CMD5), Determine if the card is an IO card. */
//    SDMMC_INIT_STEP(io_supported, sdmmc_init_io);

//    const bool is_mem = card->is_mem;
    const bool is_mem = 1;
    const bool is_sdio = !is_mem;

    /* Enable CRC16 checks for data transfers in SPI mode */
//    SDMMC_INIT_STEP(is_spi, sdmmc_init_spi_crc);

    /* Use SEND_OP_COND to set up card OCR */
    SDMMC_INIT_STEP(is_mem, mmc_init_ocr);

//    const bool is_mmc = is_mem && card->is_mmc;
    const bool is_mmc = 1;
//    const bool is_sdmem = is_mem && !is_mmc;
    const bool is_sdmem = 1;

//    ESP_LOGD(TAG, "%s: card type is %s", __func__,
//            is_sdio ? "SDIO" : is_mmc ? "MMC" : "SD");

    /* Read the contents of CID register*/
    SDMMC_INIT_STEP(is_mem, sdmmc_init_cid);

    //return ESP_ERR_TIMEOUT;

    /* Assign RCA */
    SDMMC_INIT_STEP(!is_spi, sdmmc_init_rca);

    /* Read and decode the contents of CSD register */
    //SDMMC_INIT_STEP(is_mem, sdmmc_init_csd);

    /* Decode the contents of mmc CID register */
    SDMMC_INIT_STEP(is_mmc && !is_spi, sdmmc_init_mmc_decode_cid);

    /* Switch the card from stand-by mode to data transfer mode (not needed if
     * SPI interface is used). This is needed to issue SET_BLOCKLEN and
     * SEND_SCR commands.
     */
    SDMMC_INIT_STEP(!is_spi, sdmmc_init_select_card);

    return ESP_OK;

    return ESP_ERR_TIMEOUT;

    /* SD memory cards:
     * Set block len for SDSC cards to 512 bytes (same as SDHC)
     * Read SCR
     * Wait to enter data transfer state
     */
    SDMMC_INIT_STEP(is_sdmem, sdmmc_init_sd_blocklen);
    SDMMC_INIT_STEP(is_sdmem, sdmmc_init_sd_scr);
    SDMMC_INIT_STEP(is_sdmem, sdmmc_init_sd_wait_data_ready);

    /* MMC cards: read CXD */
    SDMMC_INIT_STEP(is_mmc, sdmmc_init_mmc_read_ext_csd);

    /* Try to switch card to HS mode if the card supports it.
     * Set card->max_freq_khz value accordingly.
     */
    SDMMC_INIT_STEP(always, sdmmc_init_card_hs_mode);

    /* Set bus width. One call for every kind of card, then one for the host */
    if (!is_spi) {
        SDMMC_INIT_STEP(is_sdmem, sdmmc_init_sd_bus_width);
        SDMMC_INIT_STEP(is_sdio, sdmmc_init_io_bus_width);
        SDMMC_INIT_STEP(is_mmc, sdmmc_init_mmc_bus_width);
        SDMMC_INIT_STEP(always, sdmmc_init_host_bus_width);
    }

    /* SD card: read SD Status register */
    SDMMC_INIT_STEP(is_sdmem, sdmmc_init_sd_ssr);

    /* Switch to the host to use card->max_freq_khz frequency. */
    SDMMC_INIT_STEP(always, sdmmc_init_host_frequency);

    /* Sanity check after switching the bus mode and frequency */
    SDMMC_INIT_STEP(is_sdmem, sdmmc_check_scr);
    /* Sanity check after eMMC switch to HS mode */
    SDMMC_INIT_STEP(is_mmc, sdmmc_init_mmc_check_ext_csd);
    /* TODO: add similar checks for SDIO */

    return ESP_OK;
}

#endif

void app_main() {
    esp_err_t err;

    esp_log_level_set("*", ESP_LOG_VERBOSE);

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    host.flags = SDMMC_HOST_FLAG_1BIT;
    //config.max_freq_khz = SDMMC_FREQ_DEFAULT;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

#if 0
    slot_config.clk = 14;
    slot_config.cmd = 15;
    slot_config.d0 = 2;
    slot_config.d1 = 4;
    slot_config.d2 = 12;
    slot_config.d3 = 13;
#endif

    ESP_LOGW(TAG, "host init...");

    err = host.init();
    ESP_ERROR_CHECK(err);

    ESP_LOGW(TAG, "slot init...");
    err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_config);
    ESP_ERROR_CHECK(err);

    //err = sdmmc_host_set_card_clk(SDMMC_HOST_SLOT_1, 4000);
    //ESP_ERROR_CHECK(err);

    sdmmc_card_t *card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
    if (card == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGW(TAG, "card init...");

    for (;;) {
        if (sdmmc_card_init(&host, card) == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "slave init failed, retry...");
        vTaskDelay(100000 / portTICK_PERIOD_MS);
    }

    ESP_LOGW(TAG, "card info...");

    sdmmc_card_print_info(stdout, card);

    uint32_t status = 0;
    sdmmc_send_cmd_send_status(card, &status);

    if (status & 0x02000000){
            sdmmc_command_t cmd = {
                .opcode = MMC_SET_BLOCKLEN,
                .arg = 2,
                .flags = SCF_CMD_AC | SCF_RSP_R1
            };
            err = sdmmc_send_cmd(card, &cmd);
            ESP_ERROR_CHECK(err);

            sdmmc_command_t cmd42 = {
                .opcode = 42,
                .arg = 0,
                .flags = SCF_CMD_ADTC | SCF_RSP_R1
            };

            uint8_t data[] = { 0x8 , 0 };

            cmd42.datalen = 2;
            cmd42.blklen = 2;
            cmd42.data = data;
            cmd42.timeout_ms = 6000;

            err = sdmmc_send_cmd(card, &cmd42);            

    }

}