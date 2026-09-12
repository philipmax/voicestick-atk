#include "stick_s3_board.h"

#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "stick_s3_board";

/* ============== I2C bus (shared by ES8311) ============== */
#define STICK_S3_I2C_FREQ_HZ 100000

static i2c_master_bus_handle_t s_i2c_bus;
static bool s_i2c_inited;

/* ============== Battery ADC ============== */
/* ATK-DNESP32S3-BOX0 wiring (see xiaozhi atk-dnesp32s3-box0):
 *   BAT_VSEN  GPIO1  -> ADC1 channel 0
 *   CHG_CTRL  GPIO47 -> measurement enable (drive low while sampling)
 *   CHRG      GPIO48 -> charging status, low = charging
 *
 * The ADC pin is normally held by the charge-measurement network, so a read
 * only makes sense while CHG_CTRL is low. Levels below are the calibration
 * points used by the upstream PowerManager.
 */
#define STICK_S3_BATTERY_ADC_UNIT    ADC_UNIT_1
#define STICK_S3_BATTERY_ADC_CHANNEL ADC_CHANNEL_0
#define STICK_S3_BATTERY_SAMPLE_COUNT 10
/* CHG_CTRL also gates the battery feed, so keep the low pulse short when the
 * board runs from battery alone. */
#define STICK_S3_BATTERY_SETTLE_MS    20
/* The upstream PowerManager averages the last three measurements before
 * mapping them to a percentage; a single shot read makes the gauge jump. */
#define STICK_S3_BATTERY_WINDOW       3

static adc_oneshot_unit_handle_t s_adc_handle;
static bool s_adc_inited;

static uint16_t s_adc_window[STICK_S3_BATTERY_WINDOW];
static uint8_t s_adc_window_count;
static uint8_t s_adc_window_head;

typedef struct {
    uint16_t adc;
    uint8_t level;
} battery_level_point_t;

static const battery_level_point_t s_battery_levels[] = {
    {2951, 0},   /* 3.80V */
    {3019, 20},
    {3037, 40},
    {3091, 60},  /* 3.88V */
    {3124, 80},
    {3231, 100},
};
#define BATTERY_LEVEL_POINT_COUNT \
    (sizeof(s_battery_levels) / sizeof(s_battery_levels[0]))

/* ============== helpers ============== */
static esp_err_t init_i2c(void)
{
    if (s_i2c_inited) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = STICK_S3_PIN_I2C_SDA,
        .scl_io_num = STICK_S3_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c new master bus failed: %s", esp_err_to_name(err));
        return err;
    }

    s_i2c_inited = true;
    ESP_LOGI(TAG, "I2C init ok SDA=%d SCL=%d",
             STICK_S3_PIN_I2C_SDA, STICK_S3_PIN_I2C_SCL);
    return ESP_OK;
}

static esp_err_t init_adc(void)
{
    if (s_adc_inited) {
        return ESP_OK;
    }

    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = STICK_S3_BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle),
                        TAG, "adc oneshot unit");

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(s_adc_handle, STICK_S3_BATTERY_ADC_CHANNEL, &chan_cfg),
        TAG, "adc oneshot channel");

    s_adc_inited = true;
    ESP_LOGI(TAG, "ADC init ok channel=%d", STICK_S3_BATTERY_ADC_CHANNEL);
    return ESP_OK;
}

/*! Read the battery ADC value with the measurement path enabled, then fold it
 *  into the moving average that feeds the level mapping. */
static esp_err_t read_battery_adc(int *adc_out)
{
    if (!adc_out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_adc_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(STICK_S3_PIN_CHG_CTRL, 0), TAG, "chg_ctrl low");
    vTaskDelay(pdMS_TO_TICKS(STICK_S3_BATTERY_SETTLE_MS));

    uint32_t sum = 0;
    uint32_t samples = 0;
    for (int i = 0; i < STICK_S3_BATTERY_SAMPLE_COUNT; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc_handle, STICK_S3_BATTERY_ADC_CHANNEL, &raw) == ESP_OK) {
            sum += (uint32_t)raw;
            samples++;
        }
    }

    (void)gpio_set_level(STICK_S3_PIN_CHG_CTRL, 1);
    vTaskDelay(pdMS_TO_TICKS(STICK_S3_BATTERY_SETTLE_MS));

    if (samples == 0) {
        return ESP_FAIL;
    }

    const int single = (int)(sum / samples);
    s_adc_window[s_adc_window_head] = (uint16_t)single;
    s_adc_window_head = (uint8_t)((s_adc_window_head + 1) % STICK_S3_BATTERY_WINDOW);
    if (s_adc_window_count < STICK_S3_BATTERY_WINDOW) {
        s_adc_window_count++;
    }

    uint32_t window_sum = 0;
    for (uint8_t i = 0; i < s_adc_window_count; i++) {
        window_sum += s_adc_window[i];
    }
    const int averaged = (int)(window_sum / s_adc_window_count);

    ESP_LOGI(TAG, "battery adc single=%d avg=%d samples=%" PRIu32,
             single, averaged, samples);

    *adc_out = averaged;
    return ESP_OK;
}

/*! Map an averaged ADC value to a battery percentage (piecewise linear). */
static uint8_t adc_to_level(int adc)
{
    if (adc < s_battery_levels[0].adc) {
        return 0;
    }
    const uint8_t last = BATTERY_LEVEL_POINT_COUNT - 1;
    if (adc >= s_battery_levels[last].adc) {
        return 100;
    }
    for (uint8_t i = 0; i < last; i++) {
        const battery_level_point_t *lo = &s_battery_levels[i];
        const battery_level_point_t *hi = &s_battery_levels[i + 1];
        if (adc >= lo->adc && adc < hi->adc) {
            const float ratio = (float)(adc - lo->adc) / (float)(hi->adc - lo->adc);
            return (uint8_t)(lo->level + ratio * (hi->level - lo->level));
        }
    }
    return 100;
}

/* ============== board init ============== */
esp_err_t stick_s3_board_init(void)
{
    /* --- 1. I2C (shared bus for ES8311 codec) --- */
    esp_err_t err = init_i2c();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed: %s (codec will not work)", esp_err_to_name(err));
    }

    /* --- 2. Power / codec / speaker control outputs --- */
    /* SYS_POW and CODEC_PWR are driven the way the upstream board code does
     * it: input+output with the internal pull-up enabled. */
    const gpio_config_t hold_cfg = {
        .pin_bit_mask = (1ULL << STICK_S3_PIN_SYS_POW) |
                        (1ULL << STICK_S3_PIN_CODEC_PWR),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&hold_cfg), TAG, "power hold gpio");

    const gpio_config_t chg_ctrl_cfg = {
        .pin_bit_mask = 1ULL << STICK_S3_PIN_CHG_CTRL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&chg_ctrl_cfg), TAG, "chg ctrl gpio");

    /* Keep the system powered (SYS_POW low would cut the rail) */
    ESP_RETURN_ON_ERROR(gpio_set_level(STICK_S3_PIN_SYS_POW, 1), TAG, "sys_pow high");
    ESP_RETURN_ON_ERROR(gpio_set_level(STICK_S3_PIN_CODEC_PWR, 1), TAG, "codec_pwr high");
    /* Battery measurement path idle */
    ESP_RETURN_ON_ERROR(gpio_set_level(STICK_S3_PIN_CHG_CTRL, 1), TAG, "chg_ctrl high");

    const gpio_config_t spk_cfg = {
        .pin_bit_mask = 1ULL << STICK_S3_PIN_SPK_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&spk_cfg), TAG, "spk gpio");
    ESP_RETURN_ON_ERROR(gpio_set_level(STICK_S3_PIN_SPK_GPIO, 0), TAG, "spk off");

    /* --- 3. Charging status input (open-drain on the PMIC, low = charging) ---
     * Button GPIOs are intentionally left alone: the iot_button component
     * configures their direction and pull according to the active level. */
    const gpio_config_t chrg_cfg = {
        .pin_bit_mask = 1ULL << STICK_S3_PIN_CHRG,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&chrg_cfg), TAG, "chrg gpio");

    /* --- 4. Battery ADC --- */
    err = init_adc();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC init failed: %s (battery level unavailable)", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "board init ok");
    return ESP_OK;
}

/* ============== public API ============== */
i2c_master_bus_handle_t stick_s3_board_i2c_bus(void)
{
    return s_i2c_bus;
}

esp_err_t stick_s3_board_battery_level(int *level_percent)
{
    if (!level_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    int adc = 0;
    ESP_RETURN_ON_ERROR(read_battery_adc(&adc), TAG, "battery adc read");

    *level_percent = adc_to_level(adc);
    ESP_LOGD(TAG, "battery adc=%d level=%d", adc, *level_percent);
    return ESP_OK;
}

esp_err_t stick_s3_board_battery_charging(bool *charging)
{
    if (!charging) {
        return ESP_ERR_INVALID_ARG;
    }
    /* CHRG is pulled low by the charger while a charge is in progress */
    *charging = (gpio_get_level(STICK_S3_PIN_CHRG) == 0);
    return ESP_OK;
}

esp_err_t stick_s3_board_usb_powered(bool *usb_powered)
{
    if (!usb_powered) {
        return ESP_ERR_INVALID_ARG;
    }
    /* The board does not expose VBUS separately; charging implies USB power */
    bool charging = false;
    esp_err_t err = stick_s3_board_battery_charging(&charging);
    *usb_powered = charging;
    return err;
}

void stick_s3_board_power_off(void)
{
    ESP_LOGW(TAG, "power off: releasing SYS_POW");

    /* Stop the battery measurement path first, then release the power hold */
    (void)gpio_set_level(STICK_S3_PIN_CHG_CTRL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    (void)gpio_set_level(STICK_S3_PIN_SYS_POW, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

esp_err_t stick_s3_board_clear_power_irqs(uint8_t *sys_status)
{
    if (sys_status) {
        *sys_status = 0;
    }
    /* No PMIC interrupt controller on this board */
    return ESP_OK;
}

void stick_s3_board_prepare_deep_sleep(void)
{
    (void)gpio_set_level(STICK_S3_PIN_CODEC_PWR, 0);
    (void)gpio_set_level(STICK_S3_PIN_SPK_GPIO, 0);
    /* SYS_POW stays high: deep sleep is a software state, not a power cut */
}

bool stick_s3_front_button_pressed(void)
{
    return gpio_get_level(STICK_S3_PIN_BUTTON_PRIMARY) == 0;
}

bool stick_s3_side_button_pressed(void)
{
    return gpio_get_level(STICK_S3_PIN_BUTTON_SECONDARY) == 0;
}
