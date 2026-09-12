#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* ============== Buttons ============== */
/* ATK-DNESP32S3-BOX0 has three buttons (levels match the xiaozhi board defs
 * in atk_dnesp32s3_box0.cc: Button(gpio, active_high)):
 *   R_BUTTON(GPIO0, active_low)  -> primary PTT (replaces the front button)
 *   L_BUTTON(GPIO3, active_low)  -> secondary (replaces the side button)
 *   M_BUTTON(GPIO4, active_high) -> long-press power on/off
 */
#define STICK_S3_PIN_BUTTON_PRIMARY   0   /* R_BUTTON, PTT, pressed = low  */
#define STICK_S3_PIN_BUTTON_SECONDARY 3   /* L_BUTTON, side, pressed = low */
#define STICK_S3_PIN_BUTTON_POWER     4   /* M_BUTTON, power, pressed = high */

/* ============== I2C (shared by ES8311 codec) ============== */
#define STICK_S3_PIN_I2C_SCL 12
#define STICK_S3_PIN_I2C_SDA 11

/* ============== ES8311 codec (I2S) ============== */
#define STICK_S3_PIN_ES8311_MCLK 13
#define STICK_S3_PIN_ES8311_BCLK 5
#define STICK_S3_PIN_ES8311_LRCK 10
/* Pin names follow the codec's perspective (same convention as the original
 * StickS3 port), which is inverted relative to the xiaozhi config.h naming:
 *   STICK_S3_PIN_ES8311_DIN  = codec serial data input  (DSDIN, MCU -> codec,
 *                              speaker path) = I2S dout = xiaozhi's I2S_DOUT = GPIO9
 *   STICK_S3_PIN_ES8311_DOUT = codec serial data output (ASDOUT, codec -> MCU,
 *                              mic path)     = I2S din  = xiaozhi's I2S_DIN  = GPIO6
 */
#define STICK_S3_PIN_ES8311_DIN  9
#define STICK_S3_PIN_ES8311_DOUT 6

/* ============== LCD (ST7789, 240x240) ============== */
#define STICK_S3_PIN_LCD_MOSI 40
#define STICK_S3_PIN_LCD_SCK  39
#define STICK_S3_PIN_LCD_DC   38
#define STICK_S3_PIN_LCD_CS   41
#define STICK_S3_PIN_LCD_RST  GPIO_NUM_NC
#define STICK_S3_PIN_LCD_BL   42

/* ============== Power / codec / speaker ============== */
#define STICK_S3_PIN_SYS_POW      2   /* system power hold: low = power off */
#define STICK_S3_PIN_CODEC_PWR    14  /* codec power enable */
#define STICK_S3_PIN_SPK_GPIO     21  /* speaker PA enable */
#define STICK_S3_PIN_CHRG         48  /* charging status input, low = charging */
#define STICK_S3_PIN_CHG_CTRL     47  /* battery ADC measurement enable */
#define STICK_S3_PIN_BAT_VSEN     1   /* battery voltage sense (ADC1_CH0) */

/* M_BUTTON is active-high, so the deep-sleep fallback wakes on a high level */
#define STICK_S3_POWER_BUTTON_PRESSED_LEVEL 1

/* Legacy aliases used by main.c and ui_status.c */
#define STICK_S3_PIN_BUTTON_FRONT  STICK_S3_PIN_BUTTON_PRIMARY
#define STICK_S3_PIN_BUTTON_SIDE   STICK_S3_PIN_BUTTON_SECONDARY

/* ============== Board-level API ============== */
esp_err_t stick_s3_board_init(void);
i2c_master_bus_handle_t stick_s3_board_i2c_bus(void);

/* Battery / power: ATK board has no M5PM1 PMIC, values come from
 * the ADC divider on BAT_VSEN plus the CHRG status pin. */
esp_err_t stick_s3_board_battery_level(int *level_percent);
esp_err_t stick_s3_board_battery_charging(bool *charging);
esp_err_t stick_s3_board_usb_powered(bool *usb_powered);

/* Power-off sequence shared by the power key and low-battery shutdown */
void stick_s3_board_power_off(void);

/* Legacy stubs — ATK board has no PMIC interrupt controller */
esp_err_t stick_s3_board_clear_power_irqs(uint8_t *sys_status);
void stick_s3_board_prepare_deep_sleep(void);

bool stick_s3_front_button_pressed(void);
bool stick_s3_side_button_pressed(void);
