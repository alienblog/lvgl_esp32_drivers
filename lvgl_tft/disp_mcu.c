// simple MCU interface driver for the displays.
// Mostly from ESP examples/peripherals/lcd

static const char* TAG = "lcd-mcu";

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#include "lvgl_helpers.h"
#include "disp_mcu.h"

#define MCU_LCD_PIXEL_CLOCK_HZ (10 * 1000 * 1000)

static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_i80_bus_config_t bus_config = {
    .dc_gpio_num = CONFIG_LV_TFT_MCU_DC_NUM,
    .wr_gpio_num = CONFIG_LV_TFT_MCU_WR_NUM,
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .data_gpio_nums = {
        CONFIG_LV_TFT_MCU_DATA0,
        CONFIG_LV_TFT_MCU_DATA1,
        CONFIG_LV_TFT_MCU_DATA2,
        CONFIG_LV_TFT_MCU_DATA3,
        CONFIG_LV_TFT_MCU_DATA4,
        CONFIG_LV_TFT_MCU_DATA5,
        CONFIG_LV_TFT_MCU_DATA6,
        CONFIG_LV_TFT_MCU_DATA7,
#if CONFIG_LV_TFT_MCU_BUS_WIDTH_16
        CONFIG_LV_TFT_MCU_DATA8,
        CONFIG_LV_TFT_MCU_DATA9,
        CONFIG_LV_TFT_MCU_DATA10,
        CONFIG_LV_TFT_MCU_DATA11,
        CONFIG_LV_TFT_MCU_DATA12,
        CONFIG_LV_TFT_MCU_DATA13,
        CONFIG_LV_TFT_MCU_DATA14,
        CONFIG_LV_TFT_MCU_DATA15,
#endif
    },
#if CONFIG_LV_TFT_MCU_BUS_WIDTH_16
    .bus_width = 16,
#else
    .bus_width = 8,
#endif
    .max_transfer_bytes = DISP_BUF_SIZE * sizeof(lv_color_t),
    .psram_trans_align = 64,
    .sram_trans_align = 4,
};

static bool mcu_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx);

static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = CONFIG_LV_TFT_MCU_CS_NUM,
    .pclk_hz = MCU_LCD_PIXEL_CLOCK_HZ,
    .trans_queue_depth = 10,
    .dc_levels = {
        .dc_idle_level = 0,
        .dc_cmd_level = 0,
        .dc_dummy_level = 0,
        .dc_data_level = 1,
    },
    .flags = {
        .cs_active_high = 0,
        .reverse_color_bits = 0,
        .swap_color_bytes = 1,
        .pclk_active_neg = 0,
        .pclk_idle_low = 0,
    },
    .on_color_trans_done = mcu_notify_lvgl_flush_ready,
    .user_ctx = NULL,
#if CONFIG_LV_TFT_LCD_CMD_WIDTH_16
    .lcd_cmd_bits = 16,
#else
    .lcd_cmd_bits = 8,
#endif
#if CONFIG_LV_TFT_LCD_PARAM_WIDTH_16
    .lcd_param_bits = 16,
#else
    .lcd_param_bits = 8,
#endif
};

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = CONFIG_LV_TFT_MCU_RST_NUM,
    .rgb_endian = LCD_RGB_ENDIAN_BGR,
    .bits_per_pixel = sizeof(lv_color_t) * 8,
};

static bool mcu_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    lv_disp_drv_t* disp_driver = (lv_disp_drv_t*)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

void lcd_mcu_lvgl_flush_cb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_map) {
    // use the static one esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_HX8357
extern esp_err_t esp_lcd_new_panel_hx8357(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t* panel_dev_config, esp_lcd_panel_handle_t* ret_panel);
#endif

void disp_mcu_panel_init(lv_disp_drv_t* disp_drv) {
    ESP_LOGI(TAG, "Initialize Intel 8080 bus");

    io_config.user_ctx = disp_drv;  // for the flush_ready cb

    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_HX8357
    ESP_LOGI(TAG, "Lcd Panel New");
    ESP_ERROR_CHECK(esp_lcd_new_panel_hx8357(io_handle, &panel_config, &panel_handle));
#endif
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    // esp_lcd_panel_set_gap(panel_handle, 0, 20);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}