/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "st7796s.h"

static const char* TAG = "lcd_panel.st7796u";

static esp_err_t panel_st7796u_del(esp_lcd_panel_t* panel);
static esp_err_t panel_st7796u_reset(esp_lcd_panel_t* panel);
static esp_err_t panel_st7796u_init(esp_lcd_panel_t* panel);
static esp_err_t panel_st7796u_draw_bitmap(esp_lcd_panel_t* panel, int x_start,
                                           int y_start, int x_end, int y_end,
                                           const void* color_data);
static esp_err_t panel_st7796u_invert_color(esp_lcd_panel_t* panel,
                                            bool invert_color_data);
static esp_err_t panel_st7796u_mirror(esp_lcd_panel_t* panel, bool mirror_x,
                                      bool mirror_y);
static esp_err_t panel_st7796u_swap_xy(esp_lcd_panel_t* panel, bool swap_axes);
static esp_err_t panel_st7796u_set_gap(esp_lcd_panel_t* panel, int x_gap,
                                       int y_gap);
static esp_err_t panel_st7796u_disp_on_off(esp_lcd_panel_t* panel, bool on_off);

typedef struct {
  esp_lcd_panel_t base;
  esp_lcd_panel_io_handle_t io;
  int reset_gpio_num;
  bool reset_level;
  int x_gap;
  int y_gap;
  uint8_t fb_bits_per_pixel;
  uint8_t madctl_val;  // save current value of LCD_CMD_MADCTL register
  uint8_t colmod_cal;  // save surrent value of LCD_CMD_COLMOD register
} st7796u_panel_t;

esp_err_t esp_lcd_new_panel_st7796u(
    const esp_lcd_panel_io_handle_t io,
    const esp_lcd_panel_dev_config_t* panel_dev_config,
    esp_lcd_panel_handle_t* ret_panel)
{
  esp_err_t ret = ESP_OK;
  st7796u_panel_t* st7796u = NULL;
  ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG,
                    err, TAG, "invalid argument");
  st7796u = calloc(1, sizeof(st7796u_panel_t));
  ESP_GOTO_ON_FALSE(st7796u, ESP_ERR_NO_MEM, err, TAG,
                    "no mem for st7796u panel");

  if (panel_dev_config->reset_gpio_num >= 0) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG,
                      "configure GPIO for RST line failed");
  }

  switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
      st7796u->madctl_val = 0;
      break;
    case ESP_LCD_COLOR_SPACE_BGR:
      st7796u->madctl_val |= LCD_CMD_BGR_BIT;
      break;
    default:
      ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                        "unsupported color space");
      break;
  }

  uint8_t fb_bits_per_pixel = 0;
  switch (panel_dev_config->bits_per_pixel) {
    case 16:  // RGB565
      st7796u->colmod_cal = 0x55;
      fb_bits_per_pixel = 16;
      break;
    case 18:  // RGB666
      st7796u->colmod_cal = 0x66;
      // each color component (R/G/B) should occupy the 6 high bits of a byte,
      // which means 3 full bytes are required for a pixel
      fb_bits_per_pixel = 24;
      break;
    default:
      ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                        "unsupported pixel width");
      break;
  }

  st7796u->io = io;
  st7796u->fb_bits_per_pixel = fb_bits_per_pixel;
  st7796u->reset_gpio_num = panel_dev_config->reset_gpio_num;
  st7796u->reset_level = panel_dev_config->flags.reset_active_high;
  st7796u->base.del = panel_st7796u_del;
  st7796u->base.reset = panel_st7796u_reset;
  st7796u->base.init = panel_st7796u_init;
  st7796u->base.draw_bitmap = panel_st7796u_draw_bitmap;
  st7796u->base.invert_color = panel_st7796u_invert_color;
  st7796u->base.set_gap = panel_st7796u_set_gap;
  st7796u->base.mirror = panel_st7796u_mirror;
  st7796u->base.swap_xy = panel_st7796u_swap_xy;
  st7796u->base.disp_on_off = panel_st7796u_disp_on_off;
  *ret_panel = &(st7796u->base);
  ESP_LOGD(TAG, "new st7796u panel @%p", st7796u);

  return ESP_OK;

err:
  if (st7796u) {
    if (panel_dev_config->reset_gpio_num >= 0) {
      gpio_reset_pin(panel_dev_config->reset_gpio_num);
    }
    free(st7796u);
  }
  return ret;
}

static esp_err_t panel_st7796u_del(esp_lcd_panel_t* panel)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);

  if (st7796u->reset_gpio_num >= 0) {
    gpio_reset_pin(st7796u->reset_gpio_num);
  }
  ESP_LOGD(TAG, "del st7796u panel @%p", st7796u);
  free(st7796u);
  return ESP_OK;
}

static esp_err_t panel_st7796u_reset(esp_lcd_panel_t* panel)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  esp_lcd_panel_io_handle_t io = st7796u->io;

  // perform hardware reset
  if (st7796u->reset_gpio_num >= 0) {
    gpio_set_level(st7796u->reset_gpio_num, st7796u->reset_level);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(st7796u->reset_gpio_num, !st7796u->reset_level);
    vTaskDelay(pdMS_TO_TICKS(10));
  } else {  // perform software reset
    esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(
        20));  // spec, wait at least 5m before sending new command
  }

  return ESP_OK;
}

typedef struct {
  uint8_t cmd;
  uint8_t data[16];
  uint8_t
      data_bytes;  // Length of data in above data array; 0xFF = end of cmds.
} lcd_init_cmd_t;

static const lcd_init_cmd_t vendor_specific_init[] = {
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},       /*Power control*/
    {0xC1, {0x11}, 1},       /*Power control */
    {0xC5, {0x35, 0x3E}, 2}, /*VCOM control*/
    {0xC7, {0xBE}, 1},       /*VCOM control*/
    {0x36, {0x28}, 1},       /*Memory Access Control*/
    {0x3A, {0x55}, 1},       /*Pixel Format Set*/
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0,
     {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02,
      0x07, 0x05, 0x00},
     15},
    {0XE1,
     {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D,
      0x38, 0x3A, 0x1F},
     15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

static esp_err_t panel_st7796u_init(esp_lcd_panel_t* panel)
{
  ESP_LOGI(TAG, "init st7796u.");
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  esp_lcd_panel_io_handle_t io = st7796u->io;

  // LCD goes into sleep mode and display will be turned off after power on
  // reset, exit sleep mode first
  esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL,
                            (uint8_t[]){
                                st7796u->madctl_val,
                            },
                            1);
  esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD,
                            (uint8_t[]){
                                st7796u->colmod_cal,
                            },
                            1);

  // vendor specific initialization, it can be different between manufacturers
  // should consult the LCD supplier for initialization sequence code
  int cmd = 0;
  while (vendor_specific_init[cmd].data_bytes != 0xff) {
    esp_lcd_panel_io_tx_param(io, vendor_specific_init[cmd].cmd,
                              vendor_specific_init[cmd].data,
                              vendor_specific_init[cmd].data_bytes & 0x1F);
    cmd++;
  }

  panel_st7796u_swap_xy(panel, true);
  panel_st7796u_mirror(panel, true, true);
  return ESP_OK;
}

static esp_err_t panel_st7796u_draw_bitmap(esp_lcd_panel_t* panel, int x_start,
                                           int y_start, int x_end, int y_end,
                                           const void* color_data)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  assert((x_start < x_end) && (y_start < y_end) &&
         "start position must be smaller than end position");
  esp_lcd_panel_io_handle_t io = st7796u->io;

  x_start += st7796u->x_gap;
  x_end += st7796u->x_gap;
  y_start += st7796u->y_gap;
  y_end += st7796u->y_gap;

  // define an area of frame memory where MCU can access
  esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET,
                            (uint8_t[]){
                                (x_start >> 8) & 0xFF,
                                x_start & 0xFF,
                                ((x_end - 1) >> 8) & 0xFF,
                                (x_end - 1) & 0xFF,
                            },
                            4);
  esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET,
                            (uint8_t[]){
                                (y_start >> 8) & 0xFF,
                                y_start & 0xFF,
                                ((y_end - 1) >> 8) & 0xFF,
                                (y_end - 1) & 0xFF,
                            },
                            4);
  // transfer frame buffer
  size_t len =
      (x_end - x_start) * (y_end - y_start) * st7796u->fb_bits_per_pixel / 8;
  esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

  return ESP_OK;
}

static esp_err_t panel_st7796u_invert_color(esp_lcd_panel_t* panel,
                                            bool invert_color_data)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  esp_lcd_panel_io_handle_t io = st7796u->io;
  int command = 0;
  if (invert_color_data) {
    command = LCD_CMD_INVON;
  } else {
    command = LCD_CMD_INVOFF;
  }
  esp_lcd_panel_io_tx_param(io, command, NULL, 0);
  return ESP_OK;
}

static esp_err_t panel_st7796u_mirror(esp_lcd_panel_t* panel, bool mirror_x,
                                      bool mirror_y)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  esp_lcd_panel_io_handle_t io = st7796u->io;
  if (mirror_x) {
    st7796u->madctl_val |= LCD_CMD_MX_BIT;
  } else {
    st7796u->madctl_val &= ~LCD_CMD_MX_BIT;
  }
  if (mirror_y) {
    st7796u->madctl_val |= LCD_CMD_MY_BIT;
  } else {
    st7796u->madctl_val &= ~LCD_CMD_MY_BIT;
  }
  esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL,
                            (uint8_t[]){st7796u->madctl_val}, 1);
  return ESP_OK;
}

static esp_err_t panel_st7796u_swap_xy(esp_lcd_panel_t* panel, bool swap_axes)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  esp_lcd_panel_io_handle_t io = st7796u->io;
  if (swap_axes) {
    st7796u->madctl_val |= LCD_CMD_MV_BIT;
  } else {
    st7796u->madctl_val &= ~LCD_CMD_MV_BIT;
  }
  esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL,
                            (uint8_t[]){st7796u->madctl_val}, 1);
  return ESP_OK;
}

static esp_err_t panel_st7796u_set_gap(esp_lcd_panel_t* panel, int x_gap,
                                       int y_gap)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  st7796u->x_gap = x_gap;
  st7796u->y_gap = y_gap;
  return ESP_OK;
}

static esp_err_t panel_st7796u_disp_on_off(esp_lcd_panel_t* panel, bool on_off)
{
  st7796u_panel_t* st7796u = __containerof(panel, st7796u_panel_t, base);
  esp_lcd_panel_io_handle_t io = st7796u->io;
  int command = 0;
  if (!on_off) {
    command = LCD_CMD_DISPOFF;
  } else {
    command = LCD_CMD_DISPON;
  }
  esp_lcd_panel_io_tx_param(io, command, NULL, 0);
  return ESP_OK;
}
