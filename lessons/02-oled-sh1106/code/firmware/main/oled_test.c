#include <string.h>
#include "driver/i2c_master.h"
#include "message_bitmap.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WIDTH 128
#define HEIGHT 64
#define COLUMN_OFFSET 2
static const char *TAG = "oled_test";
static uint8_t frame[WIDTH * HEIGHT / 8];

static esp_err_t flush_frame(i2c_master_dev_handle_t oled)
{
    for (unsigned page = 0; page < HEIGHT / 8; ++page) {
        const uint8_t position[] = {
            0x00, (uint8_t)(0xB0 | page),
            COLUMN_OFFSET & 0x0F, 0x10 | (COLUMN_OFFSET >> 4),
        };
        esp_err_t err = i2c_master_transmit(oled, position, sizeof(position), 100);
        if (err != ESP_OK) return err;
        uint8_t payload[WIDTH + 1];
        payload[0] = 0x40; // Data control byte.
        memcpy(payload + 1, frame + page * WIDTH, WIDTH);
        err = i2c_master_transmit(oled, payload, sizeof(payload), 100);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t init_sh1106(i2c_master_dev_handle_t oled)
{
    // Register configuration and 2-column offset checked against:
    // https://github.com/adafruit/Adafruit_SH110x/blob/master/Adafruit_SH1106G.cpp
    const uint8_t init[] = {
        0x00,        // Command control byte.
        0xAE,        // Display off.
        0xD5, 0x80,  // Display clock.
        0xA8, 0x3F,  // 64 rows.
        0xD3, 0x00,  // Display offset.
        0x40,        // Start line 0.
        0xAD, 0x8B,  // SH1106 internal DC/DC enabled.
        0xA1, 0xC8,  // Segment and COM scan direction.
        0xDA, 0x12,  // COM pin configuration.
        0x81, 0x7F,  // Moderate contrast.
        0xD9, 0x1F,  // Precharge period.
        0xDB, 0x40,  // VCOM level.
        0x33,        // Internal panel pump voltage selection.
        0xA6, 0xA4,  // Normal polarity; display RAM contents.
    };
    esp_err_t err = i2c_master_transmit(oled, init, sizeof(init), 100);
    if (err != ESP_OK) return err;
    memset(frame, 0, sizeof(frame));
    err = flush_frame(oled);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));
    const uint8_t on[] = {0x00, 0xAF};
    return i2c_master_transmit(oled, on, sizeof(on), 100);
}

void app_main(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
    ESP_LOGI(TAG, "SH1106 128x64: SDA=GPIO8, SCL=GPIO9, I2C=100kHz");
    // The user confirmed the controller model; probing only discovers the address.
    uint16_t address = 0;
    while (!address) {
        for (uint16_t candidate = 0x3C; candidate <= 0x3D; ++candidate) {
            esp_err_t err = i2c_master_probe(bus, candidate, 50);
            if (err == ESP_OK) { address = candidate; break; }
            if (err != ESP_ERR_NOT_FOUND) {
                ESP_LOGE(TAG, "I2C probe error: %s", esp_err_to_name(err));
                break;
            }
        }
        if (!address) {
            ESP_LOGW(TAG, "OLED not responding. Disconnect USB before checking wiring; retrying in 5 seconds.");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    ESP_LOGI(TAG, "OLED responded at 7-bit address 0x%02X", address);
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t oled;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &oled));
    ESP_ERROR_CHECK(init_sh1106(oled));
    memcpy(frame, message_bitmap, sizeof(frame));
    ESP_ERROR_CHECK(flush_frame(oled));
    ESP_LOGI(TAG, "Message sent: Hello OLED / ESP32 + ROS2");
    // The display retains its RAM contents without repeated transfers.
}
