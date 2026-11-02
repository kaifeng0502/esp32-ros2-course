#include <string.h>
#include "driver/i2c_master.h"
#include "waiting_bitmap.h"
#include "network.h"
#include "motor.h"
#include "motor_display.h"
#include "esp_timer.h"
#include "encoder.h"
#include "encoder_protocol.h"
#include "esp_random.h"
#include "driver/usb_serial_jtag.h"
#include "esp_crc.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WIDTH 128
#define HEIGHT 64
#define COLUMN_OFFSET 2
static const char *TAG = "codex_oled";
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

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool receive_frame(char *line, i2c_master_dev_handle_t oled)
{
    uint32_t sequence, crc;
    int offset = 0;
    if (sscanf(line, "OLED1 %" SCNu32 " %" SCNx32 " %n", &sequence, &crc, &offset) != 2
        || offset == 0 || strlen(line + offset) != sizeof(frame) * 2) return false;
    // Keep malformed packets from partially changing the live framebuffer.
    static uint8_t incoming[sizeof(frame)];
    for (size_t i = 0; i < sizeof(frame); ++i) {
        int hi = hex_digit(line[offset + i * 2]);
        int lo = hex_digit(line[offset + i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        incoming[i] = (uint8_t)((hi << 4) | lo);
    }
    if (esp_crc32_le(0, incoming, sizeof(incoming)) != crc) return false;
    memcpy(frame, incoming, sizeof(frame));
    esp_err_t err = flush_frame(oled);
    char reply[64];
    int len = err == ESP_OK
        ? snprintf(reply, sizeof(reply), "ACK %" PRIu32 "\n", sequence)
        : snprintf(reply, sizeof(reply), "ERR DISPLAY %s\n", esp_err_to_name(err));
    usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
    return err == ESP_OK;
}

void app_main(void)
{
    motor_init();
    encoder_init();
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
    for (uint16_t candidate = 0x3C; candidate <= 0x3D; ++candidate) {
        esp_err_t err = i2c_master_probe(bus, candidate, 50);
        if (err == ESP_OK) { address = candidate; break; }
        if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "I2C probe error: %s", esp_err_to_name(err));
            break;
        }
    }
    i2c_master_dev_handle_t oled = NULL;
    if (address) {
        ESP_LOGI(TAG, "OLED responded at 7-bit address 0x%02X", address);
        const i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = 100000,
        };
        esp_err_t display_err = i2c_master_bus_add_device(bus, &device_config, &oled);
        if (display_err == ESP_OK) display_err = init_sh1106(oled);
        if (display_err == ESP_OK) {
            memcpy(frame, waiting_bitmap, sizeof(frame));
            display_err = flush_frame(oled);
        }
        if (display_err != ESP_OK) {
            ESP_LOGW(TAG, "OLED initialization failed (%s); continuing with USB commands.",
                     esp_err_to_name(display_err));
            if (oled) i2c_master_bus_rm_device(oled);
            oled = NULL;
        }
    } else {
        ESP_LOGW(TAG, "OLED absent; continuing with USB commands. Reconnect OLED, then reset to use it.");
    }
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 4096,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    network_init();
    const uint32_t encoder_boot_id = esp_random();
    static network_frame_t wireless_frame;
    static char line[2200];
    size_t used = 0;
    bool overflow = false;
    bool connected = false;
    bool motor_view = false;
    int64_t last_motor_frame = -200000;
    TickType_t last_frame = xTaskGetTickCount();
    while (true) {
        uint8_t chunk[128];
        int got = usb_serial_jtag_read_bytes(chunk, sizeof(chunk), pdMS_TO_TICKS(20));
        for (int i = 0; i < got; ++i) {
            if (chunk[i] == '\n') {
                line[used] = 0;
                if (!overflow && strcmp(line, "DISPLAY MOTOR") == 0) motor_view = true;
                if (!overflow && (strcmp(line, "DISPLAY STATUS") == 0 || strcmp(line, "DISPLAY CODEX") == 0)) {
                    motor_view = false;
                    connected = true;
                    last_frame = xTaskGetTickCount() - pdMS_TO_TICKS(6001);
                }
                if (!overflow && !motor_view && oled && receive_frame(line, oled)) {
                    last_frame = xTaskGetTickCount();
                    connected = true;
                }
                if (!overflow) network_command(line);
                if (!overflow && motor_command(line)) {
                    motor_view = true;
                    const char *reply = "ACK MOTOR\n";
                    usb_serial_jtag_write_bytes(reply, strlen(reply), pdMS_TO_TICKS(100));
                }
                if (!overflow && strcmp(line, "ENC?") == 0) {
                    encoder_snapshot_t counts = encoder_snapshot();
                    char reply[80];
                    int len = snprintf(reply, sizeof(reply), "ENC %" PRIu32 " %" PRIu32 " %d %d\n",
                                       counts.c1_edges, counts.c2_edges,
                                       counts.c1_level, counts.c2_level);
                    usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
                }
                if (!overflow && strcmp(line, "ENC2?") == 0) {
                    encoder_snapshot_t counts = encoder_snapshot();
                    char reply[160];
                    int len = encoder_format_state(reply, sizeof(reply), encoder_boot_id,
                                                   esp_timer_get_time(), &counts);
                    if (len > 0 && len < sizeof(reply))
                        usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
                }
                if (!overflow && strcmp(line, "LAB?") == 0) {
                    char reply[512];
                    int len = motor_lab_status(reply, sizeof(reply));
                    if (len > 0 && len < sizeof(reply))
                        usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
                }
                if (!overflow && strcmp(line, "MPC?") == 0) {
                    char reply[512];
                    int len = motor_predictive_status(reply, sizeof(reply));
                    if (len > 0 && len < sizeof(reply))
                        usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
                }
                if (!overflow && strcmp(line, "MPC2?") == 0) {
                    char reply[128];
                    int len = motor_constraint_status(reply, sizeof(reply));
                    if (len > 0 && len < sizeof(reply))
                        usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
                }
                if (!overflow && (strcmp(line, "SAMPLES?") == 0 || strcmp(line,"SAMPLES2?")==0)) {
                    static char reply[2048]; // Keep the USB batch off app_main's small task stack.
                    int len = strcmp(line,"SAMPLES2?")==0 ? motor_sample_batch2(reply,sizeof(reply)) :
                                                          motor_sample_batch(reply, sizeof(reply));
                    if (len > 0 && len < sizeof(reply))
                        usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
                }
                if (!overflow && strcmp(line, "PI?") == 0) {
                    char reply[192];
                    int len = motor_pi_status(reply, sizeof(reply));
                    usb_serial_jtag_write_bytes(reply, len, pdMS_TO_TICKS(100));
                }
                memset(line, 0, used);
                used = 0;
                overflow = false;
            } else if (!overflow && used < sizeof(line) - 1) {
                line[used++] = (char)chunk[i];
            } else {
                overflow = true;
            }
        }
        // Motor timing runs in its own task, independently of USB/OLED traffic.
        if (oled && network_take_frame(&wireless_frame)) {
            if (!motor_view) {
                memcpy(frame, wireless_frame.pixels, sizeof(frame));
                if (flush_frame(oled) == ESP_OK) {
                    network_ack(&wireless_frame);
                    last_frame = xTaskGetTickCount();
                    connected = true;
                }
            }
        }
        if (oled && motor_view && esp_timer_get_time() - last_motor_frame >= 200000) {
            motor_status_t status = motor_get_status();
            encoder_snapshot_t enc = encoder_snapshot();
            last_motor_frame = esp_timer_get_time();
            motor_display_render(frame, &status, &enc, last_motor_frame);
            esp_err_t err = flush_frame(oled);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Motor display failed (%s); continuing motor/USB control.",
                         esp_err_to_name(err));
                i2c_master_bus_rm_device(oled);
                oled = NULL;
            }
        }
        if (oled && !motor_view && connected && xTaskGetTickCount() - last_frame > pdMS_TO_TICKS(6000)) {
            memcpy(frame, waiting_bitmap, sizeof(frame));
            esp_err_t display_err = flush_frame(oled);
            if (display_err != ESP_OK) {
                ESP_LOGW(TAG, "OLED update failed: %s", esp_err_to_name(display_err));
            }
            connected = false;
        }
    }
}
