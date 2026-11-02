#include <stdint.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "encoder.h"

// Motor encoder board: green C1 -> GPIO11, yellow C2 -> GPIO10.
#define ENCODER_C1 GPIO_NUM_11
#define ENCODER_C2 GPIO_NUM_10

static const char *TAG = "encoder";
static portMUX_TYPE counter_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t c1_edges;
static volatile uint32_t c2_edges;
static volatile int64_t c1_signed;
static volatile int64_t c2_signed;

static void c1_isr(void *arg)
{
    portENTER_CRITICAL_ISR(&counter_lock);
    ++c1_edges;
    // Decode both edges of each channel: 1400 signed edges/rev/channel.
    c1_signed += gpio_get_level(ENCODER_C1) == gpio_get_level(ENCODER_C2) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&counter_lock);
}

static void c2_isr(void *arg)
{
    portENTER_CRITICAL_ISR(&counter_lock);
    ++c2_edges;
    c2_signed += gpio_get_level(ENCODER_C1) != gpio_get_level(ENCODER_C2) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&counter_lock);
}

void encoder_init(void)
{
    const gpio_config_t pins = {
        .pin_bit_mask = (1ULL << ENCODER_C1) | (1ULL << ENCODER_C2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&pins));
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_C1, c1_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_C2, c2_isr, NULL));
    ESP_LOGI(TAG, "Encoder ready: C1=GPIO11, C2=GPIO10");
}

encoder_snapshot_t encoder_snapshot(void)
{
    encoder_snapshot_t result;
    portENTER_CRITICAL(&counter_lock);
    result.c1_edges = c1_edges;
    result.c2_edges = c2_edges;
    result.c1_signed = c1_signed;
    result.c2_signed = c2_signed;
    portEXIT_CRITICAL(&counter_lock);
    result.c1_level = gpio_get_level(ENCODER_C1);
    result.c2_level = gpio_get_level(ENCODER_C2);
    return result;
}
