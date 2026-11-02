idf_component_register(SRCS "codex_oled.c" "network.c" "motor.c" "predictive.c" "encoder.c" "encoder_protocol.c" "motor_display.c"
    PRIV_REQUIRES esp_driver_i2c esp_driver_usb_serial_jtag esp_driver_gpio esp_driver_ledc esp_timer esp_hw_support
                  esp_wifi esp_netif esp_event nvs_flash lwip mbedtls
    INCLUDE_DIRS ".")
