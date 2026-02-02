idf_component_register(
    SRCS "main.c" "audio_config_test.c" "ring_buffer.c" "audio_producer.c" "audio_capture_hal.c" "wifi_esp_hosted.c"
    INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/../include"
    PRIV_REQUIRES esp_timer driver esp_driver_i2s nvs_flash esp-dsp
)
