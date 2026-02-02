/*
 * Dept 5: ESP-Hosted SDIO to ESP32-C6 (Wi-Fi).
 * Bring up only after audio producer gates pass (0 capture overruns, latency within gate, 72h stress).
 *
 * Prerequisites: 0 capture overruns 10 min; fast lane within budget; e2e <= 24 ms; 72h stress passed.
 * Steps: Add ESP-Hosted (SDIO) component; configure SDIO host for ESP32-P4; init WiFi after audio_producer_start().
 * Isolation: Capture task remains highest priority; WiFi must not increase capture_overruns or break latency gate.
 *
 * Stub: no ESP-Hosted component linked yet; call wifi_esp_hosted_init() after audio stable to enable when implemented.
 */
#include "esp_log.h"

static const char *TAG = "wifi_esp_hosted";

void wifi_esp_hosted_init(void)
{
    ESP_LOGI(TAG, "Dept 5 stub: ESP-Hosted SDIO not linked; run after audio gates pass.");
}
