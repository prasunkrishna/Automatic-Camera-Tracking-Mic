// anchor1_ftm_responder_main.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#define TAG "ANCHOR1"

// ---------- Config ----------
#define ANCHOR_SSID     "ANCHOR1"
#define ANCHOR_PASS     "12345678"   // >=8 chars; change if you want
#define ANCHOR_CHANNEL  6
//--------------------------------

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "SoftAP started.");
            break;
        case WIFI_EVENT_AP_STOP:
            ESP_LOGW(TAG, "SoftAP stopped.");
            break;
        default:
            break;
        }
    }
}

static void wifi_init_softap(void)
{
    // netif
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();   // brings up AP interface + DHCP

    // driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL, NULL));

    // config AP
    wifi_config_t wifi_config = { 0 };
    strlcpy((char *)wifi_config.ap.ssid, ANCHOR_SSID, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, ANCHOR_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len      = strlen(ANCHOR_SSID);
    wifi_config.ap.channel       = ANCHOR_CHANNEL;
    wifi_config.ap.max_connection= 4;
    wifi_config.ap.authmode      = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.ftm_responder = true;         // <<< enable FTM responder support

    if (strlen(ANCHOR_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // log MAC (BSSID)
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    ESP_LOGI(TAG, "SoftAP '%s' up. Channel=%d  Password=%s", ANCHOR_SSID, ANCHOR_CHANNEL,
             strlen(ANCHOR_PASS) ? ANCHOR_PASS : "<open>");
    ESP_LOGI(TAG, "AP MAC: " MACSTR, MAC2STR(mac));
}

void app_main(void)
{
    // robust NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();

    // Idle forever; Wi-Fi driver handles FTM responder.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
