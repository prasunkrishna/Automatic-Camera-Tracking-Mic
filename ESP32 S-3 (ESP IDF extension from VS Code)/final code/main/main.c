#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "driver/i2s.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

// ------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------
#define WIFI_SSID       "main"
#define WIFI_PASS       "12345678"

// FTM anchor MACs
static const uint8_t ANCHOR1_MAC[6] = {0x50, 0x78, 0x7D, 0x18, 0x7D, 0x91};
static const uint8_t ANCHOR2_MAC[6] = {0x28, 0x37, 0x2F, 0xF8, 0x19, 0xF5};

// FTM settings
#define FTM_CHANNEL      6
#define FTM_FRM_COUNT    64
#define FTM_BURST_PERIOD 10

// HTTP server (ESP8266)
#define ESP8266_SERVER_IP    "192.168.4.1"
#define ESP8266_SERVER_PORT  80
#define ESP8266_ENDPOINT     "/update"

// UDP settings
#define UDP_TARGET_IP     "192.168.4.2"
#define UDP_TARGET_PORT   4210
#define SENDER_ID         1

#define SAMPLE_RATE_HZ      16000
#define PACKET_MS           30
#define SAMPLES_PER_PACKET  ((SAMPLE_RATE_HZ * PACKET_MS) / 1000)

// I2S pin config (INMP441)
#define I2S_BCLK_PIN     8
#define I2S_LRCLK_PIN    9
#define I2S_DATA_IN_PIN  6

static const char *TAG = "FTM_I2S_COMBO";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_ftm_event_group;
#define FTM_DONE_BIT BIT0
#define FTM_FAIL_BIT BIT1

static int32_t s_anchor1_dist_cm = -1;
static int32_t s_anchor2_dist_cm = -1;
static volatile int32_t s_last_rtt_ns = -1;
static volatile int32_t s_last_dist_cm = -1;

static int udp_sock = -1;
static struct sockaddr_in udp_dest_addr;
static int32_t i2s_buf[SAMPLES_PER_PACKET];
static int16_t pcm_buf[SAMPLES_PER_PACKET];
static uint8_t udp_packet[1 + SAMPLES_PER_PACKET * 2];

// ------------------------------------------------------------------
// Wi-Fi + Event Handler
// ------------------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_config);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t sta_config = {0};
    strcpy((char *)sta_config.sta.ssid, WIFI_SSID);
    strcpy((char *)sta_config.sta.password, WIFI_PASS);
    sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    esp_wifi_start();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "Wi-Fi connect timeout");
    }
}

// ------------------------------------------------------------------
// I2S Init
// ------------------------------------------------------------------
static void i2s_init(void)
{
    i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE_HZ,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 6,
        .dma_buf_len = 256,
        .use_apll = false
    };

    i2s_pin_config_t pin_cfg = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRCLK_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATA_IN_PIN
    };

    i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_cfg);
}

// ------------------------------------------------------------------
// UDP Init
// ------------------------------------------------------------------
static int udp_init(void)
{
    udp_dest_addr.sin_addr.s_addr = inet_addr(UDP_TARGET_IP);
    udp_dest_addr.sin_family = AF_INET;
    udp_dest_addr.sin_port = htons(UDP_TARGET_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket failed");
    }
    return sock;
}

static inline int16_t i2s_24_to_pcm16(int32_t s)
{
    int32_t shifted = s >> 14;
    if (shifted > 32767) shifted = 32767;
    if (shifted < -32768) shifted = -32768;
    return (int16_t)shifted;
}

// ------------------------------------------------------------------
// FTM Handlers
// ------------------------------------------------------------------
static void ftm_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    wifi_event_ftm_report_t *r = (wifi_event_ftm_report_t *)data;
    if (r->status == FTM_STATUS_SUCCESS) {
        s_last_rtt_ns = r->rtt_est;
        s_last_dist_cm = r->dist_est;
        xEventGroupSetBits(s_ftm_event_group, FTM_DONE_BIT);
    } else {
        xEventGroupSetBits(s_ftm_event_group, FTM_FAIL_BIT);
    }
}

static int32_t run_ftm_to(const uint8_t mac[6])
{
    s_last_dist_cm = -1;
    xEventGroupClearBits(s_ftm_event_group, FTM_DONE_BIT | FTM_FAIL_BIT);

    wifi_ftm_initiator_cfg_t cfg = {
        .frm_count = FTM_FRM_COUNT,
        .burst_period = FTM_BURST_PERIOD,
        .channel = FTM_CHANNEL,
        .use_get_report_api = false
    };
    memcpy(cfg.resp_mac, mac, 6);

    if (esp_wifi_ftm_initiate_session(&cfg) != ESP_OK) return -1;

    EventBits_t bits = xEventGroupWaitBits(s_ftm_event_group, FTM_DONE_BIT | FTM_FAIL_BIT,
                                           pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    esp_wifi_ftm_end_session();

    return (bits & FTM_DONE_BIT) ? s_last_dist_cm : -1;
}

// ------------------------------------------------------------------
// POST distance difference
// ------------------------------------------------------------------
static void post_difference(int32_t diff, int32_t a1, int32_t a2)
{
    char url[128], body[128];
    snprintf(url, sizeof(url), "http://%s:%d%s", ESP8266_SERVER_IP, ESP8266_SERVER_PORT, ESP8266_ENDPOINT);
    snprintf(body, sizeof(body), "{\"difference\":%" PRId32 ",\"a1\":%" PRId32 ",\"a2\":%" PRId32 "}", diff, a1, a2);

    esp_http_client_config_t cfg = { .url = url, .method = HTTP_METHOD_POST };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

// ------------------------------------------------------------------
// Tasks
// ------------------------------------------------------------------
static void ftm_task(void *arg)
{
    while (1) {
        s_anchor1_dist_cm = run_ftm_to(ANCHOR1_MAC);
        vTaskDelay(pdMS_TO_TICKS(200));
        s_anchor2_dist_cm = run_ftm_to(ANCHOR2_MAC);
        vTaskDelay(pdMS_TO_TICKS(200));

        if (s_anchor1_dist_cm >= 0 && s_anchor2_dist_cm >= 0) {
            int32_t diff = s_anchor2_dist_cm - s_anchor1_dist_cm;
            post_difference(diff, s_anchor1_dist_cm, s_anchor2_dist_cm);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void mic_task(void *arg)
{
    udp_packet[0] = SENDER_ID;

    while (1) {
        size_t samples = 0;
        while (samples < SAMPLES_PER_PACKET) {
            size_t bytes_read = 0;
            i2s_read(I2S_NUM_0, &i2s_buf[samples],
                     (SAMPLES_PER_PACKET - samples) * sizeof(int32_t),
                     &bytes_read, pdMS_TO_TICKS(50));
            samples += bytes_read / sizeof(int32_t);
        }

        for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
            pcm_buf[i] = i2s_24_to_pcm16(i2s_buf[i]);
            udp_packet[1 + 2*i] = pcm_buf[i] & 0xFF;
            udp_packet[1 + 2*i + 1] = (pcm_buf[i] >> 8) & 0xFF;
        }

        sendto(udp_sock, udp_packet, sizeof(udp_packet), 0,
               (struct sockaddr *)&udp_dest_addr, sizeof(udp_dest_addr));
    }
}

// ------------------------------------------------------------------
// Main
// ------------------------------------------------------------------
void app_main(void)
{
    nvs_flash_init();
    wifi_init();
    i2s_init();
    s_ftm_event_group = xEventGroupCreate();
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_FTM_REPORT, ftm_event_handler, NULL);
    udp_sock = udp_init();

    xTaskCreate(ftm_task, "ftm_task", 4096, NULL, 5, NULL);
    xTaskCreate(mic_task, "mic_task", 4096, NULL, 5, NULL);
}
