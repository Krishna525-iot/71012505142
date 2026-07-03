#include "espnow_helper.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include <string.h>
#include "debug_control.h"

static const char *TAG = "ESPNOW_HELPER";

const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Application callback
static espnow_data_callback_t app_callback = NULL;


// ---------- NEW: Structured packet support ----------

typedef enum {
    DOME_L = 0,
    DOME_M = 1,
    DOME_R = 2
} dome_t;

typedef struct {
    uint8_t dome;
    uint8_t type;
    uint8_t payload[16];
    uint8_t len;
} espnow_packet_t;


// ESP-NOW receive callback
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data,
                           int data_len)
{
    if (data == NULL || data_len <= 0)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGW(TAG, "Invalid ESP-NOW data received");
        }
        return;
    }

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "ESP-NOW data received, len: %d", data_len);
    }

    // If data matches our structured packet
    if (data_len == sizeof(espnow_packet_t))
    {
        espnow_packet_t pkt;
        memcpy(&pkt, data, sizeof(espnow_packet_t));

        if (DEBUG_INFO)
        {
            ESP_LOGI(TAG, "Packet from dome: %d type: %d len: %d",
                     pkt.dome, pkt.type, pkt.len);
        }

        // Forward only payload to app (existing behavior)
        if (app_callback)
        {
            app_callback(pkt.payload, pkt.len);
        }
    }
    else
    {
        // Legacy support – pass raw
        if (app_callback)
        {
            app_callback(data, data_len);
        }
    }
}


// ESP-NOW Initialization
esp_err_t espnow_init(espnow_data_callback_t callback)
{
    esp_err_t err;

    app_callback = callback;

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Initializing ESP-NOW...");
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to register ESP-NOW receive callback: %s",
                     esp_err_to_name(err));
        }
        return err;
    }

    uint8_t pmk[16] = "pmk1234567890123";
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));

    esp_now_peer_info_t peer_info = {};
    memset(&peer_info, 0, sizeof(esp_now_peer_info_t));

    peer_info.channel = 0;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);

    err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to add broadcast peer: %s",
                     esp_err_to_name(err));
        }
        return err;
    }

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    }

    return ESP_OK;
}


// ---------- NEW: Dome-aware send function ----------

esp_err_t espnow_send_dome(uint8_t dome,
                           uint8_t type,
                           const uint8_t *payload,
                           size_t len)
{
    if (!payload || len == 0 || len > 16)
    {
        return ESP_ERR_INVALID_ARG;
    }

    espnow_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.dome = dome;
    pkt.type = type;
    pkt.len  = len;

    memcpy(pkt.payload, payload, len);

    esp_err_t err = esp_now_send(broadcast_mac,
                                 (uint8_t *)&pkt,
                                 sizeof(pkt));

    if (err != ESP_OK && DEBUG_INFO)
    {
        ESP_LOGE(TAG, "Failed to send dome packet: %s",
                 esp_err_to_name(err));
    }

    return err;
}


// Legacy send (kept for backward compatibility)
esp_err_t espnow_send(const uint8_t *data, size_t len)
{
    if (!data || len <= 0)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Invalid data to send");
        }
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_now_send(broadcast_mac, data, len);

    if (err != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to send data via ESP-NOW: %s",
                     esp_err_to_name(err));
        }
        return err;
    }

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Data sent via ESP-NOW successfully");
    }

    return ESP_OK;
}
