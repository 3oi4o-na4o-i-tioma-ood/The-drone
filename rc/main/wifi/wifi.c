// wifi.c - Wi-Fi STA + UDP sender (RC side)

#include "wifi.h"

#include <errno.h>
#include <string.h>
#include <inttypes.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char *TAG = "wifi";

static EventGroupHandle_t s_ev = NULL;
static esp_netif_t *s_sta_netif = NULL;
static int s_udp_sock = -1;
static struct sockaddr_in s_remote = {0};

static bool s_started = false;
static bool s_handlers_registered = false;

#define WIFI_CONNECTED_BIT BIT0

static esp_err_t nvs_init_if_needed(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  return err;
}

static void on_wifi_event(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  (void)arg;
  (void)event_base;
  (void)event_data;

  if (event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "STA start -> connect");
    esp_wifi_connect();
  } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGW(TAG, "STA disconnected -> reconnect");
    if (s_ev) {
      xEventGroupClearBits(s_ev, WIFI_CONNECTED_BIT);
    }
    esp_wifi_connect();
  }
}

static void on_ip_event(void *arg, esp_event_base_t event_base, int32_t event_id,
                        void *event_data) {
  (void)arg;
  (void)event_base;
  (void)event_data;

  if (event_id == IP_EVENT_STA_GOT_IP) {
    if (s_ev) {
      xEventGroupSetBits(s_ev, WIFI_CONNECTED_BIT);
    }
    ESP_LOGI(TAG, "STA got IP");
  }
}

static void apply_defaults(wifi_sta_udp_config_t *dst,
                           const wifi_sta_udp_config_t *src) {
  *dst = *src;
  if (!dst->pass) {
    dst->pass = "";
  }
  if (dst->connect_timeout_ms == 0) {
    dst->connect_timeout_ms = 10000;
  }
  if (dst->tx_timeout_ms == 0) {
    dst->tx_timeout_ms = 50;
  }
}

esp_err_t wifi_sta_udp_start(const wifi_sta_udp_config_t *cfg) {
  if (!cfg || !cfg->ssid || cfg->ssid[0] == '\0' || !cfg->remote_ip ||
      cfg->remote_ip[0] == '\0' || cfg->remote_port == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_started) {
    return ESP_ERR_INVALID_STATE;
  }

  wifi_sta_udp_config_t c;
  apply_defaults(&c, cfg);

  esp_err_t err = nvs_init_if_needed();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_netif_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(err));
    return err;
  }

  s_ev = xEventGroupCreate();
  if (!s_ev) {
    return ESP_ERR_NO_MEM;
  }

  s_sta_netif = esp_netif_create_default_wifi_sta();
  if (!s_sta_netif) {
    wifi_sta_udp_stop();
    return ESP_FAIL;
  }

  wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&init_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
    wifi_sta_udp_stop();
    return err;
  }

  err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event,
                                   NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi handler register failed: %s", esp_err_to_name(err));
    wifi_sta_udp_stop();
    return err;
  }

  err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_event,
                                   NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ip handler register failed: %s", esp_err_to_name(err));
    wifi_sta_udp_stop();
    return err;
  }

  s_handlers_registered = true;

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi storage set failed: %s", esp_err_to_name(err));
    wifi_sta_udp_stop();
    return err;
  }

  wifi_config_t wifi_cfg = {0};
  strlcpy((char *)wifi_cfg.sta.ssid, c.ssid, sizeof(wifi_cfg.sta.ssid));
  strlcpy((char *)wifi_cfg.sta.password, c.pass,
          sizeof(wifi_cfg.sta.password));
  wifi_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  wifi_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

  err = esp_wifi_set_mode(WIFI_MODE_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi mode set failed: %s", esp_err_to_name(err));
    wifi_sta_udp_stop();
    return err;
  }

  err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi config set failed: %s", esp_err_to_name(err));
    wifi_sta_udp_stop();
    return err;
  }

  err = esp_wifi_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
    wifi_sta_udp_stop();
    return err;
  }

  // Prepare UDP socket + remote address
  memset(&s_remote, 0, sizeof(s_remote));
  s_remote.sin_family = AF_INET;
  s_remote.sin_port = htons(c.remote_port);
  if (inet_pton(AF_INET, c.remote_ip, &s_remote.sin_addr) != 1) {
    ESP_LOGE(TAG, "invalid remote_ip: %s", c.remote_ip);
    wifi_sta_udp_stop();
    return ESP_ERR_INVALID_ARG;
  }

  s_udp_sock = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (s_udp_sock < 0) {
    ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
    wifi_sta_udp_stop();
    return ESP_FAIL;
  }

  struct timeval tv = {.tv_sec = 0, .tv_usec = c.tx_timeout_ms * 1000};
  (void)setsockopt(s_udp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  // Optional: connect() sets default destination for send()
  if (connect(s_udp_sock, (struct sockaddr *)&s_remote, sizeof(s_remote)) != 0) {
    ESP_LOGE(TAG, "connect(udp) failed: errno=%d", errno);
    wifi_sta_udp_stop();
    return ESP_FAIL;
  }

  // Wait for IP
  EventBits_t bits = xEventGroupWaitBits(
      s_ev, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
      pdMS_TO_TICKS(c.connect_timeout_ms));
  if ((bits & WIFI_CONNECTED_BIT) == 0) {
    ESP_LOGW(TAG, "connect timeout (%" PRIu32 " ms)", c.connect_timeout_ms);
    // keep running; caller can retry sends later
  }

  s_started = true;
  ESP_LOGI(TAG, "STA started. SSID='%s' remote=%s:%u", c.ssid, c.remote_ip,
           (unsigned)c.remote_port);
  return ESP_OK;
}

bool wifi_sta_udp_is_connected(void) {
  if (!s_ev) {
    return false;
  }
  return (xEventGroupGetBits(s_ev) & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t wifi_sta_udp_send(const void *data, size_t len) {
  if (!s_started || s_udp_sock < 0) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!data || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!wifi_sta_udp_is_connected()) {
    return ESP_ERR_INVALID_STATE;
  }

  int rc = (int)send(s_udp_sock, data, len, 0);
  if (rc < 0) {
    ESP_LOGE(TAG, "send() failed: errno=%d", errno);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t wifi_sta_udp_stop(void) {
  if (s_udp_sock >= 0) {
    shutdown(s_udp_sock, SHUT_RDWR);
    close(s_udp_sock);
    s_udp_sock = -1;
  }

  // Best-effort teardown
  if (s_handlers_registered) {
    (void)esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                       &on_wifi_event);
    (void)esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                       &on_ip_event);
    s_handlers_registered = false;
  }

  (void)esp_wifi_stop();
  (void)esp_wifi_deinit();

  if (s_sta_netif) {
    esp_netif_destroy(s_sta_netif);
    s_sta_netif = NULL;
  }

  if (s_ev) {
    vEventGroupDelete(s_ev);
    s_ev = NULL;
  }

  s_started = false;
  memset(&s_remote, 0, sizeof(s_remote));
  return ESP_OK;
}
