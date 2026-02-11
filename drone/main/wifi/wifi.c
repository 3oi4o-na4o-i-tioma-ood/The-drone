// wifi.c - SoftAP + UDP receiver

#include "wifi.h"

#include <errno.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char *TAG = "wifi";

static TaskHandle_t s_udp_task = NULL;
static int s_udp_sock = -1;
static wifi_udp_rx_cb_t s_rx_cb = NULL;
static void *s_rx_ctx = NULL;
static uint16_t s_udp_port = 0;

static esp_netif_t *s_ap_netif = NULL;
static bool s_started = false;

static esp_err_t nvs_init_if_needed(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  return err;
}

static void udp_server_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "UDP server task starting on port %u", (unsigned)s_udp_port);

  struct sockaddr_in bind_addr = {0};
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_port = htons(s_udp_port);
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  s_udp_sock = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (s_udp_sock < 0) {
    ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
    s_udp_task = NULL;
    vTaskDelete(NULL);
    return;
  }

  int yes = 1;
  (void)setsockopt(s_udp_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  if (bind(s_udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
    ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
    close(s_udp_sock);
    s_udp_sock = -1;
    s_udp_task = NULL;
    vTaskDelete(NULL);
    return;
  }

  uint8_t rx_buf[1472]; // typical MTU-safe UDP payload size
  for (;;) {
    struct sockaddr_in from = {0};
    socklen_t from_len = sizeof(from);
    int rlen = (int)recvfrom(s_udp_sock, rx_buf, sizeof(rx_buf), 0,
                             (struct sockaddr *)&from, &from_len);
    if (rlen < 0) {
      int err = errno;
      if (err == EINTR) {
        continue;
      }
      ESP_LOGE(TAG, "recvfrom() failed: errno=%d", err);
      break;
    }

    char from_ip[INET_ADDRSTRLEN] = {0};
    inet_ntoa_r(from.sin_addr, from_ip, sizeof(from_ip));
    uint16_t from_port = ntohs(from.sin_port);

    if (s_rx_cb) {
      s_rx_cb(rx_buf, (size_t)rlen, from_ip, from_port, s_rx_ctx);
    } else {
      ESP_LOGI(TAG, "UDP rx %d bytes from %s:%u", rlen, from_ip,
               (unsigned)from_port);
    }
  }

  if (s_udp_sock >= 0) {
    close(s_udp_sock);
    s_udp_sock = -1;
  }
  s_udp_task = NULL;
  vTaskDelete(NULL);
}

static void apply_defaults(wifi_ap_udp_config_t *dst,
                           const wifi_ap_udp_config_t *src) {
  *dst = *src;
  if (!dst->pass) {
    dst->pass = "";
  }
  if (dst->channel == 0) {
    dst->channel = 1;
  }
  if (dst->max_connection == 0) {
    dst->max_connection = 4;
  }
  if (dst->beacon_interval_ms == 0) {
    dst->beacon_interval_ms = 100;
  }
  if (dst->udp_task_stack_bytes == 0) {
    dst->udp_task_stack_bytes = 4096;
  }
  if (dst->udp_task_priority == 0) {
    dst->udp_task_priority = 5;
  }
}

esp_err_t wifi_ap_udp_start(const wifi_ap_udp_config_t *cfg,
                            wifi_udp_rx_cb_t cb, void *cb_ctx) {
  if (!cfg || !cfg->ssid || cfg->ssid[0] == '\0' || cfg->udp_port == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_started) {
    return ESP_ERR_INVALID_STATE;
  }

  wifi_ap_udp_config_t c;
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

  s_ap_netif = esp_netif_create_default_wifi_ap();
  if (!s_ap_netif) {
    return ESP_FAIL;
  }

  wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&init_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi storage set failed: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_wifi_set_mode(WIFI_MODE_AP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi mode set failed: %s", esp_err_to_name(err));
    return err;
  }

  wifi_config_t wifi_cfg = {0};
  strlcpy((char *)wifi_cfg.ap.ssid, c.ssid, sizeof(wifi_cfg.ap.ssid));
  wifi_cfg.ap.ssid_len = (uint8_t)strlen(c.ssid);
  strlcpy((char *)wifi_cfg.ap.password, c.pass, sizeof(wifi_cfg.ap.password));
  wifi_cfg.ap.channel = c.channel;
  wifi_cfg.ap.max_connection = c.max_connection;
  wifi_cfg.ap.ssid_hidden = c.ssid_hidden ? 1 : 0;
  wifi_cfg.ap.beacon_interval = c.beacon_interval_ms;

  if (c.pass[0] == '\0') {
    wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
  } else {
    wifi_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  }

  err = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "wifi config set failed: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_wifi_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "SoftAP started. SSID='%s' channel=%u auth=%s",
           c.ssid, (unsigned)c.channel,
           (wifi_cfg.ap.authmode == WIFI_AUTH_OPEN) ? "open" : "wpa/wpa2");

  s_rx_cb = cb;
  s_rx_ctx = cb_ctx;
  s_udp_port = c.udp_port;

  BaseType_t ok = xTaskCreate(udp_server_task, "udp_rx",
                              c.udp_task_stack_bytes, NULL,
                              c.udp_task_priority, &s_udp_task);
  if (ok != pdPASS) {
    s_udp_task = NULL;
    (void)wifi_ap_udp_stop();
    return ESP_ERR_NO_MEM;
  }

  s_started = true;
  return ESP_OK;
}

esp_err_t wifi_ap_udp_stop(void) {
  // Stop UDP first
  if (s_udp_sock >= 0) {
    shutdown(s_udp_sock, SHUT_RDWR);
    close(s_udp_sock);
    s_udp_sock = -1;
  }

  if (s_udp_task) {
    TaskHandle_t t = s_udp_task;
    s_udp_task = NULL;
    vTaskDelete(t);
  }

  s_rx_cb = NULL;
  s_rx_ctx = NULL;
  s_udp_port = 0;

  // Best-effort Wi-Fi teardown
  if (s_started) {
    (void)esp_wifi_stop();
    (void)esp_wifi_deinit();
    if (s_ap_netif) {
      esp_netif_destroy(s_ap_netif);
      s_ap_netif = NULL;
    }
    (void)esp_event_loop_delete_default();
  }

  s_started = false;
  return ESP_OK;
}
