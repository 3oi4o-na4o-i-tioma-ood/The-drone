// wifi.h - Wi-Fi STA + UDP sender (RC side)
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const char *ssid;       // required
  const char *pass;       // "" or NULL for open network

  const char *remote_ip;  // required (e.g. "192.168.4.1")
  uint16_t remote_port;   // required (e.g. 3333)

  uint32_t connect_timeout_ms; // default 10000 if 0 (wait in start)
  int tx_timeout_ms;           // default 50 if 0
} wifi_sta_udp_config_t;

// Connects to Wi-Fi as a station and prepares a UDP socket aimed at
// cfg->remote_ip:cfg->remote_port.
esp_err_t wifi_sta_udp_start(const wifi_sta_udp_config_t *cfg);

// True when STA has an IP (connected).
bool wifi_sta_udp_is_connected(void);

// Sends one UDP datagram to configured remote.
esp_err_t wifi_sta_udp_send(const void *data, size_t len);

// Stops UDP + Wi-Fi (best-effort).
esp_err_t wifi_sta_udp_stop(void);

#ifdef __cplusplus
}
#endif
