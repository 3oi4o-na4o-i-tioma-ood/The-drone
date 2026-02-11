// wifi.h - SoftAP + UDP receiver
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*wifi_udp_rx_cb_t)(const uint8_t *data, size_t len,
                                 const char *from_ip, uint16_t from_port,
                                 void *ctx);

typedef struct {
  const char *ssid;             // required
  const char *pass;             // "" or NULL for open AP
  uint8_t channel;              // default 1 if 0
  uint8_t max_connection;       // default 4 if 0
  bool ssid_hidden;             // default false
  uint16_t beacon_interval_ms;  // default 100 if 0

  uint16_t udp_port;            // required
  int udp_task_stack_bytes;     // default 4096 if 0
  int udp_task_priority;        // default 5 if 0
} wifi_ap_udp_config_t;

// Starts Wi-Fi SoftAP + a UDP receiver task bound to cfg->udp_port.
// Incoming datagrams invoke cb() from the UDP task context.
esp_err_t wifi_ap_udp_start(const wifi_ap_udp_config_t *cfg,
                            wifi_udp_rx_cb_t cb, void *cb_ctx);

// Stops the UDP receiver task and de-inits Wi-Fi (best-effort).
esp_err_t wifi_ap_udp_stop(void);

#ifdef __cplusplus
}
#endif
