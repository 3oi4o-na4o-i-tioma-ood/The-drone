#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "unity.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "wifi.h"

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "DRONE_RC";

#define LEFT_BUTTON_PIN 18
#define RIGHT_BUTTON_PIN 12
#define UP_BUTTON_PIN 14
#define DOWN_BUTTON_PIN 15
#define CENTER_BUTTON_PIN 25

// ESP32 ADC1 channels are on GPIO32-39 (GPIO34/35 are input-only and common for ADC)
#define JOYSTICK_X_CH ADC1_CHANNEL_6 // GPIO34
#define JOYSTICK_Y_CH ADC1_CHANNEL_7 // GPIO35

#define LEFT_BUTTON_BIT 0
#define RIGHT_BUTTON_BIT 1
#define UP_BUTTON_BIT 2
#define DOWN_BUTTON_BIT 3
#define CENTER_BUTTON_BIT 4

typedef struct __attribute__((packed)) {
  uint8_t buttons;
  uint8_t joystick_x; // 0..255
  uint8_t joystick_y; // 0..255
} rc_udp_packet_t;

static void task_udp_tx(void *p) {
  (void)p;
  ESP_LOGI("udp", "task_udp_tx started");

  for (;;) {
    ESP_LOGI("udp", "Left button: %d", gpio_get_level(LEFT_BUTTON_PIN));

    uint8_t button_state = (
      ((!gpio_get_level(LEFT_BUTTON_PIN)) << LEFT_BUTTON_BIT) |
      ((!gpio_get_level(RIGHT_BUTTON_PIN)) << RIGHT_BUTTON_BIT) |
      ((!gpio_get_level(UP_BUTTON_PIN)) << UP_BUTTON_BIT) |
      ((!gpio_get_level(DOWN_BUTTON_PIN)) << DOWN_BUTTON_BIT) |
      ((!gpio_get_level(CENTER_BUTTON_PIN)) << CENTER_BUTTON_BIT));

    // adc1_get_raw returns 0..4095 (12-bit) on ESP32.
    int raw_x = adc1_get_raw(JOYSTICK_X_CH);
    int raw_y = adc1_get_raw(JOYSTICK_Y_CH);
    if (raw_x < 0) raw_x = 0;
    if (raw_y < 0) raw_y = 0;
    if (raw_x > 4095) raw_x = 4095;
    if (raw_y > 4095) raw_y = 4095;

    rc_udp_packet_t pkt = {
        .buttons = button_state,
        .joystick_x = (uint8_t)((raw_x * 255) / 4095),
        .joystick_y = (uint8_t)((raw_y * 255) / 4095),
    };

    esp_err_t err = wifi_sta_udp_send(&pkt, sizeof(pkt));
    if (err != ESP_OK) {
      ESP_LOGW("udp", "send failed: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void init_buttons() {
  gpio_config_t config = {
    .pin_bit_mask = (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << CENTER_BUTTON_PIN),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&config);

  // Configure ADC for joystick
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(JOYSTICK_X_CH, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(JOYSTICK_Y_CH, ADC_ATTEN_DB_11);
}

void app_main() {
  printf("Hi");
  ESP_LOGI(TAG, "Hi");

  init_buttons();

  wifi_sta_udp_config_t sta_cfg = {
      .ssid = "DRONE_AP",
      .pass = "drone1234",
      .remote_ip = "192.168.4.1",
      .remote_port = 3333,
  };
  ESP_ERROR_CHECK(wifi_sta_udp_start(&sta_cfg));
  xTaskCreate(&task_udp_tx, "task_udp_tx", 4096, NULL, 5, NULL);


  vTaskDelay(2000 / portTICK_PERIOD_MS);

  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}