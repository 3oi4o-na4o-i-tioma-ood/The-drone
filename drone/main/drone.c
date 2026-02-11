#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BLDC.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "linear_algebra.h"
#include "mpu6050.h"
#include "stabilization.h"
#include "unity.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "lora.h"
#include "wifi.h"
#include "esp_timer.h"

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define LEFT_BUTTON_BIT 0
#define RIGHT_BUTTON_BIT 1
#define UP_BUTTON_BIT 2
#define DOWN_BUTTON_BIT 3
#define CENTER_BUTTON_BIT 4

static const char *TAG = "mpu6050 test";

static mpu6050_handle_t mpu6050 = NULL;

#define gyro_recent_values_count 1

float gyro_recent_values[gyro_recent_values_count];

int oldest_value_index = 0;

float angleIntegralY = 0;
float angleIntegralX = 0;

// Returns the average of the most recent values after the update
float save_new_gyro_value(float value) {
  gyro_recent_values[oldest_value_index] = value;

  oldest_value_index = (oldest_value_index + 1) % gyro_recent_values_count;

  float sum = 0;
  for (int i = 0; i < gyro_recent_values_count; i++) {
    sum += gyro_recent_values[i];
  }

  return sum / gyro_recent_values_count;
}

typedef struct {
  BLDC motorTop;
  BLDC motorRight;
  BLDC motorBottom;
  BLDC motorLeft;
} Drone;

Drone drone;

void mpu6050_get_orientation(VectorFloat *const i, VectorFloat *const j,
                             VectorFloat *const k) {
  int16_t quaternion[4];

  mpu6050_dmp_get_quaternion(quaternion);

  Quaternion q = {
      (float)quaternion[0] / 16384.0f, (float)quaternion[1] / 16384.0f,
      (float)quaternion[2] / 16384.0f, (float)quaternion[3] / 16384.0f};

  i->x = 1;
  i->y = 0;
  i->z = 0;

  j->x = 0;
  j->y = 1;
  j->z = 0;

  k->x = 0;
  k->y = 0;
  k->z = 1;

  rotateWithQuaternion(i, &q);
  rotateWithQuaternion(j, &q);
  rotateWithQuaternion(k, &q);
}

void gyro_rotate45(const double gyroX, const double gyroY, double *gyroXR,
                   double *gyroYR) {
  *gyroXR = (gyroX + gyroY) * sqrt(2) / 2;
  *gyroYR = (gyroX - gyroY) * sqrt(2) / 2;
}

float max(float a, float b) { return a > b ? a : b; }

float min(float a, float b) { return a < b ? a : b; }

float minmax(float n, float n_min, float n_max) {
  return min(max(n, n_min), n_max);
}

void BLDC_factory_reset() {
  BLDC_set_throttle(&drone.motorTop, 1);
  BLDC_set_throttle(&drone.motorBottom, 1);
  BLDC_set_throttle(&drone.motorLeft, 1);
  BLDC_set_throttle(&drone.motorRight, 1);

  vTaskDelay(5000 / portTICK_PERIOD_MS);

  BLDC_set_throttle(&drone.motorTop, 0);
  BLDC_set_throttle(&drone.motorBottom, 0);
  BLDC_set_throttle(&drone.motorLeft, 0);
  BLDC_set_throttle(&drone.motorRight, 0);

  vTaskDelay(7000 / portTICK_PERIOD_MS);
}

float angleErrorX = 0;
float angleErrorY = 0;

int msSinceTakeOff = 0;
float average_throttle = 0.8;
float angularSpeed = 0;

void task_tx(void *p) {
  ESP_LOGI(TAG, "task_tx started");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Trying to send packet");
    lora_send_packet((uint8_t *)"Hello", 5);
    printf("packet sent...\n");
    ESP_LOGI(TAG, "packet sent");
  }

  ESP_LOGI(TAG, "task_tx ended");
}

typedef struct __attribute__((packed)) {
  uint8_t buttons;
  uint8_t joystick_x; // 0..255
  uint8_t joystick_y; // 0..255
} rc_udp_packet_t;

static volatile rc_udp_packet_t s_last_rc_pkt = {0};
static volatile int64_t s_last_rc_pkt_us = 0;

static void udp_rx_cb(const uint8_t *data, size_t len, const char *from_ip,
                      uint16_t from_port, void *ctx) {
  (void)ctx;
  if (len == sizeof(rc_udp_packet_t)) {
    rc_udp_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt)); // safe even if data is unaligned

    s_last_rc_pkt = pkt;
    s_last_rc_pkt_us = esp_timer_get_time();

    ESP_LOGI("udp", "rc from %s:%u buttons=0x%02x joy=(%u,%u)", from_ip,
             (unsigned)from_port, (unsigned)pkt.buttons,
             (unsigned)pkt.joystick_x, (unsigned)pkt.joystick_y);
    return;
  }

  ESP_LOGI("udp", "rx %u bytes from %s:%u (unparsed)", (unsigned)len, from_ip,
           (unsigned)from_port);
}

void app_main() {
  printf("Hi");
  ESP_LOGI(TAG, "Hi");

  lora_init();
  lora_set_frequency(868e6);
  lora_enable_crc();
  xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);

  wifi_ap_udp_config_t wifi_cfg = {
      .ssid = "DRONE_AP",
      .pass = "drone1234",
      .udp_port = 3333,
  };
  ESP_ERROR_CHECK(wifi_ap_udp_start(&wifi_cfg, udp_rx_cb, NULL));

  for (int i = 0; i < gyro_recent_values_count; i++) {
    gyro_recent_values[i] = 0;
  }

  // BLDC_create(&drone.motorLeft, 32, LEDC_CHANNEL_0);
  // BLDC_create(&drone.motorRight, 33, LEDC_CHANNEL_1);

  BLDC_create(&drone.motorTop, 33, LEDC_CHANNEL_0);
  BLDC_create(&drone.motorRight, 27, LEDC_CHANNEL_1);
  BLDC_create(&drone.motorBottom, 25, LEDC_CHANNEL_2);
  BLDC_create(&drone.motorLeft, 26, LEDC_CHANNEL_3);

  BLDC_factory_reset();

  ESP_LOGI(TAG, "Throttle to 0");

  // vTaskDelay(500 / portTICK_PERIOD_MS);
  // BLDC_set_throttle(&drone.motorTop, 0.25);
  // BLDC_set_throttle(&drone.motorRight, 0.25);
  // BLDC_set_throttle(&drone.motorBottom, 0.25);
  // BLDC_set_throttle(&drone.motorLeft, 0.25);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);

  BLDC_set_throttle(&drone.motorTop, 0);
  BLDC_set_throttle(&drone.motorRight, 0);
  BLDC_set_throttle(&drone.motorBottom, 0);
  BLDC_set_throttle(&drone.motorLeft, 0);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  mpu6050_init(&mpu6050, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO,
               I2C_MASTER_FREQ_HZ);
  mpu6050_gyro_value_t gyro;

  vTaskDelay(10 / portTICK_PERIOD_MS);

  // const int errorsInterations = 100;

  // for (int i = 0; i < errorsInterations; i++)
  // {
  //     mpu6050_get_current_FIFO_packet(mpu6050);

  //     VectorFloat i, j, k;
  //     mpu6050_get_orientation(&i, &j, &k);

  //     angleErrorX += asin(j.z) / M_PI * 180;
  //     angleErrorY += asin(i.z) / M_PI * 180;

  //     ESP_LOGI(TAG, "Angle x: %.2f, y: %.2f", asin(j.z) / M_PI * 180,
  //     asin(i.z) / M_PI * 180);

  //     vTaskDelay(50 / portTICK_PERIOD_MS);
  // }

  // angleErrorX /= errorsInterations;
  // angleErrorY /= errorsInterations;

  ESP_LOGI(TAG, "Angle error x: %.2f, y: %.2f", angleErrorX, angleErrorY);

  vTaskDelay(2000 / portTICK_PERIOD_MS);

  while (true) {
    // struct timeval tv_now;
    // gettimeofday(&tv_now, NULL);
    // int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L +
    // (int64_t)tv_now.tv_usec; printf("%" PRId64 "\n", time_us);

    // BLDC_set_throttle(&drone.motorTop, throttle);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    msSinceTakeOff += 10;

    if(s_last_rc_pkt.buttons & (1 << CENTER_BUTTON_BIT)) {
      ESP_LOGI(TAG, "Center button pressed, stopping the drone");
      BLDC_set_throttle(&drone.motorTop, 0);
      BLDC_set_throttle(&drone.motorRight, 0);
      BLDC_set_throttle(&drone.motorBottom, 0);
      BLDC_set_throttle(&drone.motorLeft, 0);
      return;
    }

    if(s_last_rc_pkt.buttons & (1 << UP_BUTTON_BIT)) {
      average_throttle = 0.9;
    } else if(s_last_rc_pkt.buttons & (1 << DOWN_BUTTON_BIT)) {
      average_throttle = 0.5;
    } else {
      average_throttle = 0.7;
    }

    if(s_last_rc_pkt.buttons & (1 << LEFT_BUTTON_BIT)) {
      angularSpeed = 0.3;
    } else if(s_last_rc_pkt.buttons & (1 << RIGHT_BUTTON_BIT)) {
      angularSpeed = -0.3;
    }
    else {
      angularSpeed = 0;
    }



    if (msSinceTakeOff >= 10000) {
      BLDC_set_throttle(&drone.motorTop, 0);
      BLDC_set_throttle(&drone.motorRight, 0);
      BLDC_set_throttle(&drone.motorBottom, 0);
      BLDC_set_throttle(&drone.motorLeft, 0);
      break;
    }

    mpu6050_get_current_FIFO_packet(mpu6050);

    VectorFloat i, j, k;
    mpu6050_get_orientation(&i, &j, &k);

    VectorFloat angle;

    angle.x = asin(j.z) / M_PI * 180 - angleErrorX;
    angle.y = (asin(i.z) / M_PI * 180 - angleErrorY) * -1;

    if (isnan(angle.y)) {
      continue;
    }

    angleIntegralY += angle.y * 0.01; // Assuming dt is constant 0.01s
    angleIntegralX += angle.x * 0.01;

    mpu6050_get_gyro(mpu6050, &gyro);

    // float gyroAvY = save_new_gyro_value(gyro.gyro_y);

    double accY = calcAcc(gyro.gyro_y, angle.y, angleIntegralY);
    double accX = calcAcc(gyro.gyro_x, angle.x, angleIntegralX);

    // ESP_LOGI(TAG, "i.z: %.2f, j.z: %.2f", i.z, j.z);
    //  ESP_LOGI(TAG, "Angle: %.2f", angle.y);
    //  ESP_LOGI(TAG, "Gyro : %.2f", gyro.gyro_y);
    // ESP_LOGI(TAG, "Integ: %.2f", angleIntegralY);

    // ESP_LOGI(TAG, "Acc  : %.2f", accY);

    // ESP_LOGI(TAG, "top: %.2f, bottom: %.2f\n",
    //          minmax(0.4 - accY, 0.1, 1) * 0.5,
    //          minmax(0.4 + accY, 0.1, 1) * 0.5);

    double tgtSpeedY = (s_last_rc_pkt.joystick_y - 128) / 255.0 * 0.2;
    double tgtSpeedX = (s_last_rc_pkt.joystick_x - 128) / 255.0 * 0.2;

    BLDC_set_throttle(&drone.motorLeft,
                      minmax(0.4 - accX + angularSpeed + tgtSpeedY, 0.1, 1) * average_throttle);
    BLDC_set_throttle(&drone.motorRight,
                      minmax(0.4 + accX + angularSpeed - tgtSpeedY, 0.1, 1) * average_throttle);

    BLDC_set_throttle(&drone.motorTop,
                      minmax(0.4 + accY - angularSpeed + tgtSpeedX, 0.1, 1) * average_throttle);
    BLDC_set_throttle(&drone.motorBottom,
                      minmax(0.4 - accY - angularSpeed - tgtSpeedX, 0.1, 1) * average_throttle);
  }
}