#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"
#include "linear_algebra.h"
#include "BLDC.h"
#include "stabilization.h"
#include "esp_sntp.h"
#include <inttypes.h>

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";

static mpu6050_handle_t mpu6050 = NULL;

typedef struct
{
    BLDC motorTop;
    BLDC motorRight;
    BLDC motorBottom;
    BLDC motorLeft;
} Drone;

Drone drone;

void mpu6050_get_orientation(VectorFloat *const i, VectorFloat *const j, VectorFloat *const k)
{
    int16_t quaternion[4];

    mpu6050_dmp_get_quaternion(quaternion);

    Quaternion q = {
        (float)quaternion[0] / 16384.0f,
        (float)quaternion[1] / 16384.0f,
        (float)quaternion[2] / 16384.0f,
        (float)quaternion[3] / 16384.0f};

    i->x = sqrt(2) / 2;
    i->y = sqrt(2) / 2;
    i->z = 0;

    j->x = sqrt(2) / 2;
    j->y = -sqrt(2) / 2;
    j->z = 0;

    k->x = 0;
    k->y = 0;
    k->z = 1;

    rotateWithQuaternion(i, &q);
    rotateWithQuaternion(j, &q);
    rotateWithQuaternion(k, &q);
}

void gyro_rotate45(const double gyroX, const double gyroY, double* gyroXR, double* gyroYR) {
    *gyroXR = (gyroX + gyroY) * sqrt(2) / 2;
    *gyroYR = (gyroX - gyroY) * sqrt(2) / 2;
}

float max(float a, float b)
{
    return a > b ? a : b;
}

float min(float a, float b)
{
    return a < b ? a : b;
}

float minmax(float n, float n_min, float n_max)
{
    return min(max(n, n_min), n_max);
}

void BLDC_factory_reset()
{
    BLDC_set_throttle(&drone.motorTop, 1);
    BLDC_set_throttle(&drone.motorBottom, 1);
    BLDC_set_throttle(&drone.motorLeft, 1);
    BLDC_set_throttle(&drone.motorRight, 1);

    vTaskDelay(7000 / portTICK_PERIOD_MS);

    BLDC_set_throttle(&drone.motorTop, 0);
    BLDC_set_throttle(&drone.motorBottom, 0);
    BLDC_set_throttle(&drone.motorLeft, 0);
    BLDC_set_throttle(&drone.motorRight, 0);

    vTaskDelay(1900 / portTICK_PERIOD_MS);

    BLDC_set_throttle(&drone.motorTop, 1);
    BLDC_set_throttle(&drone.motorBottom, 1);
    BLDC_set_throttle(&drone.motorLeft, 1);
    BLDC_set_throttle(&drone.motorRight, 1);

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    BLDC_set_throttle(&drone.motorTop, 0);
    BLDC_set_throttle(&drone.motorBottom, 0);
    BLDC_set_throttle(&drone.motorLeft, 0);
    BLDC_set_throttle(&drone.motorRight, 0);

    vTaskDelay(500 / portTICK_PERIOD_MS);
}

float angleErrorX = 0;
float angleErrorY = 0;

void app_main()
{
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Hi");

    // BLDC_create(&drone.motorTop, 25, LEDC_CHANNEL_0);
    // BLDC_create(&drone.motorRight, 26, LEDC_CHANNEL_1);
    // BLDC_create(&drone.motorBottom, 27, LEDC_CHANNEL_2);
    // BLDC_create(&drone.motorLeft, 33, LEDC_CHANNEL_3);

    BLDC_create(&drone.motorLeft, 32, LEDC_CHANNEL_0);
    BLDC_create(&drone.motorRight, 33, LEDC_CHANNEL_1);

    // BLDC_factory_reset();
    // vTaskDelay(15000 / portTICK_PERIOD_MS);

    vTaskDelay(3500 / portTICK_PERIOD_MS);
    BLDC_set_throttle(&drone.motorTop, 0);
    BLDC_set_throttle(&drone.motorRight, 0);
    BLDC_set_throttle(&drone.motorBottom, 0);
    BLDC_set_throttle(&drone.motorLeft, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    mpu6050_init(&mpu6050, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    mpu6050_gyro_value_t gyro;

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    for (int i = 0; i < 100; i++)
    {
        mpu6050_get_current_FIFO_packet(mpu6050);

        VectorFloat i, j, k;
        mpu6050_get_orientation(&i, &j, &k);

        angleErrorX += asin(i.z) / M_PI * 180;
        angleErrorY += asin(j.z) / M_PI * 180;

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    angleErrorX /= 300;
    angleErrorY /= 300;

    ESP_LOGI(TAG, "Angle error x: %.2f, y: %.2f", angleErrorX, angleErrorY);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (float t = 0; t <= 1; t += 0.24)
    {

        BLDC_set_throttle(&drone.motorTop, t * 0.5);
        BLDC_set_throttle(&drone.motorBottom, t * 0.5);
        BLDC_set_throttle(&drone.motorLeft, t * 0.5);
        BLDC_set_throttle(&drone.motorRight, t * 0.5);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    for (float t = 1; t > 0; t -= 0.24)
    {
        BLDC_set_throttle(&drone.motorTop, t * 0.5);
        BLDC_set_throttle(&drone.motorBottom, t * 0.5);
        BLDC_set_throttle(&drone.motorLeft, t * 0.5);
        BLDC_set_throttle(&drone.motorRight, t * 0.5);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    BLDC_set_throttle(&drone.motorTop, 0);
    BLDC_set_throttle(&drone.motorRight, 0);
    BLDC_set_throttle(&drone.motorBottom, 0);
    BLDC_set_throttle(&drone.motorLeft, 0);

    while (true)
    {
        // struct timeval tv_now;
        // gettimeofday(&tv_now, NULL);
        // int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        // printf("%" PRId64 "\n", time_us);

        // BLDC_set_throttle(&drone.motorTop, throttle);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        mpu6050_get_current_FIFO_packet(mpu6050);

        VectorFloat i, j, k;
        mpu6050_get_orientation(&i, &j, &k);

        VectorFloat angle;

        angle.x = asin(i.z) / M_PI * 180 - angleErrorX;
        angle.y = asin(j.z) / M_PI * 180 - angleErrorY;

        mpu6050_get_gyro(mpu6050, &gyro);

        double gyroX;
        double gyroY;

        gyro_rotate45(gyro.gyro_x, gyro.gyro_y, &gyroX, &gyroY);

        double accX = calcAcc(-gyroX, angle.x) * 0.5;
        double accY = calcAcc(-gyroY, angle.y) * 0.5;

        // ESP_LOGI(TAG, "i.z: %.2f, j.z: %.2f", i.z, j.z);
        //ESP_LOGI(TAG, "Angle x: %.2f, y: %.2f", angle.x, angle.y);
        ESP_LOGI(TAG, "Gyro x: %.2f, y: %.2f", gyroX, gyroY);
        // ESP_LOGI(TAG, "Acc x: %.2f, y: %.2f\n", accX, accY);

        ESP_LOGI(TAG, "top: %.2f, bottom: %.2f, left: %.2f, right: %.2f\n",
                 minmax(0.4 + accY, 0.1, 1) * 0.5,
                 minmax(0.4 - accY, 0.1, 1) * 0.5,
                 minmax(0.4 - accX, 0.1, 1) * 0.5,
                 minmax(0.4 + accX, 0.1, 1) * 0.5);

        BLDC_set_throttle(&drone.motorTop,      minmax(0.4 + accY, 0.1, 1) * 0.5);
        BLDC_set_throttle(&drone.motorBottom,   minmax(0.4 - accY, 0.1, 1) * 0.5);
        BLDC_set_throttle(&drone.motorLeft,     minmax(0.4 - accX, 0.1, 1) * 0.5);
        BLDC_set_throttle(&drone.motorRight,    minmax(0.4 + accX, 0.1, 1) * 0.5);
    }
}