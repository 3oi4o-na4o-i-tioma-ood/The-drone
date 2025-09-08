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
#include <math.h>

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";

static mpu6050_handle_t mpu6050 = NULL;

#define gyro_recent_values_count 1

float gyro_recent_values[gyro_recent_values_count];

int oldest_value_index = 0;

float angleIntegralY = 0;
float angleIntegralX = 0;

// Returns the average of the most recent values after the update
float save_new_gyro_value(float value)
{
    gyro_recent_values[oldest_value_index] = value;

    oldest_value_index = (oldest_value_index + 1) % gyro_recent_values_count;

    float sum = 0;
    for (int i = 0; i < gyro_recent_values_count; i++)
    {
        sum += gyro_recent_values[i];
    }

    return sum / gyro_recent_values_count;
}

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

void gyro_rotate45(const double gyroX, const double gyroY, double *gyroXR, double *gyroYR)
{
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
float average_throttle = 0.6;

void app_main()
{
    printf("Hi");

    for (int i = 0; i < gyro_recent_values_count; i++)
    {
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
    BLDC_set_throttle(&drone.motorTop, 0.25);
    BLDC_set_throttle(&drone.motorRight, 0.25);
    BLDC_set_throttle(&drone.motorBottom, 0.25);
    BLDC_set_throttle(&drone.motorLeft, 0.25);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    BLDC_set_throttle(&drone.motorTop, 0);
    BLDC_set_throttle(&drone.motorRight, 0);
    BLDC_set_throttle(&drone.motorBottom, 0);
    BLDC_set_throttle(&drone.motorLeft, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    mpu6050_init(&mpu6050, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    mpu6050_gyro_value_t gyro;

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // const int errorsInterations = 100;

    // for (int i = 0; i < errorsInterations; i++)
    // {
    //     mpu6050_get_current_FIFO_packet(mpu6050);

    //     VectorFloat i, j, k;
    //     mpu6050_get_orientation(&i, &j, &k);

    //     angleErrorX += asin(j.z) / M_PI * 180;
    //     angleErrorY += asin(i.z) / M_PI * 180;

    //     ESP_LOGI(TAG, "Angle x: %.2f, y: %.2f", asin(j.z) / M_PI * 180, asin(i.z) / M_PI * 180);

    //     vTaskDelay(50 / portTICK_PERIOD_MS);
    // }

    // angleErrorX /= errorsInterations;
    // angleErrorY /= errorsInterations;

    ESP_LOGI(TAG, "Angle error x: %.2f, y: %.2f", angleErrorX, angleErrorY);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    while (true)
    {
        // struct timeval tv_now;
        // gettimeofday(&tv_now, NULL);
        // int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        // printf("%" PRId64 "\n", time_us);

        // BLDC_set_throttle(&drone.motorTop, throttle);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        msSinceTakeOff += 10;
        if (msSinceTakeOff >= 5000)
        {
            average_throttle = 0.3;
        }

        if(msSinceTakeOff >= 7000) {
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

        if (isnan(angle.y))
        {
            continue;
        }

        angleIntegralY += angle.y * 0.01; // Assuming dt is constant 0.01s
        angleIntegralX += angle.x * 0.01;

        mpu6050_get_gyro(mpu6050, &gyro);

        // float gyroAvY = save_new_gyro_value(gyro.gyro_y);

        double accY = calcAcc(gyro.gyro_y, angle.y, angleIntegralY);
        double accX = calcAcc(gyro.gyro_x, angle.x, angleIntegralX);

        //ESP_LOGI(TAG, "i.z: %.2f, j.z: %.2f", i.z, j.z);
        // ESP_LOGI(TAG, "Angle: %.2f", angle.y);
        // ESP_LOGI(TAG, "Gyro : %.2f", gyro.gyro_y);
        //ESP_LOGI(TAG, "Integ: %.2f", angleIntegralY);

        // ESP_LOGI(TAG, "Acc  : %.2f", accY);

        // ESP_LOGI(TAG, "top: %.2f, bottom: %.2f\n",
        //          minmax(0.4 - accY, 0.1, 1) * 0.5,
        //          minmax(0.4 + accY, 0.1, 1) * 0.5);

        BLDC_set_throttle(&drone.motorLeft, minmax(0.4 - accX, 0.1, 1) * average_throttle);
        BLDC_set_throttle(&drone.motorRight, minmax(0.4 + accX, 0.1, 1) * average_throttle);

        BLDC_set_throttle(&drone.motorTop, minmax(0.4 + accY, 0.1, 1) * average_throttle);
        BLDC_set_throttle(&drone.motorBottom, minmax(0.4 - accY, 0.1, 1) * average_throttle);
    }
}