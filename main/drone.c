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

void mpu6050_get_normal(VectorFloat *const normal)
{
    int16_t quaternion[4];

    mpu6050_dmp_get_quaternion(quaternion);

    Quaternion q = {
        (float)quaternion[0] / 16384.0f,
        (float)quaternion[1] / 16384.0f,
        (float)quaternion[2] / 16384.0f,
        (float)quaternion[3] / 16384.0f};

    normal->x = 0;
    normal->y = 0;
    normal->z = 1;
    rotateWithQuaternion(normal, &q);
}

void app_main()
{
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Hi");

    // mpu6050_init(&mpu6050, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    BLDC_create(&drone.motorTop, 25, LEDC_CHANNEL_0);
    BLDC_create(&drone.motorRight, 26, LEDC_CHANNEL_1);
    BLDC_create(&drone.motorBottom, 27, LEDC_CHANNEL_2);
    BLDC_create(&drone.motorLeft, 33, LEDC_CHANNEL_3);

    float throttle = 0.0;
    int dir = 1;
    while (true)
    {
        BLDC_set_throttle(&drone.motorTop, throttle);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        throttle += 0.1 * dir;
        if (throttle >= 0.7 || throttle <= 0.1)
        {
            dir *= -1;
        }

        // mpu6050_get_current_FIFO_packet(mpu6050);

        // VectorFloat normal;
        // mpu6050_get_normal(&normal);

        // ESP_LOGI(TAG, "x:%.2f, y:%.2f, z:%.2f\n", normal.x, normal.y, normal.z);
    }
}