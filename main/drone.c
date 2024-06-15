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

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";

static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

/**
 * @brief i2c master initialization
 */
static void mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t mpu6050_deviceid;

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

    mpu6050_dmp_initialize(mpu6050);

    mpu6050_set_DMP_enabled(mpu6050, true);

    // ret = mpu6050_get_acce(mpu6050, &acce);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

    // ret = mpu6050_get_gyro(mpu6050, &gyro);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    // ret = mpu6050_get_temp(mpu6050, &temp);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    // ESP_LOGI(TAG, "t:%.2f \n", temp.temp);
}

void mpu6050_get_normal(VectorFloat* const normal)
{
    int16_t quaternion[4];

    mpu6050_dmp_get_quaternion(quaternion);

    Quaternion q = {
        (float)quaternion[0] / 16384.0f,
        (float)quaternion[1] / 16384.0f,
        (float)quaternion[2] / 16384.0f,
        (float)quaternion[3] / 16384.0f};

    normal.x = 0;
    normal.y = 0;
    normal.z = 1;
    rotateWithQuaternion(&normal, &q);
}

void app_main()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Hi");

    mpu6050_init();

    esp_err_t ret;
    // mpu6050_acce_value_t acce;

    while (true)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // ret = mpu6050_get_acce(mpu6050, &acce);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

        const int fifoReadResult = mpu6050_get_current_FIFO_packet(mpu6050);

        // ESP_LOGI(TAG, "fifoReadResult: %d", fifoReadResult);

        VectorFloat normal;
        mpu6050_get_normal(&normal);

        ESP_LOGI(TAG, "x:%.2f, y:%.2f, z:%.2f\n", normal.x, normal.y, normal.z);
    }
}