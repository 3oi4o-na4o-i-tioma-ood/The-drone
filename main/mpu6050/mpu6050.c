/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_log.h"
#include <string.h>

#define ALPHA 0.99f             /*!< Weight of gyroscope */
#define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */

/* MPU6050 register */
#define MPU6050_GYRO_CONFIG 0x1Bu
#define MPU6050_ACCEL_CONFIG 0x1Cu
#define MPU6050_INTR_PIN_CFG 0x37u
#define MPU6050_INTR_ENABLE 0x38u
#define MPU6050_INTR_STATUS 0x3Au
#define MPU6050_ACCEL_XOUT_H 0x3Bu
#define MPU6050_GYRO_XOUT_H 0x43u
#define MPU6050_TEMP_XOUT_H 0x41u
#define MPU6050_PWR_MGMT_1 0x6Bu
#define MPU6050_WHO_AM_I 0x75u

const char *TAG = "mpu6050";

const uint8_t MPU6050_DATA_RDY_INT_BIT = (uint8_t)BIT0;
const uint8_t MPU6050_I2C_MASTER_INT_BIT = (uint8_t)BIT3;
const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT = (uint8_t)BIT4;
const uint8_t MPU6050_MOT_DETECT_INT_BIT = (uint8_t)BIT6;
const uint8_t MPU6050_ALL_INTERRUPTS = (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

typedef struct
{
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_addr;
    uint32_t counter;
    float dt; /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
    uint8_t *dmpPacketBuffer;
    uint16_t dmpPacketSize;
} mpu6050_dev_t;

static esp_err_t mpu6050_write(mpu6050_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mpu6050_read(mpu6050_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

mpu6050_handle_t mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
{
    mpu6050_dev_t *sensor = (mpu6050_dev_t *)calloc(1, sizeof(mpu6050_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
    sensor->dmpPacketSize = 0;
    return (mpu6050_handle_t)sensor;
}

void mpu6050_delete(mpu6050_handle_t sensor)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;
    free(sens);
}

esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid)
{
    return mpu6050_read(sensor, MPU6050_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret)
    {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_sleep(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret)
    {
        return ret;
    }
    tmp |= BIT6;
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3, acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_read(sensor, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs)
    {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs)
    {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t *const interrupt_configuration)
{
    esp_err_t ret = ESP_OK;

    if (NULL == interrupt_configuration)
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin))
    {
        // Set GPIO connected to MPU6050 INT pin only when user configures interrupts.
        mpu6050_dev_t *sensor_device = (mpu6050_dev_t *)sensor;
        sensor_device->int_pin = interrupt_configuration->interrupt_pin;
    }
    else
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    uint8_t int_pin_cfg = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level)
    {
        int_pin_cfg |= BIT7;
    }

    if (INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode)
    {
        int_pin_cfg |= BIT6;
    }

    if (INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch)
    {
        int_pin_cfg |= BIT5;
    }

    if (INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior)
    {
        int_pin_cfg |= BIT4;
    }

    ret = mpu6050_write(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    gpio_int_type_t gpio_intr_type;

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level)
    {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
    }
    else
    {
        gpio_intr_type = GPIO_INTR_POSEDGE;
    }

    gpio_config_t int_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (BIT0 << interrupt_configuration->interrupt_pin)};

    ret = gpio_config(&int_gpio_config);

    return ret;
}

esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr)
{
    esp_err_t ret;
    mpu6050_dev_t *sensor_device = (mpu6050_dev_t *)sensor;

    if (NULL == sensor_device)
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = gpio_isr_handler_add(
        sensor_device->int_pin,
        ((gpio_isr_t) * (isr)),
        ((void *)sensor));

    if (ESP_OK != ret)
    {
        return ret;
    }

    ret = gpio_intr_enable(sensor_device->int_pin);

    return ret;
}

esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    if (enabled_interrupts != interrupt_sources)
    {

        enabled_interrupts |= interrupt_sources;

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret)
    {
        return ret;
    }

    if (0 != (enabled_interrupts & interrupt_sources))
    {
        enabled_interrupts &= (~interrupt_sources);

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t sensor, uint8_t *const out_intr_status)
{
    esp_err_t ret;

    if (NULL == out_intr_status)
    {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = mpu6050_read(sensor, MPU6050_INTR_STATUS, out_intr_status, 1);

    return ret;
}

inline uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status)
{
    return (MPU6050_DATA_RDY_INT_BIT == (MPU6050_DATA_RDY_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status)
{
    return (uint8_t)(MPU6050_I2C_MASTER_INT_BIT == (MPU6050_I2C_MASTER_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status)
{
    return (uint8_t)(MPU6050_FIFO_OVERFLOW_INT_BIT == (MPU6050_FIFO_OVERFLOW_INT_BIT & interrupt_status));
}

esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;

    ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = mpu6050_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK)
    {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;

    ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
    if (ret != ESP_OK)
    {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value)
{
    uint8_t data_rd[2];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                       const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    mpu6050_dev_t *sens = (mpu6050_dev_t *)sensor;

    sens->counter++;
    if (sens->counter == 1)
    {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);

    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;

    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}

void mpu6050_write_bit(mpu6050_handle_t sensor, const uint8_t reg_addr, uint8_t bit, bool data)
{
    uint8_t byte = 0;
    mpu6050_read(sensor, reg_addr, &byte, 1);
    byte = (data == 0
               ? (byte & ~(1 << bit))
               : (byte | (1 << bit)));
    mpu6050_write(sensor, reg_addr, &byte, 1);
}


/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool mpu6050_write_bits(mpu6050_handle_t sensor, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (mpu6050_read(sensor, regAddr, &b, 1) == ESP_OK) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return mpu6050_write(sensor, regAddr, &b, 1);
    } else {
        return false;
    }
}

// void mpu6050_read_bit(mpu6050_handle_t sensor, const uint8_t reg_addr, uint8_t bit, uint8_t* buffer)
// {
//     uint8_t byte = 0;
//     mpu6050_read(sensor, reg_addr, &byte, 1);
//     *buffer = byte & (1 << bit);
// }

uint8_t mpu6050_read_bit(mpu6050_handle_t sensor, const uint8_t reg_addr, uint8_t bit)
{
    uint8_t byte = 0;
    mpu6050_read(sensor, reg_addr, &byte, 1);
    return byte & (1 << bit);
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 */
void mpu6050_read_bits(mpu6050_handle_t sensor, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t b;
    if (mpu6050_read(sensor, regAddr, &b, 1) == ESP_OK) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
}

// BANK_SEL register

void mpu6050_set_memory_bank(mpu6050_handle_t sensor, uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    mpu6050_write(sensor, MPU6050_RA_BANK_SEL, &bank, 1);
}

// MEM_START_ADDR register

void mpu6050_set_memory_start_address(mpu6050_handle_t sensor, uint8_t address) {
    mpu6050_write(sensor, MPU6050_RA_MEM_START_ADDR, &address, 1);
}


bool mpu6050_write_memory_block(mpu6050_handle_t sensor, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem) {
    mpu6050_set_memory_bank(sensor, bank, false, false);
    mpu6050_set_memory_start_address(sensor, address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer=0;
    uint8_t *progBuffer=0;
    uint16_t i;
    uint8_t j;
    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        mpu6050_write(sensor, MPU6050_RA_MEM_R_W, progBuffer, chunkSize);

        // verify data if needed
        if (verify && verifyBuffer) {
            mpu6050_set_memory_bank(sensor, bank, false, false);
            mpu6050_set_memory_start_address(sensor, address);
            mpu6050_read(sensor, MPU6050_RA_MEM_R_W, verifyBuffer, chunkSize);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                /*Serial.print("Block write verification error, bank ");
                Serial.print(bank, DEC);
                Serial.print(", address ");
                Serial.print(address, DEC);
                Serial.print("!\nExpected:");
                for (j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (progBuffer[j] < 16) Serial.print("0");
                    Serial.print(progBuffer[j], HEX);
                }
                Serial.print("\nReceived:");
                for (uint8_t j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (verifyBuffer[i + j] < 16) Serial.print("0");
                    Serial.print(verifyBuffer[i + j], HEX);
                }
                Serial.print("\n");*/
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_set_memory_bank(sensor, bank, false, false);
            mpu6050_set_memory_start_address(sensor, address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return true;
}



// DMP_CFG_1 register

uint8_t mpu6050_get_DMP_config1(mpu6050_handle_t sensor) {
    uint8_t buffer;
    mpu6050_read(sensor, MPU6050_RA_DMP_CFG_1, &buffer, 1);
    return buffer;
}
void mpu6050_set_DMP_config1(mpu6050_handle_t sensor, uint8_t config) {
    mpu6050_write(sensor, MPU6050_RA_DMP_CFG_1, &config, 1);
}

// DMP_CFG_2 register

uint8_t mpu6050_get_DMP_config2(mpu6050_handle_t sensor) {
    uint8_t buffer;
    mpu6050_read(sensor, MPU6050_RA_DMP_CFG_2, &buffer, 1);
    return buffer;
}
void mpu6050_set_DMP_config2(mpu6050_handle_t sensor, uint8_t config) {
    mpu6050_write(sensor, MPU6050_RA_DMP_CFG_2, &config, 1);
}



// XG_OFFS_TC register

uint8_t mpu6050_get_OTP_bank_valid(mpu6050_handle_t sensor) {
    return mpu6050_read_bit(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT);
}
void mpu6050_set_OTP_bank_valid(mpu6050_handle_t sensor, bool enabled) {
    mpu6050_write_bit(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t mpu6050_get_X_gyro_offset_TC(mpu6050_handle_t sensor) {
    uint8_t buffer;
    mpu6050_read_bits(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, &buffer);
    return buffer;
}
void mpu6050_set_X_gyro_offset_TC(mpu6050_handle_t sensor, int8_t offset) {
    mpu6050_write_bits(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}





bool mpu6050_get_DMP_enabled(mpu6050_handle_t sensor) {
    return mpu6050_read_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT);
}
void mpu6050_set_DMP_enabled(mpu6050_handle_t sensor, bool enabled) {
    mpu6050_write_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
void mpu6050_reset_DMP(mpu6050_handle_t sensor) {
    mpu6050_write_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}


/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void mpu6050_set_FIFO_enabled(mpu6050_handle_t sensor, bool enabled) {
    mpu6050_write_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void mpu6050_reset_FIFO(mpu6050_handle_t sensor)
{
    mpu6050_write_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}


/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void mpu6050_set_full_scale_gyro_range(mpu6050_handle_t sensor, uint8_t range) {
    mpu6050_write_bits(sensor, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void mpu6050_set_DLPF_mode(mpu6050_handle_t sensor, uint8_t mode) {
    mpu6050_write_bits(sensor, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void mpu6050_set_external_frame_sync(mpu6050_handle_t sensor, uint8_t sync) {
    mpu6050_write_bits(sensor, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void mpu6050_set_rate(mpu6050_handle_t sensor, uint8_t rate) {
    mpu6050_write(sensor, MPU6050_RA_SMPLRT_DIV, &rate, 1);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void mpu6050_set_int_enabled(mpu6050_handle_t sensor, uint8_t enabled) {
    mpu6050_write(sensor, MPU6050_RA_INT_ENABLE, enabled, 1);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void mpu6050_set_clock_source(mpu6050_handle_t sensor, uint8_t source) {
    mpu6050_write_bits(sensor, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}


uint8_t mpu6050_read_memory_byte(mpu6050_handle_t sensor) {
    uint8_t buffer;
    mpu6050_read(sensor, MPU6050_RA_MEM_R_W, &buffer, 1);
    return buffer;
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void mpu6050_set_sleep_enabled(mpu6050_handle_t sensor, bool enabled) {
    mpu6050_write_bit(sensor, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void mpu6050_reset(mpu6050_handle_t sensor) {
    mpu6050_write_bit(sensor, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}





esp_err_t mpu6050_dmp_initialize(mpu6050_handle_t sensor)
{
    // reset device
    ESP_LOGI(TAG, "\n\nResetting MPU6050...");
    mpu6050_reset(sensor);
    vTaskDelay(50 / portTICK_PERIOD_MS); // wait after reset

    // enable sleep mode and wake cycle
    /*Serial.println(TAG, "Enabling sleep mode..."));
    setSleepEnabled(true);
    Serial.println(TAG, "Enabling wake cycle..."));
    setWakeCycleEnabled(true);*/

    // disable sleep mode
    mpu6050_set_sleep_enabled(sensor, false);

    // get MPU hardware revision
    mpu6050_set_memory_bank(sensor, 0x10, true, true);
    mpu6050_set_memory_start_address(sensor, 0x06);
    ESP_LOGI(TAG, "Checking hardware revision...");
    ESP_LOGI(TAG, "Revision @ user[16][6] = ");
    ESP_LOGI(TAG, "%d", mpu6050_read_memory_byte(sensor));
    ESP_LOGI(TAG, "Resetting memory bank selection to 0...");
    mpu6050_set_memory_bank(sensor, 0, false, false);

    // check OTP bank valid
    ESP_LOGI(TAG, "Reading OTP bank valid flag...");
    ESP_LOGI(TAG, "OTP bank is ");
    ESP_LOGI(TAG, "%s", mpu6050_get_OTP_bank_valid(sensor) ? "valid!" : "invalid!");

    // setup weird slave stuff (?)
    // ESP_LOGI(TAG, "Setting slave 0 address to 0x7F..."));
    // setSlaveAddress(0, 0x7F);
    // ESP_LOGI(TAG, "Disabling I2C Master mode..."));
    // setI2CMasterModeEnabled(false);
    // ESP_LOGI(TAG, "Setting slave 0 address to 0x68 (self)..."));
    // setSlaveAddress(0, 0x68);
    // DEBUG_PRINTLN(TAG, "Resetting I2C Master control..."));
    // resetI2CMaster();
    // delay(20);
    ESP_LOGI(TAG, "Setting clock source to Z Gyro...");
    mpu6050_set_clock_source(sensor, MPU6050_CLOCK_PLL_ZGYRO);

    ESP_LOGI(TAG, "Setting DMP and FIFO_OFLOW interrupts enabled...");
    mpu6050_set_int_enabled(sensor, 1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT | 1 << MPU6050_INTERRUPT_DMP_INT_BIT);

    ESP_LOGI(TAG, "Setting sample rate to 200Hz...");
    mpu6050_set_rate(sensor, 4); // 1khz / (1 + 4) = 200 Hz

    ESP_LOGI(TAG, "Setting external frame sync to TEMP_OUT_L[0]...");
    mpu6050_set_external_frame_sync(sensor, MPU6050_EXT_SYNC_TEMP_OUT_L);

    ESP_LOGI(TAG, "Setting DLPF bandwidth to 42Hz...");
    mpu6050_set_DLPF_mode(sensor, MPU6050_DLPF_BW_42);

    ESP_LOGI(TAG, "Setting gyro sensitivity to +/- 2000 deg/sec...");
    mpu6050_set_full_scale_gyro_range(sensor, MPU6050_GYRO_FS_2000);

    // load DMP code into memory banks
    // ESP_LOGI(TAG, "Writing DMP code to MPU memory banks (");
    // ESP_LOGI(TAG, "%d", MPU6050_DMP_CODE_SIZE);
    // ESP_LOGI(TAG, " bytes)");
    // if (!writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) return 1; // Failed
    // ESP_LOGI(TAG, "Success! DMP code written and verified.");

    // Set the FIFO Rate Divisor int the DMP Firmware Memory
    unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
    mpu6050_write_memory_block(sensor, dmpUpdate, 0x02, 0x02, 0x16, true, false); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

    // write start address MSB into register
    mpu6050_set_DMP_config1(sensor, 0x03);
    // write start address LSB into register
    mpu6050_set_DMP_config2(sensor, 0x00);

    ESP_LOGI(TAG, "Clearing OTP Bank flag...");
    mpu6050_set_OTP_bank_valid(sensor, false);

    //ESP_LOGI(TAG, "Setting motion detection threshold to 2...");
    //setMotionDetectionThreshold(2);

    //ESP_LOGI(TAG, "Setting zero-motion detection threshold to 156...");
    //setZeroMotionDetectionThreshold(156);

    //ESP_LOGI(TAG, "Setting motion detection duration to 80...");
    //setMotionDetectionDuration(80);

    //ESP_LOGI(TAG, "Setting zero-motion detection duration to 0...");
    //setZeroMotionDetectionDuration(0);
    ESP_LOGI(TAG, "Enabling FIFO...");
    mpu6050_set_FIFO_enabled(sensor, true);

    ESP_LOGI(TAG, "Resetting DMP...");
    mpu6050_reset_DMP(sensor);

    ESP_LOGI(TAG, "DMP is good to go! Finally.");

    ESP_LOGI(TAG, "Disabling DMP (you turn it on later)...");
    mpu6050_set_DMP_enabled(sensor, false);

    ESP_LOGI(TAG, "Setting up internal 42-byte (default) DMP packet buffer...");
    //((mpu6050_dev_t*)sensor)->dmpPacketSize = 42;

    ESP_LOGI(TAG, "Resetting FIFO and clearing INT status one last time...");
    mpu6050_reset_FIFO(sensor);
    // getIntStatus();

    return 0; // success
}