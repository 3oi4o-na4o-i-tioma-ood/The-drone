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
#include "esp_timer.h"
#include "esp_log.h"
#include "unity.h"

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

uint8_t fifoPacket[42];

static const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] = {
    /* bank # 0 */
    0xFB,
    0x00,
    0x00,
    0x3E,
    0x00,
    0x0B,
    0x00,
    0x36,
    0x00,
    0x01,
    0x00,
    0x02,
    0x00,
    0x03,
    0x00,
    0x00,
    0x00,
    0x65,
    0x00,
    0x54,
    0xFF,
    0xEF,
    0x00,
    0x00,
    0xFA,
    0x80,
    0x00,
    0x0B,
    0x12,
    0x82,
    0x00,
    0x01,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x28,
    0x00,
    0x00,
    0xFF,
    0xFF,
    0x45,
    0x81,
    0xFF,
    0xFF,
    0xFA,
    0x72,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x03,
    0xE8,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x01,
    0x7F,
    0xFF,
    0xFF,
    0xFE,
    0x80,
    0x01,
    0x00,
    0x1B,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
    0x00,
    0x02,
    0xCB,
    0x47,
    0xA2,
    0x20,
    0x00,
    0x00,
    0x00,
    0x20,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
    0x00,
    0x60,
    0x00,
    0x00,
    0x00,
    0x41,
    0xFF,
    0x00,
    0x00,
    0x00,
    0x00,
    0x0B,
    0x2A,
    0x00,
    0x00,
    0x16,
    0x55,
    0x00,
    0x00,
    0x21,
    0x82,
    0xFD,
    0x87,
    0x26,
    0x50,
    0xFD,
    0x80,
    0x00,
    0x00,
    0x00,
    0x1F,
    0x00,
    0x00,
    0x00,
    0x05,
    0x80,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x03,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x04,
    0x6F,
    0x00,
    0x02,
    0x65,
    0x32,
    0x00,
    0x00,
    0x5E,
    0xC0,
    0x40,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xFB,
    0x8C,
    0x6F,
    0x5D,
    0xFD,
    0x5D,
    0x08,
    0xD9,
    0x00,
    0x7C,
    0x73,
    0x3B,
    0x00,
    0x6C,
    0x12,
    0xCC,
    0x32,
    0x00,
    0x13,
    0x9D,
    0x32,
    0x00,
    0xD0,
    0xD6,
    0x32,
    0x00,
    0x08,
    0x00,
    0x40,
    0x00,
    0x01,
    0xF4,
    0xFF,
    0xE6,
    0x80,
    0x79,
    0x02,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xD0,
    0xD6,
    0x00,
    0x00,
    0x27,
    0x10,
    /* bank # 1 */
    0xFB,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x01,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xFA,
    0x36,
    0xFF,
    0xBC,
    0x30,
    0x8E,
    0x00,
    0x05,
    0xFB,
    0xF0,
    0xFF,
    0xD9,
    0x5B,
    0xC8,
    0xFF,
    0xD0,
    0x9A,
    0xBE,
    0x00,
    0x00,
    0x10,
    0xA9,
    0xFF,
    0xF4,
    0x1E,
    0xB2,
    0x00,
    0xCE,
    0xBB,
    0xF7,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00,
    0x00,
    0x04,
    0x00,
    0x02,
    0x00,
    0x02,
    0x02,
    0x00,
    0x00,
    0x0C,
    0xFF,
    0xC2,
    0x80,
    0x00,
    0x00,
    0x01,
    0x80,
    0x00,
    0x00,
    0xCF,
    0x80,
    0x00,
    0x40,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x06,
    0x00,
    0x00,
    0x00,
    0x00,
    0x14,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x09,
    0x23,
    0xA1,
    0x35,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x03,
    0x3F,
    0x68,
    0xB6,
    0x79,
    0x35,
    0x28,
    0xBC,
    0xC6,
    0x7E,
    0xD1,
    0x6C,
    0x80,
    0x00,
    0xFF,
    0xFF,
    0x40,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xB2,
    0x6A,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x3F,
    0xF0,
    0x00,
    0x00,
    0x00,
    0x30,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
    0x00,
    0x25,
    0x4D,
    0x00,
    0x2F,
    0x70,
    0x6D,
    0x00,
    0x00,
    0x05,
    0xAE,
    0x00,
    0x0C,
    0x02,
    0xD0,
    /* bank # 2 */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x65,
    0x00,
    0x54,
    0xFF,
    0xEF,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00,
    0x44,
    0x00,
    0x01,
    0x00,
    0x05,
    0x8B,
    0xC1,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x65,
    0x00,
    0x00,
    0x00,
    0x54,
    0x00,
    0x00,
    0xFF,
    0xEF,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x1B,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x1B,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    /* bank # 3 */
    0xD8,
    0xDC,
    0xBA,
    0xA2,
    0xF1,
    0xDE,
    0xB2,
    0xB8,
    0xB4,
    0xA8,
    0x81,
    0x91,
    0xF7,
    0x4A,
    0x90,
    0x7F,
    0x91,
    0x6A,
    0xF3,
    0xF9,
    0xDB,
    0xA8,
    0xF9,
    0xB0,
    0xBA,
    0xA0,
    0x80,
    0xF2,
    0xCE,
    0x81,
    0xF3,
    0xC2,
    0xF1,
    0xC1,
    0xF2,
    0xC3,
    0xF3,
    0xCC,
    0xA2,
    0xB2,
    0x80,
    0xF1,
    0xC6,
    0xD8,
    0x80,
    0xBA,
    0xA7,
    0xDF,
    0xDF,
    0xDF,
    0xF2,
    0xA7,
    0xC3,
    0xCB,
    0xC5,
    0xB6,
    0xF0,
    0x87,
    0xA2,
    0x94,
    0x24,
    0x48,
    0x70,
    0x3C,
    0x95,
    0x40,
    0x68,
    0x34,
    0x58,
    0x9B,
    0x78,
    0xA2,
    0xF1,
    0x83,
    0x92,
    0x2D,
    0x55,
    0x7D,
    0xD8,
    0xB1,
    0xB4,
    0xB8,
    0xA1,
    0xD0,
    0x91,
    0x80,
    0xF2,
    0x70,
    0xF3,
    0x70,
    0xF2,
    0x7C,
    0x80,
    0xA8,
    0xF1,
    0x01,
    0xB0,
    0x98,
    0x87,
    0xD9,
    0x43,
    0xD8,
    0x86,
    0xC9,
    0x88,
    0xBA,
    0xA1,
    0xF2,
    0x0E,
    0xB8,
    0x97,
    0x80,
    0xF1,
    0xA9,
    0xDF,
    0xDF,
    0xDF,
    0xAA,
    0xDF,
    0xDF,
    0xDF,
    0xF2,
    0xAA,
    0x4C,
    0xCD,
    0x6C,
    0xA9,
    0x0C,
    0xC9,
    0x2C,
    0x97,
    0x97,
    0x97,
    0x97,
    0xF1,
    0xA9,
    0x89,
    0x26,
    0x46,
    0x66,
    0xB0,
    0xB4,
    0xBA,
    0x80,
    0xAC,
    0xDE,
    0xF2,
    0xCA,
    0xF1,
    0xB2,
    0x8C,
    0x02,
    0xA9,
    0xB6,
    0x98,
    0x00,
    0x89,
    0x0E,
    0x16,
    0x1E,
    0xB8,
    0xA9,
    0xB4,
    0x99,
    0x2C,
    0x54,
    0x7C,
    0xB0,
    0x8A,
    0xA8,
    0x96,
    0x36,
    0x56,
    0x76,
    0xF1,
    0xB9,
    0xAF,
    0xB4,
    0xB0,
    0x83,
    0xC0,
    0xB8,
    0xA8,
    0x97,
    0x11,
    0xB1,
    0x8F,
    0x98,
    0xB9,
    0xAF,
    0xF0,
    0x24,
    0x08,
    0x44,
    0x10,
    0x64,
    0x18,
    0xF1,
    0xA3,
    0x29,
    0x55,
    0x7D,
    0xAF,
    0x83,
    0xB5,
    0x93,
    0xAF,
    0xF0,
    0x00,
    0x28,
    0x50,
    0xF1,
    0xA3,
    0x86,
    0x9F,
    0x61,
    0xA6,
    0xDA,
    0xDE,
    0xDF,
    0xD9,
    0xFA,
    0xA3,
    0x86,
    0x96,
    0xDB,
    0x31,
    0xA6,
    0xD9,
    0xF8,
    0xDF,
    0xBA,
    0xA6,
    0x8F,
    0xC2,
    0xC5,
    0xC7,
    0xB2,
    0x8C,
    0xC1,
    0xB8,
    0xA2,
    0xDF,
    0xDF,
    0xDF,
    0xA3,
    0xDF,
    0xDF,
    0xDF,
    0xD8,
    0xD8,
    0xF1,
    0xB8,
    0xA8,
    0xB2,
    0x86,
    /* bank # 4 */
    0xB4,
    0x98,
    0x0D,
    0x35,
    0x5D,
    0xB8,
    0xAA,
    0x98,
    0xB0,
    0x87,
    0x2D,
    0x35,
    0x3D,
    0xB2,
    0xB6,
    0xBA,
    0xAF,
    0x8C,
    0x96,
    0x19,
    0x8F,
    0x9F,
    0xA7,
    0x0E,
    0x16,
    0x1E,
    0xB4,
    0x9A,
    0xB8,
    0xAA,
    0x87,
    0x2C,
    0x54,
    0x7C,
    0xB9,
    0xA3,
    0xDE,
    0xDF,
    0xDF,
    0xA3,
    0xB1,
    0x80,
    0xF2,
    0xC4,
    0xCD,
    0xC9,
    0xF1,
    0xB8,
    0xA9,
    0xB4,
    0x99,
    0x83,
    0x0D,
    0x35,
    0x5D,
    0x89,
    0xB9,
    0xA3,
    0x2D,
    0x55,
    0x7D,
    0xB5,
    0x93,
    0xA3,
    0x0E,
    0x16,
    0x1E,
    0xA9,
    0x2C,
    0x54,
    0x7C,
    0xB8,
    0xB4,
    0xB0,
    0xF1,
    0x97,
    0x83,
    0xA8,
    0x11,
    0x84,
    0xA5,
    0x09,
    0x98,
    0xA3,
    0x83,
    0xF0,
    0xDA,
    0x24,
    0x08,
    0x44,
    0x10,
    0x64,
    0x18,
    0xD8,
    0xF1,
    0xA5,
    0x29,
    0x55,
    0x7D,
    0xA5,
    0x85,
    0x95,
    0x02,
    0x1A,
    0x2E,
    0x3A,
    0x56,
    0x5A,
    0x40,
    0x48,
    0xF9,
    0xF3,
    0xA3,
    0xD9,
    0xF8,
    0xF0,
    0x98,
    0x83,
    0x24,
    0x08,
    0x44,
    0x10,
    0x64,
    0x18,
    0x97,
    0x82,
    0xA8,
    0xF1,
    0x11,
    0xF0,
    0x98,
    0xA2,
    0x24,
    0x08,
    0x44,
    0x10,
    0x64,
    0x18,
    0xDA,
    0xF3,
    0xDE,
    0xD8,
    0x83,
    0xA5,
    0x94,
    0x01,
    0xD9,
    0xA3,
    0x02,
    0xF1,
    0xA2,
    0xC3,
    0xC5,
    0xC7,
    0xD8,
    0xF1,
    0x84,
    0x92,
    0xA2,
    0x4D,
    0xDA,
    0x2A,
    0xD8,
    0x48,
    0x69,
    0xD9,
    0x2A,
    0xD8,
    0x68,
    0x55,
    0xDA,
    0x32,
    0xD8,
    0x50,
    0x71,
    0xD9,
    0x32,
    0xD8,
    0x70,
    0x5D,
    0xDA,
    0x3A,
    0xD8,
    0x58,
    0x79,
    0xD9,
    0x3A,
    0xD8,
    0x78,
    0x93,
    0xA3,
    0x4D,
    0xDA,
    0x2A,
    0xD8,
    0x48,
    0x69,
    0xD9,
    0x2A,
    0xD8,
    0x68,
    0x55,
    0xDA,
    0x32,
    0xD8,
    0x50,
    0x71,
    0xD9,
    0x32,
    0xD8,
    0x70,
    0x5D,
    0xDA,
    0x3A,
    0xD8,
    0x58,
    0x79,
    0xD9,
    0x3A,
    0xD8,
    0x78,
    0xA8,
    0x8A,
    0x9A,
    0xF0,
    0x28,
    0x50,
    0x78,
    0x9E,
    0xF3,
    0x88,
    0x18,
    0xF1,
    0x9F,
    0x1D,
    0x98,
    0xA8,
    0xD9,
    0x08,
    0xD8,
    0xC8,
    0x9F,
    0x12,
    0x9E,
    0xF3,
    0x15,
    0xA8,
    0xDA,
    0x12,
    0x10,
    0xD8,
    0xF1,
    0xAF,
    0xC8,
    0x97,
    0x87,
    /* bank # 5 */
    0x34,
    0xB5,
    0xB9,
    0x94,
    0xA4,
    0x21,
    0xF3,
    0xD9,
    0x22,
    0xD8,
    0xF2,
    0x2D,
    0xF3,
    0xD9,
    0x2A,
    0xD8,
    0xF2,
    0x35,
    0xF3,
    0xD9,
    0x32,
    0xD8,
    0x81,
    0xA4,
    0x60,
    0x60,
    0x61,
    0xD9,
    0x61,
    0xD8,
    0x6C,
    0x68,
    0x69,
    0xD9,
    0x69,
    0xD8,
    0x74,
    0x70,
    0x71,
    0xD9,
    0x71,
    0xD8,
    0xB1,
    0xA3,
    0x84,
    0x19,
    0x3D,
    0x5D,
    0xA3,
    0x83,
    0x1A,
    0x3E,
    0x5E,
    0x93,
    0x10,
    0x30,
    0x81,
    0x10,
    0x11,
    0xB8,
    0xB0,
    0xAF,
    0x8F,
    0x94,
    0xF2,
    0xDA,
    0x3E,
    0xD8,
    0xB4,
    0x9A,
    0xA8,
    0x87,
    0x29,
    0xDA,
    0xF8,
    0xD8,
    0x87,
    0x9A,
    0x35,
    0xDA,
    0xF8,
    0xD8,
    0x87,
    0x9A,
    0x3D,
    0xDA,
    0xF8,
    0xD8,
    0xB1,
    0xB9,
    0xA4,
    0x98,
    0x85,
    0x02,
    0x2E,
    0x56,
    0xA5,
    0x81,
    0x00,
    0x0C,
    0x14,
    0xA3,
    0x97,
    0xB0,
    0x8A,
    0xF1,
    0x2D,
    0xD9,
    0x28,
    0xD8,
    0x4D,
    0xD9,
    0x48,
    0xD8,
    0x6D,
    0xD9,
    0x68,
    0xD8,
    0xB1,
    0x84,
    0x0D,
    0xDA,
    0x0E,
    0xD8,
    0xA3,
    0x29,
    0x83,
    0xDA,
    0x2C,
    0x0E,
    0xD8,
    0xA3,
    0x84,
    0x49,
    0x83,
    0xDA,
    0x2C,
    0x4C,
    0x0E,
    0xD8,
    0xB8,
    0xB0,
    0xA8,
    0x8A,
    0x9A,
    0xF5,
    0x20,
    0xAA,
    0xDA,
    0xDF,
    0xD8,
    0xA8,
    0x40,
    0xAA,
    0xD0,
    0xDA,
    0xDE,
    0xD8,
    0xA8,
    0x60,
    0xAA,
    0xDA,
    0xD0,
    0xDF,
    0xD8,
    0xF1,
    0x97,
    0x86,
    0xA8,
    0x31,
    0x9B,
    0x06,
    0x99,
    0x07,
    0xAB,
    0x97,
    0x28,
    0x88,
    0x9B,
    0xF0,
    0x0C,
    0x20,
    0x14,
    0x40,
    0xB8,
    0xB0,
    0xB4,
    0xA8,
    0x8C,
    0x9C,
    0xF0,
    0x04,
    0x28,
    0x51,
    0x79,
    0x1D,
    0x30,
    0x14,
    0x38,
    0xB2,
    0x82,
    0xAB,
    0xD0,
    0x98,
    0x2C,
    0x50,
    0x50,
    0x78,
    0x78,
    0x9B,
    0xF1,
    0x1A,
    0xB0,
    0xF0,
    0x8A,
    0x9C,
    0xA8,
    0x29,
    0x51,
    0x79,
    0x8B,
    0x29,
    0x51,
    0x79,
    0x8A,
    0x24,
    0x70,
    0x59,
    0x8B,
    0x20,
    0x58,
    0x71,
    0x8A,
    0x44,
    0x69,
    0x38,
    0x8B,
    0x39,
    0x40,
    0x68,
    0x8A,
    0x64,
    0x48,
    0x31,
    0x8B,
    0x30,
    0x49,
    0x60,
    0xA5,
    0x88,
    0x20,
    0x09,
    0x71,
    0x58,
    0x44,
    0x68,
    /* bank # 6 */
    0x11,
    0x39,
    0x64,
    0x49,
    0x30,
    0x19,
    0xF1,
    0xAC,
    0x00,
    0x2C,
    0x54,
    0x7C,
    0xF0,
    0x8C,
    0xA8,
    0x04,
    0x28,
    0x50,
    0x78,
    0xF1,
    0x88,
    0x97,
    0x26,
    0xA8,
    0x59,
    0x98,
    0xAC,
    0x8C,
    0x02,
    0x26,
    0x46,
    0x66,
    0xF0,
    0x89,
    0x9C,
    0xA8,
    0x29,
    0x51,
    0x79,
    0x24,
    0x70,
    0x59,
    0x44,
    0x69,
    0x38,
    0x64,
    0x48,
    0x31,
    0xA9,
    0x88,
    0x09,
    0x20,
    0x59,
    0x70,
    0xAB,
    0x11,
    0x38,
    0x40,
    0x69,
    0xA8,
    0x19,
    0x31,
    0x48,
    0x60,
    0x8C,
    0xA8,
    0x3C,
    0x41,
    0x5C,
    0x20,
    0x7C,
    0x00,
    0xF1,
    0x87,
    0x98,
    0x19,
    0x86,
    0xA8,
    0x6E,
    0x76,
    0x7E,
    0xA9,
    0x99,
    0x88,
    0x2D,
    0x55,
    0x7D,
    0x9E,
    0xB9,
    0xA3,
    0x8A,
    0x22,
    0x8A,
    0x6E,
    0x8A,
    0x56,
    0x8A,
    0x5E,
    0x9F,
    0xB1,
    0x83,
    0x06,
    0x26,
    0x46,
    0x66,
    0x0E,
    0x2E,
    0x4E,
    0x6E,
    0x9D,
    0xB8,
    0xAD,
    0x00,
    0x2C,
    0x54,
    0x7C,
    0xF2,
    0xB1,
    0x8C,
    0xB4,
    0x99,
    0xB9,
    0xA3,
    0x2D,
    0x55,
    0x7D,
    0x81,
    0x91,
    0xAC,
    0x38,
    0xAD,
    0x3A,
    0xB5,
    0x83,
    0x91,
    0xAC,
    0x2D,
    0xD9,
    0x28,
    0xD8,
    0x4D,
    0xD9,
    0x48,
    0xD8,
    0x6D,
    0xD9,
    0x68,
    0xD8,
    0x8C,
    0x9D,
    0xAE,
    0x29,
    0xD9,
    0x04,
    0xAE,
    0xD8,
    0x51,
    0xD9,
    0x04,
    0xAE,
    0xD8,
    0x79,
    0xD9,
    0x04,
    0xD8,
    0x81,
    0xF3,
    0x9D,
    0xAD,
    0x00,
    0x8D,
    0xAE,
    0x19,
    0x81,
    0xAD,
    0xD9,
    0x01,
    0xD8,
    0xF2,
    0xAE,
    0xDA,
    0x26,
    0xD8,
    0x8E,
    0x91,
    0x29,
    0x83,
    0xA7,
    0xD9,
    0xAD,
    0xAD,
    0xAD,
    0xAD,
    0xF3,
    0x2A,
    0xD8,
    0xD8,
    0xF1,
    0xB0,
    0xAC,
    0x89,
    0x91,
    0x3E,
    0x5E,
    0x76,
    0xF3,
    0xAC,
    0x2E,
    0x2E,
    0xF1,
    0xB1,
    0x8C,
    0x5A,
    0x9C,
    0xAC,
    0x2C,
    0x28,
    0x28,
    0x28,
    0x9C,
    0xAC,
    0x30,
    0x18,
    0xA8,
    0x98,
    0x81,
    0x28,
    0x34,
    0x3C,
    0x97,
    0x24,
    0xA7,
    0x28,
    0x34,
    0x3C,
    0x9C,
    0x24,
    0xF2,
    0xB0,
    0x89,
    0xAC,
    0x91,
    0x2C,
    0x4C,
    0x6C,
    0x8A,
    0x9B,
    0x2D,
    0xD9,
    0xD8,
    0xD8,
    0x51,
    0xD9,
    0xD8,
    0xD8,
    0x79,
    /* bank # 7 */
    0xD9,
    0xD8,
    0xD8,
    0xF1,
    0x9E,
    0x88,
    0xA3,
    0x31,
    0xDA,
    0xD8,
    0xD8,
    0x91,
    0x2D,
    0xD9,
    0x28,
    0xD8,
    0x4D,
    0xD9,
    0x48,
    0xD8,
    0x6D,
    0xD9,
    0x68,
    0xD8,
    0xB1,
    0x83,
    0x93,
    0x35,
    0x3D,
    0x80,
    0x25,
    0xDA,
    0xD8,
    0xD8,
    0x85,
    0x69,
    0xDA,
    0xD8,
    0xD8,
    0xB4,
    0x93,
    0x81,
    0xA3,
    0x28,
    0x34,
    0x3C,
    0xF3,
    0xAB,
    0x8B,
    0xF8,
    0xA3,
    0x91,
    0xB6,
    0x09,
    0xB4,
    0xD9,
    0xAB,
    0xDE,
    0xFA,
    0xB0,
    0x87,
    0x9C,
    0xB9,
    0xA3,
    0xDD,
    0xF1,
    0x20,
    0x28,
    0x30,
    0x38,
    0x9A,
    0xF1,
    0x28,
    0x30,
    0x38,
    0x9D,
    0xF1,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xF2,
    0xA3,
    0xB4,
    0x90,
    0x80,
    0xF2,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xB2,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xB0,
    0x87,
    0xB5,
    0x99,
    0xF1,
    0x28,
    0x30,
    0x38,
    0x98,
    0xF1,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0x97,
    0xA3,
    0xA3,
    0xA3,
    0xA3,
    0xF3,
    0x9B,
    0xA3,
    0x30,
    0xDC,
    0xB9,
    0xA7,
    0xF1,
    0x26,
    0x26,
    0x26,
    0xFE,
    0xD8,
    0xFF,

};

const uint8_t MPU6050_DATA_RDY_INT_BIT = (uint8_t)BIT0;
const uint8_t MPU6050_I2C_MASTER_INT_BIT = (uint8_t)BIT3;
const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT = (uint8_t)BIT4;
const uint8_t MPU6050_MOT_DETECT_INT_BIT = (uint8_t)BIT6;
const uint8_t MPU6050_ALL_INTERRUPTS = (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

mpu6050_gyro_value_t gyro_error;

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

// esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value)
// {
//     esp_err_t ret;
//     float gyro_sensitivity;
//     mpu6050_raw_gyro_value_t raw_gyro;

//     ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
//     if (ret != ESP_OK)
//     {
//         return ret;
//     }
//     ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
//     if (ret != ESP_OK)
//     {
//         return ret;
//     }

//     gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
//     gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
//     gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
//     return ESP_OK;
// }

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

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity - gyro_error.gyro_x;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity - gyro_error.gyro_y;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity - gyro_error.gyro_z;
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
bool mpu6050_write_bits(mpu6050_handle_t sensor, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (mpu6050_read(sensor, regAddr, &b, 1) == ESP_OK)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask;                     // zero all non-important bits in data
        b &= ~(mask);                     // zero all important bits in existing byte
        b |= data;                        // combine data with existing byte
        return mpu6050_write(sensor, regAddr, &b, 1);
    }
    else
    {
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
void mpu6050_read_bits(mpu6050_handle_t sensor, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t b;
    if (mpu6050_read(sensor, regAddr, &b, 1) == ESP_OK)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
}

// BANK_SEL register

void mpu6050_set_memory_bank(mpu6050_handle_t sensor, uint8_t bank, bool prefetchEnabled, bool userBank)
{
    bank &= 0x1F;
    if (userBank)
        bank |= 0x20;
    if (prefetchEnabled)
        bank |= 0x40;
    mpu6050_write(sensor, MPU6050_RA_BANK_SEL, &bank, 1);
}

// MEM_START_ADDR register

void mpu6050_set_memory_start_address(mpu6050_handle_t sensor, uint8_t address)
{
    mpu6050_write(sensor, MPU6050_RA_MEM_START_ADDR, &address, 1);
}

bool mpu6050_write_memory_block(mpu6050_handle_t sensor, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem)
{
    mpu6050_set_memory_bank(sensor, bank, false, false);
    mpu6050_set_memory_start_address(sensor, address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer = 0;
    uint8_t *progBuffer = 0;
    uint16_t i;
    uint8_t j;
    if (verify)
        verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem)
        progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;)
    {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize)
            chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address)
            chunkSize = 256 - address;

        if (useProgMem)
        {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++)
                progBuffer[j] = pgm_read_byte(data + i + j);
        }
        else
        {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        mpu6050_write(sensor, MPU6050_RA_MEM_R_W, progBuffer, chunkSize);

        // verify data if needed
        if (verify && verifyBuffer)
        {
            mpu6050_set_memory_bank(sensor, bank, false, false);
            mpu6050_set_memory_start_address(sensor, address);
            mpu6050_read(sensor, MPU6050_RA_MEM_R_W, verifyBuffer, chunkSize);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0)
            {
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
                if (useProgMem)
                    free(progBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize)
        {
            if (address == 0)
                bank++;
            mpu6050_set_memory_bank(sensor, bank, false, false);
            mpu6050_set_memory_start_address(sensor, address);
        }
    }
    if (verify)
        free(verifyBuffer);
    if (useProgMem)
        free(progBuffer);
    return true;
}

// DMP_CFG_1 register

uint8_t mpu6050_get_DMP_config1(mpu6050_handle_t sensor)
{
    uint8_t buffer;
    mpu6050_read(sensor, MPU6050_RA_DMP_CFG_1, &buffer, 1);
    return buffer;
}
void mpu6050_set_DMP_config1(mpu6050_handle_t sensor, uint8_t config)
{
    mpu6050_write(sensor, MPU6050_RA_DMP_CFG_1, &config, 1);
}

// DMP_CFG_2 register

uint8_t mpu6050_get_DMP_config2(mpu6050_handle_t sensor)
{
    uint8_t buffer;
    mpu6050_read(sensor, MPU6050_RA_DMP_CFG_2, &buffer, 1);
    return buffer;
}
void mpu6050_set_DMP_config2(mpu6050_handle_t sensor, uint8_t config)
{
    mpu6050_write(sensor, MPU6050_RA_DMP_CFG_2, &config, 1);
}

// XG_OFFS_TC register

uint8_t mpu6050_get_OTP_bank_valid(mpu6050_handle_t sensor)
{
    return mpu6050_read_bit(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT);
}
void mpu6050_set_OTP_bank_valid(mpu6050_handle_t sensor, bool enabled)
{
    mpu6050_write_bit(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t mpu6050_get_X_gyro_offset_TC(mpu6050_handle_t sensor)
{
    uint8_t buffer;
    mpu6050_read_bits(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, &buffer);
    return buffer;
}
void mpu6050_set_X_gyro_offset_TC(mpu6050_handle_t sensor, int8_t offset)
{
    mpu6050_write_bits(sensor, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

bool mpu6050_get_DMP_enabled(mpu6050_handle_t sensor)
{
    return mpu6050_read_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT);
}
void mpu6050_set_DMP_enabled(mpu6050_handle_t sensor, bool enabled)
{
    mpu6050_write_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
void mpu6050_reset_DMP(mpu6050_handle_t sensor)
{
    mpu6050_write_bit(sensor, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void mpu6050_set_FIFO_enabled(mpu6050_handle_t sensor, bool enabled)
{
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
void mpu6050_set_full_scale_gyro_range(mpu6050_handle_t sensor, uint8_t range)
{
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
void mpu6050_set_DLPF_mode(mpu6050_handle_t sensor, uint8_t mode)
{
    mpu6050_write_bits(sensor, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void mpu6050_set_external_frame_sync(mpu6050_handle_t sensor, uint8_t sync)
{
    mpu6050_write_bits(sensor, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void mpu6050_set_rate(mpu6050_handle_t sensor, uint8_t rate)
{
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
void mpu6050_set_int_enabled(mpu6050_handle_t sensor, uint8_t enabled)
{
    ESP_LOGI(TAG, "mpu6050_set_int_enabled. enaled: %d", enabled);
    mpu6050_write(sensor, MPU6050_RA_INT_ENABLE, &enabled, 1);
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
void mpu6050_set_clock_source(mpu6050_handle_t sensor, uint8_t source)
{
    mpu6050_write_bits(sensor, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

uint8_t mpu6050_read_memory_byte(mpu6050_handle_t sensor)
{
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
void mpu6050_set_sleep_enabled(mpu6050_handle_t sensor, bool enabled)
{
    mpu6050_write_bit(sensor, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void mpu6050_reset(mpu6050_handle_t sensor)
{
    mpu6050_write_bit(sensor, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
uint8_t mpu6050_get_FIFO_byte(mpu6050_handle_t sensor)
{
    uint8_t buffer;
    mpu6050_read(sensor, MPU6050_RA_FIFO_R_W, &buffer, 1);
    return buffer;
}
void mpu6050_get_FIFO_bytes(mpu6050_handle_t sensor, uint8_t *data, uint8_t length)
{
    if (length > 0)
    {
        mpu6050_read(sensor, MPU6050_RA_FIFO_R_W, data, length);
    }
    else
    {
        *data = 0;
    }
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t mpu6050_get_FIFO_count(mpu6050_handle_t sensor)
{
    uint8_t buffer[2];
    mpu6050_read(sensor, MPU6050_RA_FIFO_COUNTH, buffer, 2);
    const uint16_t result = (((uint16_t)buffer[0]) << 8) | buffer[1];

    // ESP_LOGI(TAG, "mpu6050_get_FIFO_count: %d", result);
    return result;
}

/** Get latest byte from FIFO buffer no matter how much time has passed.
 * ===                  GetCurrentFIFOPacket                    ===
 * ================================================================
 * Returns 1) when nothing special was done
 *         2) when recovering from overflow
 *         0) when no valid data is available
 * ================================================================ */
int8_t mpu6050_get_current_FIFO_packet(mpu6050_handle_t sensor)
{ // overflow proof

    const int length = 42;

    int16_t fifoC;
    // This section of code is for when we allowed more than 1 packet to be acquired
    int64_t BreakTimer = esp_timer_get_time();
    bool packetReceived = false;
    do
    {
        if ((fifoC = mpu6050_get_FIFO_count(sensor)) > length)
        {

            if (fifoC > 200)
            {                               // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                mpu6050_reset_FIFO(sensor); // Fixes any overflow corruption
                fifoC = 0;
                while (!(fifoC = mpu6050_get_FIFO_count(sensor)) && ((esp_timer_get_time() - BreakTimer) <= MPU6050_FIFO_DEFAULT_TIMEOUT))
                    ; // Get Next New Packet
            }
            else
            { // We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
                uint8_t Trash[I2CDEVLIB_WIRE_BUFFER_LENGTH];
                while ((fifoC = mpu6050_get_FIFO_count(sensor)) > length)
                {                           // Test each time just in case the MPU is writing to the FIFO Buffer
                    fifoC = fifoC - length; // Save the last packet
                    uint16_t RemoveBytes;
                    while (fifoC)
                    {                                                                                                // fifo count will reach zero so this is safe
                        RemoveBytes = (fifoC < I2CDEVLIB_WIRE_BUFFER_LENGTH) ? fifoC : I2CDEVLIB_WIRE_BUFFER_LENGTH; // Buffer Length is different than the packet length this will efficiently clear the buffer
                        mpu6050_get_FIFO_bytes(sensor, Trash, (uint8_t)RemoveBytes);
                        fifoC -= RemoveBytes;
                    }
                }
            }
        }
        if (!fifoC)
            return 0; // Called too early no data or we timed out after FIFO Reset
        // We have 1 packet
        packetReceived = fifoC == length;
        if (!packetReceived && (esp_timer_get_time() - BreakTimer) > MPU6050_FIFO_DEFAULT_TIMEOUT)
            return 0;
    } while (!packetReceived);
    mpu6050_get_FIFO_bytes(sensor, fifoPacket, length); // Get 1 packet
    return 1;
}

uint8_t mpu6050_dmp_get_quaternion(int16_t *buffer)
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    // if (fifoPacket == 0)
    //     return 1;
    buffer[0] = ((fifoPacket[0] << 8) | fifoPacket[1]);
    buffer[1] = ((fifoPacket[4] << 8) | fifoPacket[5]);
    buffer[2] = ((fifoPacket[8] << 8) | fifoPacket[9]);
    buffer[3] = ((fifoPacket[12] << 8) | fifoPacket[13]);
    return 0;
}

bool mpu6050_write_prog_memory_block(mpu6050_handle_t sensor, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify)
{
    return mpu6050_write_memory_block(sensor, data, dataSize, bank, address, verify, true);
}

/** Set motion detection event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void mpu6050_set_motion_detection_threshold(mpu6050_handle_t sensor, uint8_t threshold)
{
    mpu6050_write(sensor, MPU6050_RA_MOT_THR, &threshold, 1);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void mpu6050_set_zero_motion_detection_threshold(mpu6050_handle_t sensor, uint8_t threshold)
{
    mpu6050_write(sensor, MPU6050_RA_ZRMOT_THR, &threshold, 1);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void mpu6050_set_motion_detection_duration(mpu6050_handle_t sensor, uint8_t duration)
{
    mpu6050_write(sensor, MPU6050_RA_MOT_DUR, &duration, 1);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void mpu6050_set_zero_motion_detection_duration(mpu6050_handle_t sensor, uint8_t duration)
{
    mpu6050_write(sensor, MPU6050_RA_ZRMOT_DUR, &duration, 1);
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
    ESP_LOGI(TAG, "Writing DMP code to MPU memory banks (");
    ESP_LOGI(TAG, "%d", MPU6050_DMP_CODE_SIZE);
    ESP_LOGI(TAG, " bytes)");
    if (!mpu6050_write_prog_memory_block(sensor, dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true))
        return 1; // Failed
    ESP_LOGI(TAG, "Success! DMP code written and verified.");

    // Set the FIFO Rate Divisor int the DMP Firmware Memory
    unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
    mpu6050_write_memory_block(sensor, dmpUpdate, 0x02, 0x02, 0x16, true, false); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

    // write start address MSB into register
    mpu6050_set_DMP_config1(sensor, 0x03);
    // write start address LSB into register
    mpu6050_set_DMP_config2(sensor, 0x00);

    ESP_LOGI(TAG, "Clearing OTP Bank flag...");
    mpu6050_set_OTP_bank_valid(sensor, false);

    ESP_LOGI(TAG, "Setting motion detection threshold to 2...");
    mpu6050_set_motion_detection_threshold(sensor, 2);

    ESP_LOGI(TAG, "Setting zero-motion detection threshold to 156...");
    mpu6050_set_zero_motion_detection_threshold(sensor, 156);

    ESP_LOGI(TAG, "Setting motion detection duration to 80...");
    mpu6050_set_motion_detection_duration(sensor, 80);

    ESP_LOGI(TAG, "Setting zero-motion detection duration to 0...");
    mpu6050_set_zero_motion_detection_duration(sensor, 0);
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

    uint8_t dummyIntStatus;
    mpu6050_get_interrupt_status(sensor, &dummyIntStatus);

    return 0; // success
}

void mpu6050_calc_error(mpu6050_handle_t sensor)
{
    const int trials = 200;

    // TO DO: acceleration error
    //   accXError = 0;
    //   accYError = 0;
    mpu6050_gyro_value_t gyro_error_temp;

    gyro_error_temp.gyro_x = 0;
    gyro_error_temp.gyro_y = 0;
    gyro_error_temp.gyro_z = 0;

    mpu6050_gyro_value_t gyro;

    for (int c = 0; c < trials; ++c)
    {
        mpu6050_get_gyro(sensor, &gyro);

        // TO DO: acceleration error
        // accXError += atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI;
        // accYError += atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI;
        gyro_error_temp.gyro_x += gyro.gyro_x;
        gyro_error_temp.gyro_y += gyro.gyro_y;
        gyro_error_temp.gyro_z += gyro.gyro_z;

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // TO DO: acceleration error
    // accXError /= trials;
    // accYError /= trials;
    gyro_error.gyro_x = gyro_error_temp.gyro_x / trials;
    gyro_error.gyro_y = gyro_error_temp.gyro_y / trials;
    gyro_error.gyro_z = gyro_error_temp.gyro_z / trials;
}

static void i2c_bus_init(i2c_port_t port, int SDA_pin, int SCL_pin, int I2C_frequency)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)SDA_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)SCL_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_frequency;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(port, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

/**
 * @brief i2c master initialization
 */
esp_err_t mpu6050_init(mpu6050_handle_t *const mpu6050, i2c_port_t port, int SDA_pin, int SCL_pin, int I2C_frequency)
{
    esp_err_t ret;
    uint8_t mpu6050_deviceid;

    i2c_bus_init(port, SDA_pin, SCL_pin, I2C_frequency);
    *mpu6050 = mpu6050_create(port, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(*mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(*mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_get_deviceid(*mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

    mpu6050_dmp_initialize(*mpu6050);

    mpu6050_set_DMP_enabled(*mpu6050, true);

    mpu6050_calc_error(*mpu6050);

    return ret;
}