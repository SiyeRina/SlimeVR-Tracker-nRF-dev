/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef BNO08X_SPI_H
#define BNO08X_SPI_H

#include "sensor/sensor.h"
#include <zephyr/drivers/spi.h>

/* =========================================================================
 *  SHTP Transport Layer Constants
 * ========================================================================= */
#define BNO08X_SPI_SHTP_HEADER_SIZE    4
#define BNO08X_SPI_SHTP_MAX_PAYLOAD    256
#define BNO08X_SPI_SHTP_CRC_SIZE       1
#define BNO08X_SPI_SHTP_MAX_PACKET     (BNO08X_SPI_SHTP_HEADER_SIZE + BNO08X_SPI_SHTP_MAX_PAYLOAD + BNO08X_SPI_SHTP_CRC_SIZE)

/* SHTP Channels */
#define BNO08X_SPI_SHTP_CH_COMMAND     0
#define BNO08X_SPI_SHTP_CH_EXECUTABLE  1
#define BNO08X_SPI_SHTP_CH_CONTROL     2
#define BNO08X_SPI_SHTP_CH_INPUT       3

/* =========================================================================
 *  SH-2 Application Layer Commands
 * ========================================================================= */
#define BNO08X_SPI_CMD_PRODUCT_ID_REQUEST   0xF9
#define BNO08X_SPI_CMD_PRODUCT_ID_RESPONSE  0xF8
#define BNO08X_SPI_CMD_SET_FEATURE          0xFD
#define BNO08X_SPI_CMD_FEATURE_RESPONSE     0xFC
#define BNO08X_SPI_CMD_RESET                0x01

/* =========================================================================
 *  Product IDs
 * ========================================================================= */
#define BNO08X_SPI_PID_BNO085           0x0085
#define BNO08X_SPI_PID_BNO086           0x0086

/* =========================================================================
 *  Sensor Report IDs
 * ========================================================================= */
#define BNO08X_SPI_REPORT_GAME_ROTATION_VECTOR   0x05
#define BNO08X_SPI_REPORT_TEMPERATURE            0x07

/* =========================================================================
 *  Packet size for FIFO (4 floats quaternion + 1 float dt_ms)
 * ========================================================================= */
#define BNO08X_SPI_PACKET_SIZE          20

/* =========================================================================
 *  Function Declarations
 * ========================================================================= */
int bno08x_spi_init(float clock_rate, float accel_time, float gyro_time,
                    float *accel_actual_time, float *gyro_actual_time);
void bno08x_spi_shutdown(void);
void bno08x_spi_update_fs(float accel_range, float gyro_range,
                          float *accel_actual_range, float *gyro_actual_range);
int bno08x_spi_update_odr(float accel_time, float gyro_time,
                          float *accel_actual_time, float *gyro_actual_time);
uint16_t bno08x_spi_fifo_read(uint8_t *rawData, uint16_t len);
int bno08x_spi_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void bno08x_spi_accel_read(float a[3]);
void bno08x_spi_gyro_read(float g[3]);
float bno08x_spi_temp_read(void);
uint8_t bno08x_spi_setup_DRDY(uint16_t threshold);
uint8_t bno08x_spi_setup_WOM(void);
void bno08x_spi_get_quaternion(float q[4]);
int bno08x_spi_read_product_id(uint8_t *pid_low, uint8_t *pid_high);
int bno08x_spi_scan_probe(struct spi_dt_spec *spi_dev, uint8_t *reg, bool interface_register);

/* =========================================================================
 *  sensor_imu_t Instance
 * ========================================================================= */
extern const sensor_imu_t sensor_imu_bno08x_spi;

#endif /* BNO08X_SPI_H */