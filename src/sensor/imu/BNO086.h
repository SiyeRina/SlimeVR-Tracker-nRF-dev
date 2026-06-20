/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef BNO086_H
#define BNO086_H

#include "sensor/sensor.h"

#include <zephyr/drivers/i2c.h>

/* I2C addresses (SA0 pin selectable) */
#define BNO086_I2C_ADDR_DEFAULT  0x4A
#define BNO086_I2C_ADDR_ALT      0x4B

/* SHTP packet framing */
#define BNO086_SHTP_HEADER_SIZE  4
#define BNO086_SHTP_CRC_SIZE     1
#define BNO086_SHTP_MAX_PAYLOAD  252
#define BNO086_SHTP_MAX_PACKET   (BNO086_SHTP_HEADER_SIZE + BNO086_SHTP_MAX_PAYLOAD + BNO086_SHTP_CRC_SIZE)

/* SHTP channels */
#define BNO086_SHTP_CH_COMMAND     0
#define BNO086_SHTP_CH_EXECUTABLE  1
#define BNO086_SHTP_CH_CONTROL     2
#define BNO086_SHTP_CH_INPUT       3  /* sensor reports arrive on this channel */

/* Sensor report IDs from SH-2 firmware */
#define BNO086_REPORT_GAME_ROTATION_VECTOR  0x05
#define BNO086_REPORT_ROTATION_VECTOR       0x08
#define BNO086_REPORT_ACCELEROMETER         0x02
#define BNO086_REPORT_GYROSCOPE             0x01

/* Command IDs */
#define BNO086_CMD_PRODUCT_ID_REQUEST   0xF9
#define BNO086_CMD_PRODUCT_ID_RESPONSE  0xF8
#define BNO086_CMD_SET_FEATURE          0xFD
#define BNO086_CMD_GET_FEATURE          0xFE
#define BNO086_CMD_FEATURE_RESPONSE     0xFC

/* Product IDs (SH-2 firmware) */
#define BNO086_PRODUCT_ID      0x0006
#define BNO085_PRODUCT_ID      0x0005

/* Check if a product ID belongs to a supported BNO08x series chip */
#define BNO08X_PID_IS_VALID(hi, lo) \
	(((hi) == 0x00 && ((lo) == 0x06 || (lo) == 0x05)))

/* Fixed-size internal packet record (5 floats = 20 bytes) */
#define BNO086_PACKET_SIZE  20

/* --------------------------------------------------------------------------
 * sensor_imu_t interface functions
 * -------------------------------------------------------------------------- */

int  bno086_init(float clock_rate, float accel_time, float gyro_time,
		 float *accel_actual_time, float *gyro_actual_time);
void bno086_shutdown(void);

void bno086_update_fs(float accel_range, float gyro_range,
		      float *accel_actual_range, float *gyro_actual_range);
int  bno086_update_odr(float accel_time, float gyro_time,
		       float *accel_actual_time, float *gyro_actual_time);

uint16_t bno086_fifo_read(uint8_t *data, uint16_t len);
int  bno086_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void bno086_accel_read(float a[3]);
void bno086_gyro_read(float g[3]);
float bno086_temp_read(void);

uint8_t bno086_setup_DRDY(uint16_t threshold);
uint8_t bno086_setup_WOM(void);

/* Direct quaternion access (BNO086 internal fusion, bypasses VQF) */
void bno086_get_quaternion(float q[4]);
int  bno086_read_product_id(uint8_t *product_id_low, uint8_t *product_id_high);

/* Driver instance */
extern const sensor_imu_t sensor_imu_bno086;

/*
 * SHTP-based scan probe.
 *
 * Tries to detect a BNO086 at the given I2C device address using the
 * SHTP Product ID Request.  This is a dedicated probe because BNO086 has
 * no standard WHO_AM_I register.
 *
 *   i2c_dev   – pointer to the device spec (address will be probed)
 *   reg       – [out] set to 0x80 for SPI marker or 0x00 for I2C if found
 *   interface_register – if true, registers the IMU with the sensor
 *                        interface system on success
 *
 * Returns the enum dev_imu value (IMU_BNO086) on success, or -1 if not
 * a BNO086.
 */
int bno086_scan_probe(struct i2c_dt_spec *i2c_dev, uint8_t *reg,
		      bool interface_register);

#endif /* BNO086_H */
