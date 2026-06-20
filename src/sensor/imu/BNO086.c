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

/*
 * BNO086 IMU driver for SlimeVR-Tracker-nRF.
 *
 * The BNO086 (Bosch Sensortec) is a 9-axis IMU with an integrated Cortex-M0
 * running SH-2 firmware.  Unlike traditional register-mapped IMUs it
 * communicates via the SHTP (Sensor Hub Transport Protocol) and outputs
 * pre-fused sensor data.
 *
 * Architecture
 * ------------
 *   I2C  (0x4A / 0x4B)  <-- no register map, pure streaming SHTP packets
 *   SHTP transport layer  -- packet framing, CRC-8, header parsing
 *   SH-2 application layer -- product-id request, sensor report configuration
 *   sensor_imu_t bridge   -- adapts to the existing VQF fusion pipeline
 *
 * The driver enables the Game Rotation Vector (report 0x05, 6-DoF fused
 * quaternion) and converts it into approximate gyroscope / accelerometer
 * readings so the existing VQF fusion can continue to operate.
 */

#include "BNO086.h"
#include "sensor/sensor_none.h"
#include "sensor/sensors_enum.h"
#include "../../util.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include <hal/nrf_gpio.h>

LOG_MODULE_REGISTER(BNO086, LOG_LEVEL_INF);

/* =========================================================================
 *  SHTP Transport Layer
 * ========================================================================= */

/*
 * CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07), initial value 0xFF.
 * Computed over the SHTP header + payload bytes.
 */
static uint8_t shtp_crc8(const uint8_t *data, uint32_t len)
{
	uint8_t crc = 0xFF;
	for (uint32_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++) {
			if (crc & 0x80) {
				crc = (uint8_t)((crc << 1) ^ 0x07);
			} else {
				crc = (uint8_t)(crc << 1);
			}
		}
	}
	return crc;
}

/*
 * Build a full SHTP packet in buf (must be at least 4 + payload_len + 1).
 * Returns total bytes written.
 */
static uint32_t shtp_build_packet(uint8_t *buf, uint8_t channel,
				   uint8_t seq, const uint8_t *payload,
				   uint32_t payload_len)
{
	buf[0] = (uint8_t)(payload_len & 0xFF);
	buf[1] = (uint8_t)((payload_len >> 8) & 0x3F);
	buf[2] = channel;
	buf[3] = seq;

	memcpy(buf + BNO086_SHTP_HEADER_SIZE, payload, payload_len);

	uint32_t hdr_payload_len = BNO086_SHTP_HEADER_SIZE + payload_len;
	buf[hdr_payload_len] = shtp_crc8(buf, hdr_payload_len);

	return hdr_payload_len + 1;
}

/*
 * Send an SHTP packet over I2C.
 * Returns 0 on success, negative on error.
 */
static int shtp_send(uint8_t channel, const uint8_t *payload, uint32_t payload_len)
{
	static uint8_t tx_seq; /* wraps naturally */

	uint8_t pkt[BNO086_SHTP_MAX_PACKET];
	uint32_t total = shtp_build_packet(pkt, channel, tx_seq++, payload,
					    payload_len);

	return ssi_write(SENSOR_INTERFACE_DEV_IMU, pkt, total);
}

/*
 * Try to read one SHTP packet from the I2C bus.
 *
 *   buf       – output buffer, must hold at least BNO086_SHTP_MAX_PACKET bytes
 *   payload   – on success points to the payload inside buf
 *   payload_len – on success is the length of the payload
 *   channel   – on success is the SHTP channel number
 *
 * Returns 0 on success, -1 if no data available, -2 on CRC error.
 */
static int shtp_recv(uint8_t *buf, uint8_t **payload, uint32_t *payload_len,
		     uint8_t *channel)
{
	/* Read header (4 bytes) */
	int err = ssi_read(SENSOR_INTERFACE_DEV_IMU, buf, BNO086_SHTP_HEADER_SIZE);
	if (err < 0) {
		return -1; /* no data / bus error */
	}

	uint32_t pkt_payload_len = (uint32_t)buf[0]
				 | ((uint32_t)(buf[1] & 0x3F) << 8);

	if (pkt_payload_len > BNO086_SHTP_MAX_PAYLOAD) {
		LOG_WRN("SHTP payload too large: %u", pkt_payload_len);
		return -1;
	}

	uint32_t remaining = pkt_payload_len + BNO086_SHTP_CRC_SIZE;
	if (remaining == 0) {
		return -1;
	}

	err = ssi_read(SENSOR_INTERFACE_DEV_IMU, buf + BNO086_SHTP_HEADER_SIZE,
		       remaining);
	if (err < 0) {
		return -1;
	}

	/* Verify CRC */
	uint32_t hdr_payload_len = BNO086_SHTP_HEADER_SIZE + pkt_payload_len;
	uint8_t expected_crc = shtp_crc8(buf, hdr_payload_len);
	uint8_t received_crc = buf[hdr_payload_len];

	if (expected_crc != received_crc) {
		LOG_WRN("SHTP CRC mismatch: calc=0x%02X recv=0x%02X",
			expected_crc, received_crc);
		return -2;
	}

	*payload     = buf + BNO086_SHTP_HEADER_SIZE;
	*payload_len = pkt_payload_len;
	*channel     = buf[2];

	return 0;
}

/*
 * Poll for incoming packets until one with the expected channel is received.
 * Used during initialisation when the BNO086 is still starting up.
 */
static int shtp_wait_for_channel(uint8_t *buf, uint8_t **payload,
				  uint32_t *payload_len, uint8_t expected_ch,
				  int timeout_ms)
{
	int64_t start = k_uptime_get();

	while ((k_uptime_get() - start) < timeout_ms) {
		uint8_t ch;
		int err = shtp_recv(buf, payload, payload_len, &ch);
		if (err == 0 && ch == expected_ch) {
			return 0;
		}
		k_msleep(5);
	}
	LOG_WRN("Timeout waiting for SHTP channel %u", expected_ch);
	return -1;
}

/* =========================================================================
 *  SH-2 Application Layer
 * ========================================================================= */

/*
 * Read BNO086 product ID to verify we are talking to the right chip.
 * Returns 0 on success.
 */
int bno086_read_product_id(uint8_t *product_id_low, uint8_t *product_id_high)
{
	uint8_t pkt_buf[BNO086_SHTP_MAX_PACKET];

	uint8_t cmd[] = {BNO086_CMD_PRODUCT_ID_REQUEST, 0x00};
	int err = shtp_send(BNO086_SHTP_CH_COMMAND, cmd, sizeof(cmd));
	if (err < 0) {
		return -1;
	}

	uint8_t *payload;
	uint32_t payload_len;
	uint8_t channel;

	/* Product ID response arrives on channel 0 */
	err = shtp_wait_for_channel(pkt_buf, &payload, &payload_len,
				    BNO086_SHTP_CH_COMMAND, 200);
	if (err < 0) {
		return -1;
	}

	if (payload_len < 4 || payload[0] != BNO086_CMD_PRODUCT_ID_RESPONSE) {
		LOG_WRN("Unexpected product ID response: len=%u id=0x%02X",
			payload_len, payload_len > 0 ? payload[0] : 0);
		return -1;
	}

	*product_id_low  = payload[1];
	*product_id_high = payload[2];

	LOG_INF("Product ID: 0x%02X%02X", *product_id_high, *product_id_low);

	/*
	 * BNO085 (0x0005) and BNO086 (0x0006) share the same SHTP/SH-2
	 * protocol and sensor reports — both are supported.
	 */
	switch (((uint16_t)*product_id_high << 8) | *product_id_low) {
	case BNO086_PRODUCT_ID:
		LOG_INF("Detected BNO086");
		break;
	case BNO085_PRODUCT_ID:
		LOG_INF("Detected BNO085 (compatible mode)");
		break;
	default:
		LOG_WRN("Unexpected product ID (expected 0x0005 or 0x0006)");
		return -2;
	}

	return 0;
}

/*
 * Configure a sensor report via the FRS (Feature Report Set) on channel 2.
 *
 *   report_id       – sensor report to enable (e.g. 0x05 for GRV)
 *   interval_us     – report interval in microseconds (0 = disable)
 */
static int bno086_set_report(uint8_t report_id, uint32_t interval_us)
{
	uint8_t cmd[17] = {
		BNO086_CMD_SET_FEATURE,   /* [ 0] set-feature command */
		report_id,                 /* [ 1] feature report ID    */
		0x00,                      /* [ 2] flags                */
		0x00, 0x00,                /* [ 3-4] change sensitivity (uint16 LE) */
		0x00, 0x00, 0x00, 0x00,    /* [ 5-8] report interval   (uint32 LE) */
		0x00, 0x00, 0x00, 0x00,    /* [9-12] batch timeout     (uint32 LE) */
		0x00, 0x00, 0x00, 0x00,    /*[13-16] sensor-specific config (uint32 LE) */
	};

	/* Pack report interval as little-endian uint32 */
	cmd[5] = (uint8_t)(interval_us & 0xFF);
	cmd[6] = (uint8_t)((interval_us >> 8) & 0xFF);
	cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
	cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);

	int err = shtp_send(BNO086_SHTP_CH_CONTROL, cmd, sizeof(cmd));
	if (err < 0) {
		LOG_ERR("Failed to send set-feature for report 0x%02X", report_id);
	}

	return err;
}

/* =========================================================================
 *  Sensor data decoding
 * ========================================================================= */

/*
 * Decode a Game Rotation Vector report (report ID 0x05).
 *   payload[0]   = 0x05 (report ID)
 *   payload[1]   = sequence number
 *   payload[2]   = status (3 = high accuracy)
 *   payload[3]   = delay (µs)
 *   payload[4-7]  = i  (Q30 fixed point, int32 LE)
 *   payload[8-11] = j
 *   payload[12-15]= k
 *   payload[16-19]= real
 *
 * Output q[4] = [w, x, y, z] in standard quaternion order.
 */

static inline float q30_to_float(const uint8_t *p)
{
	int32_t raw = (int32_t)(((uint32_t)p[0])
		  | ((uint32_t)p[1] << 8)
		  | ((uint32_t)p[2] << 16)
		  | ((uint32_t)p[3] << 24));
	return (float)raw * (1.0f / (float)(1 << 30));
}

static void decode_grv(const uint8_t *payload, float q[4])
{
	q[1] = q30_to_float(payload + 4);   /* x = i  */
	q[2] = q30_to_float(payload + 8);   /* y = j  */
	q[3] = q30_to_float(payload + 12);  /* z = k  */
	q[0] = q30_to_float(payload + 16);  /* w = real */
}

/* =========================================================================
 *  Driver state
 * ========================================================================= */

static struct {
	/* Output data rate tracking */
	float accel_time;       /* nominal accel sample period (s) */
	float gyro_time;        /* nominal gyro  sample period (s) */
	float actual_time;      /* actual GRV sample period (s)    */

	/* Cached last sample for gyro differentiation */
	float last_q[4];
	float last_q_norm;
	int64_t last_ticks;
	bool last_q_valid;

	/* Cached output for direct-read callbacks */
	float cached_accel[3];
	float cached_gyro[3];
	float cached_temp;      /* °C */
	bool sample_ready;      /* a full sample was decoded this cycle */

	/* Initialised flag */
	bool inited;
} bno;

/* =========================================================================
 *  sensor_imu_t callbacks
 * ========================================================================= */

static float bno086_requested_accel_time;
static float bno086_requested_gyro_time;

int bno086_init(float clock_rate, float accel_time, float gyro_time,
		float *accel_actual_time, float *gyro_actual_time)
{
	bno086_requested_accel_time = accel_time;
	bno086_requested_gyro_time  = gyro_time;

	/* BNO086 doesn't use an external clock input, ignore clock_rate. */
	(void)clock_rate;

	LOG_INF("BNO086 initializing at %.1f Hz gyro / %.1f Hz accel rate",
		1.0 / (double)gyro_time, 1.0 / (double)accel_time);

	/*
	 * Wait for BNO086 to finish booting.
	 * After reset/power-up the chip takes ~300-400 ms before it starts
	 * sending SHTP advertisement packets on channel 0.
	 */
	uint8_t pkt_buf[BNO086_SHTP_MAX_PACKET];
	uint8_t *payload;
	uint32_t payload_len;
	uint8_t channel;
	int err;

	/* Drain any stale bytes, then poll for advertisement */
	{
		uint8_t dummy[4];
		ssi_read(SENSOR_INTERFACE_DEV_IMU, dummy, sizeof(dummy));
	}
	k_msleep(50);

	err = shtp_wait_for_channel(pkt_buf, &payload, &payload_len,
				    BNO086_SHTP_CH_COMMAND, 500);
	if (err < 0) {
		LOG_ERR("BNO086 did not respond (no advertisement)");
		return -1;
	}
	LOG_INF("BNO086 boot advertisement received");

	/* Verify product ID */
	uint8_t pid_low, pid_high;
	err = bno086_read_product_id(&pid_low, &pid_high);
	if (err < 0) {
		return -1;
	}

	/*
	 * Compute GRV report interval from the requested gyro rate.
	 * Use the faster of accel/gyro as the nominal ODR.
	 */
	float desired_odr = 1.0f / MIN(accel_time, gyro_time);
	if (desired_odr < 1.0f) {
		desired_odr = 1.0f;
	}
	if (desired_odr > 400.0f) {
		desired_odr = 400.0f; /* BNO086 GRV max ~400 Hz */
	}

	uint32_t interval_us = (uint32_t)(1e6f / desired_odr);
	if (interval_us < 2500) {
		interval_us = 2500; /* SH-2 minimum report interval */
	}

	/* Enable Game Rotation Vector reports */
	err = bno086_set_report(BNO086_REPORT_GAME_ROTATION_VECTOR, interval_us);
	if (err < 0) {
		LOG_ERR("Failed to configure GRV report");
		return -1;
	}
	LOG_INF("GRV configured at %u us interval (~%.1f Hz)",
		interval_us, 1e6f / (float)interval_us);

	/* Wait for first sensor report to arrive */
	err = shtp_wait_for_channel(pkt_buf, &payload, &payload_len,
				    BNO086_SHTP_CH_INPUT, 300);
	if (err < 0) {
		LOG_WRN("No sensor report received after configuration");
		/* Continue anyway – reports may start later */
	}

	/* Compute actual ODR */
	float actual_odr = 1e6f / (float)interval_us;
	bno.actual_time = 1.0f / actual_odr;
	bno.accel_time  = bno.actual_time;
	bno.gyro_time   = bno.actual_time;

	*accel_actual_time = bno.accel_time;
	*gyro_actual_time  = bno.gyro_time;

	bno.last_q_valid = false;
	bno.last_ticks   = 0;
	bno.sample_ready = false;
	bno.cached_temp  = 25.0f;
	bno.inited       = true;

	LOG_INF("BNO086 initialized: ODR=%.1f Hz", actual_odr);
	return 0;
}

void bno086_shutdown(void)
{
	if (!bno.inited) {
		return;
	}
	/* Disable GRV reports */
	bno086_set_report(BNO086_REPORT_GAME_ROTATION_VECTOR, 0);
	bno.inited = false;
	LOG_INF("BNO086 shutdown");
}

void bno086_update_fs(float accel_range, float gyro_range,
		      float *accel_actual_range, float *gyro_actual_range)
{
	/*
	 * BNO086 has no configurable full-scale range in standard
	 * Game Rotation Vector mode.  Report constants.
	 */
	*accel_actual_range = 16.0f;  /* typical BNO086 accel range */
	*gyro_actual_range  = 2000.0f;/* typical BNO086 gyro range  */

	(void)accel_range;
	(void)gyro_range;
}

int bno086_update_odr(float accel_time, float gyro_time,
		      float *accel_actual_time, float *gyro_actual_time)
{
	float new_odr = 1.0f / MIN(accel_time, gyro_time);
	if (new_odr < 1.0f) {
		new_odr = 1.0f;
	}
	if (new_odr > 400.0f) {
		new_odr = 400.0f;
	}

	uint32_t interval_us = (uint32_t)(1e6f / new_odr);
	if (interval_us < 2500) {
		interval_us = 2500;
	}

	int err = bno086_set_report(BNO086_REPORT_GAME_ROTATION_VECTOR,
				    interval_us);
	if (err < 0) {
		return -1;
	}

	bno.actual_time = (float)interval_us * 1e-6f;
	bno.accel_time  = bno.actual_time;
	bno.gyro_time   = bno.actual_time;

	*accel_actual_time = bno.accel_time;
	*gyro_actual_time  = bno.gyro_time;

	LOG_INF("BNO086 ODR updated to %.1f Hz",
		1.0 / (double)bno.actual_time);
	return 0;
}

/*
 * Pack a decoded quaternion sample into the raw-data buffer as a
 * fixed-size record of 5 floats: [w, x, y, z, dt_ms].
 */
static void pack_sample(uint8_t *dst, const float q[4], float dt_ms)
{
	memcpy(dst,      &q[0],  sizeof(float));
	memcpy(dst + 4,  &q[1],  sizeof(float));
	memcpy(dst + 8,  &q[2],  sizeof(float));
	memcpy(dst + 12, &q[3],  sizeof(float));
	memcpy(dst + 16, &dt_ms, sizeof(float));
}

/*
 * fifo_read – Read available SHTP packets, decode Game Rotation Vector
 * reports, pack them into the raw-data buffer as fixed-size records.
 *
 * Returns the number of samples packed (0..max_samples).
 */
uint16_t bno086_fifo_read(uint8_t *rawData, uint16_t len)
{
	if (!bno.inited) {
		return 0;
	}

	uint16_t max_samples = len / BNO086_PACKET_SIZE;
	uint16_t samples     = 0;
	int64_t now_ticks    = k_uptime_ticks();

	bno.sample_ready = false;

	while (samples < max_samples) {
		uint8_t pkt_buf[BNO086_SHTP_MAX_PACKET];
		uint8_t *payload;
		uint32_t payload_len;
		uint8_t channel;

		int err = shtp_recv(pkt_buf, &payload, &payload_len, &channel);
		if (err < 0) {
			break; /* no more data or error */
		}

		/* Only process sensor reports on channel 3 */
		if (channel != BNO086_SHTP_CH_INPUT || payload_len < 5) {
			continue;
		}

		uint8_t report_id = payload[0];
		// uint8_t status = payload[2]; /* accuracy, unused for now */

		if (report_id != BNO086_REPORT_GAME_ROTATION_VECTOR) {
			/* Skip non-GRV reports */
			continue;
		}

		/* Decode quaternion */
		float q[4];
		decode_grv(payload, q);

		/* Compute dt from previous tick */
		float dt_ms = 0;
		if (bno.last_ticks != 0) {
			dt_ms = (float)k_ticks_to_ms_near64(
				now_ticks - bno.last_ticks);
		}

		/* Clamp dt to avoid huge jumps after gaps */
		if (dt_ms > 500.0f || dt_ms < 0.0f) {
			dt_ms = bno.actual_time * 1e3f;
		}

		bno.last_ticks = now_ticks;

		/* Pack into buffer */
		pack_sample(rawData + samples * BNO086_PACKET_SIZE, q, dt_ms);
		samples++;

		/* Cache the most recent sample for direct-read callbacks */
		memcpy(bno.last_q, q, sizeof(bno.last_q));
		bno.last_q_norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
		if (bno.last_q_norm > 0.0f) {
			bno.last_q_norm = 1.0f / bno.last_q_norm;
		}
		bno.last_q_valid = true;
		bno.sample_ready = true;
	}

	return samples;
}

/*
 * fifo_process – Given a sample index into the raw-data buffer, unpack the
 * stored quaternion, compute approximate gyroscope (from quaternion
 * differentiation) and approximate accelerometer (from gravity direction),
 * and write them to a[3] and g[3].
 *
 * Returns 0 on success, > 0 to skip.
 */
int bno086_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	uint8_t *rec = data + index * BNO086_PACKET_SIZE;

	float q_curr[4], dt_ms;
	memcpy(&q_curr[0], rec,       sizeof(float));
	memcpy(&q_curr[1], rec + 4,  sizeof(float));
	memcpy(&q_curr[2], rec + 8,  sizeof(float));
	memcpy(&q_curr[3], rec + 12, sizeof(float));
	memcpy(&dt_ms,      rec + 16, sizeof(float));

	if (dt_ms <= 0.0f || dt_ms > 1000.0f) {
		dt_ms = bno.actual_time * 1e3f;
	}

	/* Normalize current quaternion */
	float n = sqrtf(q_curr[0]*q_curr[0] + q_curr[1]*q_curr[1]
			+ q_curr[2]*q_curr[2] + q_curr[3]*q_curr[3]);
	if (n < 1e-6f) {
		return 1; /* degenerate quaternion, skip */
	}
	float inv_n = 1.0f / n;
	q_curr[0] *= inv_n;
	q_curr[1] *= inv_n;
	q_curr[2] *= inv_n;
	q_curr[3] *= inv_n;

	/*
	 * Compute accelerometer from gravity vector.
	 * In device frame: gravity = [0, 0, 1] (up in world frame).
	 * Rotate by q_curr to get body-frame gravity.
	 */
	float qw = q_curr[0], qx = q_curr[1], qy = q_curr[2], qz = q_curr[3];
	a[0] = 2.0f * (qx * qz - qw * qy);
	a[1] = 2.0f * (qy * qz + qw * qx);
	a[2] = qw * qw - qx * qx - qy * qy + qz * qz;

	/*
	 * Compute angular velocity from quaternion difference.
	 * q_diff = conj(q_prev) * q_curr
	 * gyro   = 2 * log(q_diff) / dt  ≅  2 * vec(q_diff) / dt  (small angle)
	 *
	 * We use the last cached quaternion from the driver state
	 * rather than maintaining per-sample history in the buffer.
	 */
	memset(g, 0, sizeof(float) * 3);

	if (bno.last_q_valid) {
		/* q_diff = conj(last_q) * q_curr */
		float qp[4];
		memcpy(qp, bno.last_q, sizeof(qp));

		float dqw = qp[0]*q_curr[0] + qp[1]*q_curr[1]
			  + qp[2]*q_curr[2] + qp[3]*q_curr[3];
		float dqx = qp[0]*q_curr[1] - qp[1]*q_curr[0]
			  - qp[2]*q_curr[3] + qp[3]*q_curr[2];
		float dqy = qp[0]*q_curr[2] + qp[1]*q_curr[3]
			  - qp[2]*q_curr[0] - qp[3]*q_curr[1];
		float dqz = qp[0]*q_curr[3] - qp[1]*q_curr[2]
			  + qp[2]*q_curr[1] - qp[3]*q_curr[0];

		float dt_s = dt_ms * 1e-3f;
		/* For small angles: angle ≈ 2 * arcsin(|vec|) ≈ 2 * |vec| */
		float scale = 2.0f / dt_s;

		/* Handle quaternion sign ambiguity: ensure shortest path */
		if (dqw < 0.0f) {
			scale = -scale;
		}

		g[0] = dqx * scale * (180.0f / (float)M_PI); /* deg/s */
		g[1] = dqy * scale * (180.0f / (float)M_PI);
		g[2] = dqz * scale * (180.0f / (float)M_PI);
	}

	/* Update last_q for next call */
	memcpy(bno.last_q, q_curr, sizeof(bno.last_q));
	bno.last_q_valid = true;

	/* Cache for direct-read callbacks */
	memcpy(bno.cached_accel, a, sizeof(bno.cached_accel));
	memcpy(bno.cached_gyro,  g, sizeof(bno.cached_gyro));

	return 0;
}

void bno086_accel_read(float a[3])
{
	memcpy(a, bno.cached_accel, sizeof(bno.cached_accel));
}

void bno086_gyro_read(float g[3])
{
	memcpy(g, bno.cached_gyro, sizeof(bno.cached_gyro));
}

float bno086_temp_read(void)
{
	/*
	 * BNO086 Game Rotation Vector reports don't include temperature.
	 * To get temperature we would need to enable the Temperature report
	 * (0x07) which interleaves with GRV on channel 3.
	 *
	 * For now, return a constant room-temperature value so the
	 * calibration pipeline doesn't break.
	 *
	 * TODO: enable Temperature report and decode in fifo_read /
	 *       share via a cached value here.
	 */
	return bno.cached_temp;
}

uint8_t bno086_setup_DRDY(uint16_t threshold)
{
	/*
	 * BNO086 can be configured to assert INT on new sensor data.
	 * The INT pin polarity and behaviour is controlled via FRS.
	 * For now, return a valid pin config for a basic active-low setup.
	 *
	 * TODO: implement proper INT configuration via FRS.
	 */
	(void)threshold;

	/* Default: active-low, push-pull */
	return (uint8_t)((NRF_GPIO_PIN_PULLUP << 4) | NRF_GPIO_PIN_SENSE_LOW);
}

uint8_t bno086_setup_WOM(void)
{
	/*
	 * BNO086 supports wake-on-motion via the accelerometer's
	 * any-motion / no-motion detector configured through FRS.
	 *
	 * TODO: implement WOM via FRS configuration.
	 */
	LOG_WRN("BNO086 Wake-on-Motion not yet implemented");
	return 0;
}

void bno086_get_quaternion(float q[4])
{
	if (bno.last_q_valid) {
		memcpy(q, bno.last_q, sizeof(float) * 4);
	} else {
		q[0] = 1.0f;
		q[1] = 0.0f;
		q[2] = 0.0f;
		q[3] = 0.0f;
	}
}

/*
 * SHTP-based scan probe using raw I2C (no ssi interface registration needed).
 *
 * This is a dedicated detection routine because BNO086 has no WHO_AM_I
 * register — identification requires a full SHTP Product ID exchange.
 * Call this from sensor.c's scan function when the standard I2C scan
 * finds nothing at addresses 0x4A / 0x4B.
 */
int bno086_scan_probe(struct i2c_dt_spec *i2c_dev, uint8_t *reg,
		      bool interface_register)
{
	static const uint8_t probe_addrs[] = {
		BNO086_I2C_ADDR_DEFAULT,
		BNO086_I2C_ADDR_ALT
	};

	uint16_t saved_addr = i2c_dev->addr;
	const struct device *bus = i2c_dev->bus;
	struct i2c_dt_spec tmp_dev;
	int err;

	for (int ai = 0; ai < (int)ARRAY_SIZE(probe_addrs); ai++) {
		uint8_t addr = probe_addrs[ai];

		/*
		 * If the caller already configured a specific address and
		 * it doesn't match this probe address, skip it.
		 */
		if (saved_addr >= 8 && saved_addr <= 119 &&
		    saved_addr != addr) {
			continue;
		}

		LOG_INF("BNO086 SHTP probe at 0x%02X", addr);

		/* Build a temporary device spec */
		tmp_dev.bus  = bus;
		tmp_dev.addr = addr;

		/*
		 * BNO08x takes ~350ms after power-on before it starts sending
		 * SHTP advertisements.  The sensor scan already waited 50ms,
		 * so wait an additional 350ms here (400ms total margin).
		 */
		k_msleep(350);

		/*
		 * Poll for SHTP advertisement.  After boot, BNO086 sends a
		 * packet on channel 0 whose first payload byte is 0x00.
		 * A previously-configured chip may already be streaming GRV
		 * (channel 3, report 0x05) — accept that too.
		 *
		 * Wait up to 1500ms for the chip to respond.
		 */
		int64_t deadline = k_uptime_get() + 1500;
		bool got_advert = false;
		int i2c_err_count = 0;

		while (k_uptime_get() < deadline) {
			uint8_t hdr[4];
			int err = i2c_read_dt(&tmp_dev, hdr, 4);
			if (err < 0) {
				if (i2c_err_count == 0) {
					LOG_INF("BNO086 I2C read error at 0x%02X: %d (bus may have no pull-ups or chip NACKs during boot)", addr, err);
				}
				i2c_err_count++;
				k_msleep(20);
				continue;
			}
			i2c_err_count = 0; /* reset on success */

			uint32_t pld_len = (uint32_t)hdr[0]
				| ((uint32_t)(hdr[1] & 0x3F) << 8);
			if (pld_len == 0 || pld_len > BNO086_SHTP_MAX_PAYLOAD) {
				k_msleep(10);
				continue;
			}

			uint8_t pkt_buf[BNO086_SHTP_MAX_PACKET];
			memcpy(pkt_buf, hdr, 4);
			err = i2c_read_dt(&tmp_dev, pkt_buf + 4,
					  pld_len + 1);
			if (err < 0) {
				k_msleep(10);
				continue;
			}

			/* Verify CRC */
			uint32_t check_len = 4 + pld_len;
			uint8_t calc_crc = shtp_crc8(pkt_buf, check_len);
			if (calc_crc != pkt_buf[check_len]) {
				LOG_DBG("CRC mismatch at 0x%02X", addr);
				k_msleep(10);
				continue;
			}

			/* Channel 0 with first payload byte 0x00 = advertisement */
			if (hdr[2] == 0 && pld_len >= 1 && pkt_buf[4] == 0x00) {
				got_advert = true;
				break;
			}

			/* Also accept a GRV sensor report (channel 3, report 0x05)
			 * — this means the chip is already initialized from a
			 * previous session and is streaming data. */
			if (hdr[2] == 3 && pld_len >= 5 &&
			    pkt_buf[4] == BNO086_REPORT_GAME_ROTATION_VECTOR) {
				got_advert = true;
				break;
			}
		}

		if (!got_advert) {
			LOG_INF("BNO086: no SHTP response at 0x%02X", addr);
			continue;
		}

		LOG_INF("BNO086: SHTP advertisement received at 0x%02X", addr);

		/*
		 * SHTP advertisement received.  Now send a Product ID
		 * Request to confirm this is a BNO086 (not a BNO080/BNO085).
		 */
		uint8_t prod_req_payload[] = {BNO086_CMD_PRODUCT_ID_REQUEST, 0x00};
		uint8_t tx_pkt[BNO086_SHTP_MAX_PACKET];
		uint32_t tx_len = shtp_build_packet(tx_pkt, BNO086_SHTP_CH_COMMAND,
						     0, prod_req_payload,
						     sizeof(prod_req_payload));
		err = i2c_write_dt(&tmp_dev, tx_pkt, tx_len);
		if (err < 0) {
			continue;
		}
		k_msleep(20);

		/* Read Product ID Response */
		deadline = k_uptime_get() + 200;
		bool ok = false;
		int detected_imu = -1;

		while (k_uptime_get() < deadline) {
			uint8_t hdr[4];
			err = i2c_read_dt(&tmp_dev, hdr, 4);
			if (err < 0) {
				k_msleep(5);
				continue;
			}

			uint32_t pld_len = (uint32_t)hdr[0]
				| ((uint32_t)(hdr[1] & 0x3F) << 8);
			if (pld_len < 4 || pld_len > BNO086_SHTP_MAX_PAYLOAD) {
				k_msleep(5);
				continue;
			}

			uint8_t pkt_buf[BNO086_SHTP_MAX_PACKET];
			memcpy(pkt_buf, hdr, 4);
			err = i2c_read_dt(&tmp_dev, pkt_buf + 4,
					  pld_len + 1);
			if (err < 0) {
				k_msleep(5);
				continue;
			}

			uint32_t check_len = 4 + pld_len;
			if (shtp_crc8(pkt_buf, check_len) != pkt_buf[check_len]) {
				continue;
			}

			if (hdr[2] == 0 && pld_len >= 4 &&
			    pkt_buf[4] == BNO086_CMD_PRODUCT_ID_RESPONSE) {
				uint8_t pid_low  = pkt_buf[5];
				uint8_t pid_high = pkt_buf[6];
				uint16_t pid = ((uint16_t)pid_high << 8) | pid_low;

				LOG_INF("BNO08x found at 0x%02X (pid=0x%02X%02X)",
					addr, pid_high, pid_low);

				switch (pid) {
				case BNO086_PRODUCT_ID:
					detected_imu = IMU_BNO086;
					break;
				case BNO085_PRODUCT_ID:
					detected_imu = IMU_BNO085;
					break;
				default:
					LOG_WRN("Unsupported product ID (expected 0x0005 or 0x0006)");
					break;
				}
				if (detected_imu >= 0) {
					ok = true;
				}
				break;
			}
		}

		if (!ok) {
			continue;
		}

		/* Found and verified — set up the device spec */
		i2c_dev->addr = addr;
		*reg = 0x00; /* I2C (0x80 would mean SPI) */

		if (interface_register) {
			sensor_interface_register_sensor_imu_i2c(i2c_dev);
		}

		return detected_imu;
	}

	/* Restore original address on failure */
	i2c_dev->addr = saved_addr;
	return -1;
}

/* =========================================================================
 *  sensor_imu_t instance (registered in sensors.h)
 * ========================================================================= */

const sensor_imu_t sensor_imu_bno086 = {
	.init          = bno086_init,
	.shutdown      = bno086_shutdown,
	.update_fs     = bno086_update_fs,
	.update_odr    = bno086_update_odr,
	.fifo_read     = bno086_fifo_read,
	.fifo_process  = bno086_fifo_process,
	.accel_read    = bno086_accel_read,
	.gyro_read     = bno086_gyro_read,
	.temp_read     = bno086_temp_read,
	.setup_DRDY    = bno086_setup_DRDY,
	.setup_WOM     = bno086_setup_WOM,
	.ext_setup     = imu_none_ext_setup,      /* no external mag support */
	.ext_passthrough = imu_none_ext_passthrough,
};
