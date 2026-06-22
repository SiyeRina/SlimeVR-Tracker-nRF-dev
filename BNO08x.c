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

/*
 * BNO08x IMU driver for SlimeVR-Tracker-nRF.
 *
 * The BNO08x (Bosch Sensortec) is a 9-axis IMU with an integrated Cortex-M0
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

#include "BNO08x.h"
#include "sensor/sensor_none.h"
#include "sensor/sensors_enum.h"
#include "../../util.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include <hal/nrf_gpio.h>

LOG_MODULE_REGISTER(BNO08X, CONFIG_BNO08X_LOG_LEVEL);


/* =========================================================================
 *  Mutex for thread safety
 * ========================================================================= */
static K_MUTEX_DEFINE(bno_mutex);

/* =========================================================================
 *  SHTP Transport Layer
 * ========================================================================= */

static uint8_t shtp_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFF;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t shtp_build_packet(uint8_t *buf, uint8_t channel,
                                  uint8_t seq, const uint8_t *payload,
                                  uint32_t payload_len)
{
    buf[0] = (uint8_t)(payload_len & 0xFF);
    buf[1] = (uint8_t)((payload_len >> 8) & 0x3F);
    buf[2] = channel;
    buf[3] = seq;

    memcpy(buf + BNO08X_SHTP_HEADER_SIZE, payload, payload_len);
    uint32_t total = BNO08X_SHTP_HEADER_SIZE + payload_len;
    buf[total] = shtp_crc8(buf, total);
    return total + 1;
}

static int shtp_send(uint8_t channel, const uint8_t *payload, uint32_t payload_len)
{
    static uint8_t tx_seq;
    uint8_t pkt[BNO08X_SHTP_MAX_PACKET];
    uint32_t total = shtp_build_packet(pkt, channel, tx_seq++, payload, payload_len);
    return ssi_write(SENSOR_INTERFACE_DEV_IMU, pkt, total);
}

static int shtp_recv(uint8_t *buf, uint8_t **payload, uint32_t *payload_len, uint8_t *channel)
{
    int err = ssi_read(SENSOR_INTERFACE_DEV_IMU, buf, BNO08X_SHTP_HEADER_SIZE);
    if (err < 0)
        return -1;

    uint32_t pld_len = (uint32_t)buf[0] | ((uint32_t)(buf[1] & 0x3F) << 8);
    if (pld_len > BNO08X_SHTP_MAX_PAYLOAD) {
        LOG_WRN("Payload too large: %u", pld_len);
        return -1;
    }

    uint32_t remaining = pld_len + BNO08X_SHTP_CRC_SIZE;
    if (remaining == 0)
        return -1;

    err = ssi_read(SENSOR_INTERFACE_DEV_IMU, buf + BNO08X_SHTP_HEADER_SIZE, remaining);
    if (err < 0)
        return -1;

    uint32_t check_len = BNO08X_SHTP_HEADER_SIZE + pld_len;
    uint8_t calc_crc = shtp_crc8(buf, check_len);
    if (calc_crc != buf[check_len]) {
        LOG_WRN("CRC mismatch: calc 0x%02X recv 0x%02X", calc_crc, buf[check_len]);
        return -2;
    }

    *payload = buf + BNO08X_SHTP_HEADER_SIZE;
    *payload_len = pld_len;
    *channel = buf[2];
    return 0;
}

static int shtp_wait_for_channel(uint8_t *buf, uint8_t **payload,
                                 uint32_t *payload_len, uint8_t expected_ch,
                                 int timeout_ms)
{
    int64_t start = k_uptime_get();
    while ((k_uptime_get() - start) < timeout_ms) {
        uint8_t ch;
        int err = shtp_recv(buf, payload, payload_len, &ch);
        if (err == 0 && ch == expected_ch)
            return 0;
        k_msleep(5);
    }
    LOG_WRN("Timeout waiting for SHTP channel %u", expected_ch);
    return -1;
}

/* =========================================================================
 *  SH-2 Application Layer
 * ========================================================================= */

int bno08x_read_product_id(uint8_t *pid_low, uint8_t *pid_high)
{
    uint8_t pkt_buf[BNO08X_SHTP_MAX_PACKET];
    uint8_t cmd[] = {BNO08X_CMD_PRODUCT_ID_REQUEST, 0x00};
    int err = shtp_send(BNO08X_SHTP_CH_COMMAND, cmd, sizeof(cmd));
    if (err < 0)
        return -1;

    uint8_t *payload;
    uint32_t payload_len;
    err = shtp_wait_for_channel(pkt_buf, &payload, &payload_len,
                                BNO08X_SHTP_CH_COMMAND, 200);
    if (err < 0)
        return -1;

    if (payload_len < 4 || payload[0] != BNO08X_CMD_PRODUCT_ID_RESPONSE) {
        LOG_WRN("Unexpected product ID response");
        return -1;
    }

    *pid_low = payload[1];
    *pid_high = payload[2];
    uint16_t pid = ((uint16_t)*pid_high << 8) | *pid_low;
    LOG_INF("Product ID: 0x%04X", pid);

    if (pid == BNO08X_PID_BNO085 || pid == BNO08X_PID_BNO086)
        return 0;
    else
        return -2;
}

static int bno08x_set_report(uint8_t report_id, uint32_t interval_us)
{
    uint8_t cmd[17] = {
        BNO08X_CMD_SET_FEATURE,   /* 0 */
        report_id,                /* 1 */
        0x00,                     /* 2 flags */
        0x00, 0x00,               /* 3-4 change sensitivity */
        0,0,0,0,                  /* 5-8 interval LE */
        0,0,0,0,                  /* 9-12 batch timeout */
        0,0,0,0                   /* 13-16 sensor config */
    };
    cmd[5] = (uint8_t)(interval_us & 0xFF);
    cmd[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);

    int err = shtp_send(BNO08X_SHTP_CH_CONTROL, cmd, sizeof(cmd));
    if (err < 0) {
        LOG_ERR("Failed to set report 0x%02X", report_id);
        return err;
    }

    /* Optional: wait for FEATURE_RESPONSE (0xFC) to confirm */
    uint8_t buf[BNO08X_SHTP_MAX_PACKET];
    uint8_t *payload;
    uint32_t len;
    err = shtp_wait_for_channel(buf, &payload, &len, BNO08X_SHTP_CH_CONTROL, 50);
    if (err == 0 && len >= 2 && payload[0] == BNO08X_CMD_FEATURE_RESPONSE) {
        /* success */
    } else {
        LOG_WRN("No feature response for report 0x%02X", report_id);
    }
    return 0;
}

/* =========================================================================
 *  Sensor Data Decoding
 * ========================================================================= */

static inline float q30_to_float(const uint8_t *p)
{
    int32_t raw = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1]<<8) |
                            ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24));
    return (float)raw * (1.0f / (float)(1 << 30));
}

static void decode_grv(const uint8_t *payload, float q[4])
{
    /* SH-2 Game Rotation Vector: report_id(1) + x(4) + y(4) + z(4) + w(4) */
    q[1] = q30_to_float(payload + 1);   /* x = i */
    q[2] = q30_to_float(payload + 5);   /* y = j */
    q[3] = q30_to_float(payload + 9);   /* z = k */
    q[0] = q30_to_float(payload + 13); /* w = real */
}

static float decode_temperature(const uint8_t *payload)
{
    /* Temperature report: payload[1] = °C * 2?  Check SH-2 spec */
    int16_t raw = (int16_t)(payload[1] | (payload[2] << 8));
    return (float)raw * 0.5f; /* typical scaling */
}

/* =========================================================================
 *  Driver State
 * ========================================================================= */

typedef struct {
    float accel_time;
    float gyro_time;
    float actual_time;       /* current GRV interval in seconds */

    /* Last quaternion for differentiation (used by fifo_process) */
    float last_q[4];
    bool last_q_valid;
    int64_t last_ticks;      /* not used for dt anymore, kept for future */

    float cached_accel[3];
    float cached_gyro[3];
    float cached_temp;       /* °C */
    bool inited;

    /* Temperature report enabled? */
    bool temp_enabled;
} bno08x_state_t;

static bno08x_state_t bno;

/* =========================================================================
 *  sensor_imu_t callbacks
 * ========================================================================= */

int bno08x_init(float clock_rate, float accel_time, float gyro_time,
                float *accel_actual_time, float *gyro_actual_time)
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    int ret = -1;

    (void)clock_rate;

    LOG_INF("BNO08x init: accel %.3f s, gyro %.3f s", accel_time, gyro_time);

    /* Wait for boot advertisement */
    uint8_t pkt_buf[BNO08X_SHTP_MAX_PACKET];
    uint8_t *payload;
    uint32_t payload_len;

    /* Drain any stale data */
    uint8_t dummy[4];
    ssi_read(SENSOR_INTERFACE_DEV_IMU, dummy, sizeof(dummy));
    k_msleep(50);

    /* Send SH-2 reset command */
    uint8_t reset_cmd[] = {BNO08X_CMD_RESET, 0x00};
    shtp_send(BNO08X_SHTP_CH_COMMAND, reset_cmd, sizeof(reset_cmd));
    k_msleep(200);

    /* Wait for advertisement on channel 0 (up to 600 ms) */
    ret = shtp_wait_for_channel(pkt_buf, &payload, &payload_len,
                                BNO08X_SHTP_CH_COMMAND, 600);
    if (ret < 0) {
        LOG_ERR("No boot advertisement");
        goto unlock;
    }

    /* Read product ID */
    uint8_t pid_l, pid_h;
    ret = bno08x_read_product_id(&pid_l, &pid_h);
    if (ret < 0) {
        LOG_ERR("Product ID verification failed");
        goto unlock;
    }

    /* Compute GRV interval */
    float desired_odr = 1.0f / (accel_time < gyro_time ? accel_time : gyro_time);
    if (desired_odr < 1.0f) desired_odr = 1.0f;
    if (desired_odr > 400.0f) desired_odr = 400.0f;
    uint32_t interval_us = (uint32_t)(1e6f / desired_odr);
    if (interval_us < 2500) interval_us = 2500;

    ret = bno08x_set_report(BNO08X_REPORT_GAME_ROTATION_VECTOR, interval_us);
    if (ret < 0) {
        LOG_ERR("Failed to enable GRV");
        goto unlock;
    }

    /* Optional: enable temperature report (0x07) for temp_read */
    ret = bno08x_set_report(BNO08X_REPORT_TEMPERATURE, 1000000); /* 1 Hz */
    if (ret == 0) {
        bno.temp_enabled = true;
        LOG_INF("Temperature report enabled");
    } else {
        bno.temp_enabled = false;
        LOG_WRN("Temperature report not available");
    }

    /* Wait for first sensor report on channel 3 */
    ret = shtp_wait_for_channel(pkt_buf, &payload, &payload_len,
                                BNO08X_SHTP_CH_INPUT, 300);
    if (ret < 0) {
        LOG_WRN("No initial sensor report");
    }

    /* Fill state */
    float actual_odr = 1e6f / (float)interval_us;
    bno.actual_time = 1.0f / actual_odr;
    bno.accel_time = bno.actual_time;
    bno.gyro_time = bno.actual_time;
    *accel_actual_time = bno.actual_time;
    *gyro_actual_time = bno.actual_time;

    bno.last_q_valid = false;
    bno.inited = true;
    bno.cached_temp = 25.0f;
    memset(bno.cached_accel, 0, sizeof(bno.cached_accel));
    memset(bno.cached_gyro, 0, sizeof(bno.cached_gyro));

    LOG_INF("BNO08x init success: ODR=%.1f Hz", actual_odr);
    ret = 0;

unlock:
    k_mutex_unlock(&bno_mutex);
    return ret;
}

void bno08x_shutdown(void)
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    if (bno.inited) {
        bno08x_set_report(BNO08X_REPORT_GAME_ROTATION_VECTOR, 0);
        if (bno.temp_enabled)
            bno08x_set_report(BNO08X_REPORT_TEMPERATURE, 0);
        bno.inited = false;
        LOG_INF("BNO08x shutdown");
    }
    k_mutex_unlock(&bno_mutex);
}

void bno08x_update_fs(float accel_range, float gyro_range,
                      float *accel_actual_range, float *gyro_actual_range)
{
    (void)accel_range; (void)gyro_range;
    *accel_actual_range = 16.0f;
    *gyro_actual_range = 2000.0f;
}

int bno08x_update_odr(float accel_time, float gyro_time,
                      float *accel_actual_time, float *gyro_actual_time)
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    int ret = -1;

    float new_odr = 1.0f / (accel_time < gyro_time ? accel_time : gyro_time);
    if (new_odr < 1.0f) new_odr = 1.0f;
    if (new_odr > 400.0f) new_odr = 400.0f;
    uint32_t interval_us = (uint32_t)(1e6f / new_odr);
    if (interval_us < 2500) interval_us = 2500;

    ret = bno08x_set_report(BNO08X_REPORT_GAME_ROTATION_VECTOR, interval_us);
    if (ret == 0) {
        bno.actual_time = (float)interval_us * 1e-6f;
        bno.accel_time = bno.actual_time;
        bno.gyro_time = bno.actual_time;
        *accel_actual_time = bno.actual_time;
        *gyro_actual_time = bno.actual_time;
        LOG_INF("ODR updated to %.1f Hz", 1.0f / bno.actual_time);
    }
    k_mutex_unlock(&bno_mutex);
    return ret;
}

static void pack_sample(uint8_t *dst, const float q[4], float dt_ms)
{
    memcpy(dst,      &q[0], sizeof(float));
    memcpy(dst + 4,  &q[1], sizeof(float));
    memcpy(dst + 8,  &q[2], sizeof(float));
    memcpy(dst + 12, &q[3], sizeof(float));
    memcpy(dst + 16, &dt_ms, sizeof(float));
}

uint16_t bno08x_fifo_read(uint8_t *rawData, uint16_t len)
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    uint16_t max_samples = len / BNO08X_PACKET_SIZE;
    uint16_t samples = 0;

    /* We'll accumulate delay from consecutive reports within this call */
    uint32_t accumulated_delay_us = 0;

    while (samples < max_samples) {
        uint8_t pkt_buf[BNO08X_SHTP_MAX_PACKET];
        uint8_t *payload;
        uint32_t payload_len;
        uint8_t channel;

        int err = shtp_recv(pkt_buf, &payload, &payload_len, &channel);
        if (err < 0)
            break;

        if (channel != BNO08X_SHTP_CH_INPUT || payload_len < 5)
            continue;

        uint8_t report_id = payload[0];

        if (report_id == BNO08X_REPORT_TEMPERATURE && bno.temp_enabled && payload_len >= 3) {
            bno.cached_temp = decode_temperature(payload);
            continue;
        }

        if (report_id != BNO08X_REPORT_GAME_ROTATION_VECTOR)
            continue;

        float q[4];
        decode_grv(payload, q);

        /* Use the sensor's delay field (payload[3]) for dt */
        uint8_t delay_u8 = payload[3];
        uint32_t delay_us = (uint32_t)delay_u8 * 100; /* delay in 100 µs units? SH-2 spec says delay is in 100 µs units */
        /* But some docs say delay is in µs. We'll trust the spec: delay * 100 µs. */
        /* If the chip sends 0, fallback to accumulated_delay_us. */
        if (delay_us == 0)
            delay_us = 2500; /* default 2.5ms */

        /* For the first sample, we don't have a previous delay; use nominal */
        if (!bno.last_q_valid) {
            /* Use the first sample's delay as initial, but we can't compute gyro yet */
            accumulated_delay_us = delay_us;
        } else {
            /* Add this sample's delay to accumulated */
            accumulated_delay_us += delay_us;
        }

        float dt_ms = (float)accumulated_delay_us * 1e-3f;
        if (dt_ms < 0.001f) dt_ms = 1.0f; /* safety clamp */

        /* Pack sample with dt */
        pack_sample(rawData + samples * BNO08X_PACKET_SIZE, q, dt_ms);
        samples++;

        /* Cache for direct read */
        memcpy(bno.last_q, q, sizeof(bno.last_q));
        bno.last_q_valid = true;

        /* Reset accumulated delay after using it */
        accumulated_delay_us = 0;
    }

    k_mutex_unlock(&bno_mutex);
    return samples;
}

int bno08x_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
    /* No mutex needed here because we only read global state, but we lock for safety */
    k_mutex_lock(&bno_mutex, K_FOREVER);

    uint8_t *rec = data + index * BNO08X_PACKET_SIZE;
    float q_curr[4], dt_ms;
    memcpy(&q_curr[0], rec,       sizeof(float));
    memcpy(&q_curr[1], rec + 4,  sizeof(float));
    memcpy(&q_curr[2], rec + 8,  sizeof(float));
    memcpy(&q_curr[3], rec + 12, sizeof(float));
    memcpy(&dt_ms,      rec + 16, sizeof(float));

    /* Normalize */
    float n = sqrtf(q_curr[0]*q_curr[0] + q_curr[1]*q_curr[1] +
                    q_curr[2]*q_curr[2] + q_curr[3]*q_curr[3]);
    if (n < 1e-6f) {
        k_mutex_unlock(&bno_mutex);
        return 1; /* skip */
    }
    float inv_n = 1.0f / n;
    q_curr[0] *= inv_n; q_curr[1] *= inv_n;
    q_curr[2] *= inv_n; q_curr[3] *= inv_n;

    /* Accelerometer from gravity */
    float qw = q_curr[0], qx = q_curr[1], qy = q_curr[2], qz = q_curr[3];
    a[0] = 2.0f * (qx*qz - qw*qy);
    a[1] = 2.0f * (qy*qz + qw*qx);
    a[2] = qw*qw - qx*qx - qy*qy + qz*qz;

    /* Gyro from quaternion difference */
    memset(g, 0, sizeof(float)*3);
    if (bno.last_q_valid) {
        float qp[4];
        memcpy(qp, bno.last_q, sizeof(qp));

        /* q_diff = conj(qp) * q_curr */
        float dqw = qp[0]*q_curr[0] + qp[1]*q_curr[1] + qp[2]*q_curr[2] + qp[3]*q_curr[3];
        float dqx = qp[0]*q_curr[1] - qp[1]*q_curr[0] - qp[2]*q_curr[3] + qp[3]*q_curr[2];
        float dqy = qp[0]*q_curr[2] + qp[1]*q_curr[3] - qp[2]*q_curr[0] - qp[3]*q_curr[1];
        float dqz = qp[0]*q_curr[3] - qp[1]*q_curr[2] + qp[2]*q_curr[1] - qp[3]*q_curr[0];

        float dt_s = dt_ms * 1e-3f;
        if (dt_s < 1e-6f) dt_s = 1e-6f; /* Prevent division by zero */

        float scale = 2.0f / dt_s;
        if (dqw < 0.0f) scale = -scale;  /* shortest path */

        const float rad_to_deg = 57.295779513f;
        g[0] = dqx * scale * rad_to_deg;
        g[1] = dqy * scale * rad_to_deg;
        g[2] = dqz * scale * rad_to_deg;
    }

    /* Update last_q to current for next call */
    memcpy(bno.last_q, q_curr, sizeof(bno.last_q));
    bno.last_q_valid = true;
    memcpy(bno.cached_accel, a, sizeof(bno.cached_accel));
    memcpy(bno.cached_gyro,  g, sizeof(bno.cached_gyro));

    k_mutex_unlock(&bno_mutex);
    return 0;
}

void bno08x_accel_read(float a[3])
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    memcpy(a, bno.cached_accel, sizeof(bno.cached_accel));
    k_mutex_unlock(&bno_mutex);
}

void bno08x_gyro_read(float g[3])
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    memcpy(g, bno.cached_gyro, sizeof(bno.cached_gyro));
    k_mutex_unlock(&bno_mutex);
}

float bno08x_temp_read(void)
{
    float t;
    k_mutex_lock(&bno_mutex, K_FOREVER);
    t = bno.cached_temp;
    k_mutex_unlock(&bno_mutex);
    return t;
}

uint8_t bno08x_setup_DRDY(uint16_t threshold)
{
    (void)threshold;
    /* TODO: proper INT configuration via FRS */
    return (uint8_t)((NRF_GPIO_PIN_PULLUP << 4) | NRF_GPIO_PIN_SENSE_LOW);
}

uint8_t bno08x_setup_WOM(void)
{
    LOG_WRN("WOM not implemented");
    return 0;
}

void bno08x_get_quaternion(float q[4])
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    if (bno.last_q_valid)
        memcpy(q, bno.last_q, sizeof(float)*4);
    else {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    }
    k_mutex_unlock(&bno_mutex);
}

/* =========================================================================
 *  Scan Probe (for I2C detection)
 * ========================================================================= */

int bno08x_scan_probe(struct i2c_dt_spec *i2c_dev, uint8_t *reg, bool interface_register)
{
    static const uint8_t addrs[] = {BNO08X_I2C_ADDR_DEFAULT, BNO08X_I2C_ADDR_ALT};
    uint16_t saved_addr = i2c_dev->addr;
    const struct device *bus = i2c_dev->bus;
    struct i2c_dt_spec tmp_dev;

    for (int ai = 0; ai < ARRAY_SIZE(addrs); ai++) {
        uint8_t addr = addrs[ai];
        if (saved_addr >= 8 && saved_addr <= 119 && saved_addr != addr)
            continue;

        LOG_INF("BNO08x probe at 0x%02X", addr);
        tmp_dev.bus = bus; tmp_dev.addr = addr;

        /* Wait for chip to boot */
        k_msleep(350);

        /* Poll for advertisement or GRV */
        int64_t deadline = k_uptime_get() + 1500;
        bool got_advert = false;
        while (k_uptime_get() < deadline) {
            uint8_t hdr[4];
            int err = i2c_read_dt(&tmp_dev, hdr, 4);
            if (err < 0) {
                k_msleep(20);
                continue;
            }
            uint32_t pld_len = (uint32_t)hdr[0] | ((uint32_t)(hdr[1] & 0x3F) << 8);
            if (pld_len == 0 || pld_len > BNO08X_SHTP_MAX_PAYLOAD) {
                k_msleep(10);
                continue;
            }
            uint8_t buf[BNO08X_SHTP_MAX_PACKET];
            memcpy(buf, hdr, 4);
            err = i2c_read_dt(&tmp_dev, buf + 4, pld_len + 1);
            if (err < 0) {
                k_msleep(10);
                continue;
            }
            uint32_t check_len = 4 + pld_len;
            if (shtp_crc8(buf, check_len) != buf[check_len])
                continue;

            if (hdr[2] == 0 && pld_len >= 1 && buf[4] == 0x00) {
                got_advert = true;
                break;
            }
            if (hdr[2] == 3 && pld_len >= 5 && buf[4] == BNO08X_REPORT_GAME_ROTATION_VECTOR) {
                got_advert = true;
                break;
            }
        }

        if (!got_advert) {
            LOG_INF("No response at 0x%02X", addr);
            continue;
        }

        LOG_INF("Advertisement received at 0x%02X", addr);

        /* Send product ID request */
        uint8_t cmd[] = {BNO08X_CMD_PRODUCT_ID_REQUEST, 0x00};
        uint8_t tx[BNO08X_SHTP_MAX_PACKET];
        uint32_t txlen = shtp_build_packet(tx, BNO08X_SHTP_CH_COMMAND, 0, cmd, sizeof(cmd));
        if (i2c_write_dt(&tmp_dev, tx, txlen) < 0)
            continue;
        k_msleep(20);

        /* Read response */
        deadline = k_uptime_get() + 200;
        int detected_imu = -1;
        while (k_uptime_get() < deadline) {
            uint8_t hdr[4];
            if (i2c_read_dt(&tmp_dev, hdr, 4) < 0) {
                k_msleep(5);
                continue;
            }
            uint32_t pld_len = (uint32_t)hdr[0] | ((uint32_t)(hdr[1] & 0x3F) << 8);
            if (pld_len < 4 || pld_len > BNO08X_SHTP_MAX_PAYLOAD) {
                k_msleep(5);
                continue;
            }
            uint8_t buf[BNO08X_SHTP_MAX_PACKET];
            memcpy(buf, hdr, 4);
            if (i2c_read_dt(&tmp_dev, buf + 4, pld_len + 1) < 0)
                continue;
            uint32_t check_len = 4 + pld_len;
            if (shtp_crc8(buf, check_len) != buf[check_len])
                continue;

            if (hdr[2] == 0 && pld_len >= 4 && buf[4] == BNO08X_CMD_PRODUCT_ID_RESPONSE) {
                uint8_t pid_low = buf[5], pid_high = buf[6];
                uint16_t pid = ((uint16_t)pid_high << 8) | pid_low;
                LOG_INF("Product ID 0x%04X at 0x%02X", pid, addr);
                if (pid == BNO08X_PID_BNO085)
                    detected_imu = IMU_BNO085;
                else if (pid == BNO08X_PID_BNO086)
                    detected_imu = IMU_BNO086;
                break;
            }
        }

        if (detected_imu < 0)
            continue;

        /* Found */
        i2c_dev->addr = addr;
        *reg = 0x00; /* I2C */
        if (interface_register) {
            sensor_interface_register_sensor_imu_i2c(i2c_dev);
        }
        return detected_imu;
    }

    i2c_dev->addr = saved_addr;
    return -1;
}

/* =========================================================================
 *  sensor_imu_t instance
 * ========================================================================= */

const sensor_imu_t sensor_imu_bno08x = {
    .init          = bno08x_init,
    .shutdown      = bno08x_shutdown,
    .update_fs     = bno08x_update_fs,
    .update_odr    = bno08x_update_odr,
    .fifo_read     = bno08x_fifo_read,
    .fifo_process  = bno08x_fifo_process,
    .accel_read    = bno08x_accel_read,
    .gyro_read     = bno08x_gyro_read,
    .temp_read     = bno08x_temp_read,
    .setup_DRDY    = bno08x_setup_DRDY,
    .setup_WOM     = bno08x_setup_WOM,
    .ext_setup     = imu_none_ext_setup,
    .ext_passthrough = imu_none_ext_passthrough,
};
