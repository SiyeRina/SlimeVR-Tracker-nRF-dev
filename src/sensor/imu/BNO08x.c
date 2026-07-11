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
#include <zephyr/devicetree.h>
#include <string.h>
#include <math.h>
#include <hal/nrf_gpio.h>
LOG_MODULE_REGISTER(BNO08X, LOG_LEVEL_INF);

/* =========================================================================
 *  Hardware reset via sensor VCC power-cycle
 * ========================================================================= */
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, vcc_gpios)
#include <zephyr/drivers/gpio.h>
static const struct gpio_dt_spec bno_vcc = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, vcc_gpios);
#endif

int bno08x_hardware_reset(void)
{
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, vcc_gpios)
	if (!device_is_ready(bno_vcc.port)) {
		LOG_ERR("VCC GPIO port not ready for hardware reset");
		return -1;
	}

	LOG_INF("Power-cycling sensor VCC for hardware reset...");
	gpio_pin_set_dt(&bno_vcc, 0);
	k_msleep(1000);
	gpio_pin_set_dt(&bno_vcc, 1);
	k_msleep(200);
	LOG_INF("Sensor VCC restored");
	return 0;
#else
	LOG_WRN("VCC GPIO not available, cannot hardware reset");
	return -1;
#endif
}


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
    /* SHTP 4-byte header — DOWNLINK format (host → BNO08x):
     *   [0]       = payload length LSB
     *   [1]       = len MSB (bits 5:0), bits 7:6 reserved
     *   [2]       = channel (0-3)
     *   [3]       = sequence number
     *
     * NOTE: This is DIFFERENT from the uplink (sensor → host) format,
     * where the channel is encoded in byte[1] bits 7:6 and byte[2]
     * carries the sequence number.  Using the uplink layout for
     * downlink packets routes all commands to the wrong channel. */
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

/* Read a single SHTP packet from the BNO08x over I2C.
 *
 * SHTP over I2C delivers one packet per I2C read transaction.
 * We read the maximum expected packet size in one transaction;
 * the BNO08x pads with 0xFF for any bytes the master requests
 * beyond the actual packet.  SHTP packets are non-fragmented
 * (continuation byte is always 0x00) so a single read is safe. */
static int shtp_recv(uint8_t *buf, uint8_t **payload, uint32_t *payload_len, uint8_t *channel)
{
    int err = ssi_read(SENSOR_INTERFACE_DEV_IMU, buf, BNO08X_SHTP_MAX_PACKET);
    if (err < 0)
        return -1;

    /* SHTP header (4 bytes):
     *   [0]       = payload length LSB
     *   [1]       = len MSB (bits 5:0) | channel (bits 7:6)
     *   [2]       = sequence number
     *   [3]       = continuation byte (0x00 for single segment)
     * Payload starts at buf[4]. */
    uint32_t pld_len = (uint32_t)buf[0] | ((uint32_t)(buf[1] & 0x3F) << 8);
    if (pld_len > BNO08X_SHTP_MAX_PAYLOAD) {
        LOG_WRN("Invalid payload length: %u", pld_len);
        return -1;
    }

    *payload = buf + 4;
    *payload_len = pld_len;
    *channel = (buf[1] >> 6) & 0x03;
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

    /* ---- Step A: GET_FEATURE first to "open" the CONTROL channel ---- */
    uint8_t get_cmd[] = {BNO08X_CMD_GET_FEATURE, report_id};
    LOG_INF("GET_FEATURE 0x%02X", report_id);
    int err = shtp_send(BNO08X_SHTP_CH_CONTROL, get_cmd, sizeof(get_cmd));
    if (err < 0) {
        LOG_ERR("GET_FEATURE send failed: %d", err);
        return err;
    }

    /* Listen for GET_FEATURE response (0xFC on any channel) */
    {
        uint8_t buf[BNO08X_SHTP_MAX_PACKET];
        uint8_t *payload;
        uint32_t len;
        uint8_t ch;
        int64_t dl = k_uptime_get() + 300;
        while (k_uptime_get() < dl) {
            err = shtp_recv(buf, &payload, &len, &ch);
            if (err < 0 || len == 0) { k_msleep(5); continue; }
            printk("[GET rsp] ch=%u len=%u pld[0]=0x%02X\n", ch, len, payload[0]);
            if (payload[0] == BNO08X_CMD_FEATURE_RESPONSE) {
                printk("[GET] FEATURE_RESPONSE for 0x%02X (len=%u)\n", report_id, len);
                break;
            }
            k_msleep(5);
        }
    }

    /* ---- Step B: SET_FEATURE ---- */
    printk("[SET_FEATURE] report=0x%02X interval=%u us\n", report_id, interval_us);
    err = shtp_send(BNO08X_SHTP_CH_CONTROL, cmd, sizeof(cmd));
    if (err < 0) {
        LOG_ERR("SET_FEATURE send failed: %d", err);
        return err;
    }

    /* Listen for response on ANY channel, check status byte */
    {
        uint8_t buf[BNO08X_SHTP_MAX_PACKET];
        uint8_t *payload;
        uint32_t len;
        uint8_t ch;
        int64_t deadline = k_uptime_get() + 500;
        bool got_response = false;

        while (k_uptime_get() < deadline && !got_response) {
            err = shtp_recv(buf, &payload, &len, &ch);
            if (err < 0) { k_msleep(5); continue; }
            if (len == 0) { k_msleep(5); continue; }

            printk("[SET rsp] ch=%u len=%u pld[0]=0x%02X", ch, len, payload[0]);
            if (len >= 3) {
                printk(" pld[1]=0x%02X pld[2]=0x%02X", payload[1], payload[2]);
            }
            printk("\n");

            if (payload[0] == BNO08X_CMD_FEATURE_RESPONSE && len >= 3) {
                uint8_t status = payload[2];
                printk("[SET] FEATURE_RESPONSE: report=0x%02X status=0x%02X\n",
                       payload[1], status);
                if (status == 0) {
                    got_response = true;
                    err = 0;
                } else {
                    printk("[SET] ERROR: status=0x%02X (non-zero!)\n", status);
                }
            }
            k_msleep(5);
        }

        if (got_response) {
            return 0;
        }
    }

    LOG_WRN("No valid FEATURE_RESPONSE within 500ms");
    return -ETIMEDOUT;
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
    /* SH-2 Game Rotation Vector report layout:
     *   [0]    = report_id (0x05)
     *   [1]    = sequence number
     *   [2-3]  = status (uint16)
     *   [4-7]  = delay (uint32)
     *   [8-11] = i (Q30)
     *   [12-15]= j (Q30)
     *   [16-19]= k (Q30)
     *   [20-23]= w/real (Q30) */
    q[1] = q30_to_float(payload + 8);   /* i */
    q[2] = q30_to_float(payload + 12);  /* j */
    q[3] = q30_to_float(payload + 16);  /* k */
    q[0] = q30_to_float(payload + 20);  /* real */
}

static float decode_temperature(const uint8_t *payload)
{
    /* SH-2 Temperature report (0x07):
     *   [0]    = report_id
     *   [1]    = sequence
     *   [2]    = status
     *   [3-4]  = delay
     *   [5-6]  = temperature (int16, °C * 100?) */
    int16_t raw = (int16_t)(payload[5] | (payload[6] << 8));
    return (float)raw * 0.01f;
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

    /* Init diagnostics (survive log truncation) */
    int8_t init_step;      /* last init step reached: 1=drain, 2=PID, 3=GRV, 4=temp, 5=input, 6=done, -N=fail */
    int8_t init_err;       /* error code at failure step */

    /* Temperature report enabled? */
    bool temp_enabled;
} bno08x_state_t;

static bno08x_state_t bno;
static bool g_probe_found_alive; /* true if probe found chip already running */

void bno08x_probe_mark_alive(void) {
    g_probe_found_alive = true;
}

bool bno08x_is_inited(void)
{
    return bno.inited;
}

int8_t bno08x_get_init_step(void)
{
    return bno.init_step;
}

int8_t bno08x_get_init_err(void)
{
    return bno.init_err;
}

/* =========================================================================
 *  sensor_imu_t callbacks
 * ========================================================================= */

int bno08x_init(float clock_rate, float accel_time, float gyro_time,
                float *accel_actual_time, float *gyro_actual_time)
{
    k_mutex_lock(&bno_mutex, K_FOREVER);
    int ret = -1;

    (void)clock_rate;

    LOG_INF("BNO08x init: accel %.3f s, gyro %.3f s", (double)accel_time, (double)gyro_time);

    uint8_t pkt_buf[BNO08X_SHTP_MAX_PACKET];
    uint8_t *payload;
    uint32_t payload_len;

    /* Reset diagnostics */
    bno.init_step = 0;
    bno.init_err = 0;

    /* Step 1: Drain stale boot traffic (probe already read advertisement).
     * Drain until quiet for 100ms, max 500ms total. */
    LOG_INF("[init step 1] Draining stale traffic...");
    bno.init_step = 1;

    {
        int64_t drain_deadline = k_uptime_get() + 500;
        int64_t last_pkt = k_uptime_get();
        int drained = 0;
        while (k_uptime_get() < drain_deadline) {
            uint8_t ch;
            int rv = shtp_recv(pkt_buf, &payload, &payload_len, &ch);
            if (rv < 0 || payload_len == 0) {
                if ((k_uptime_get() - last_pkt) > 100) break; /* quiet */
                k_msleep(10);
                continue;
            }
            last_pkt = k_uptime_get();
            drained++;
            LOG_INF("  Drain #%d: ch=%u len=%u pld[0]=0x%02X",
                    drained, ch, payload_len,
                    payload_len > 0 ? payload[0] : 0);
        }
        LOG_INF("  Drain done: %d packets", drained);
    }

    /* Step 2: Product ID handshake.
     * Send 0xF9 on COMMAND channel.  This tells the SH-2 firmware the
     * host is present and ready.  Response may be 0xF8 (Product ID)
     * or 0x01 (heartbeat) — either confirms the handshake. */
    LOG_INF("[init step 2] Product ID handshake...");
    bno.init_step = 2;

    {
        uint8_t pid_req[] = {BNO08X_CMD_PRODUCT_ID_REQUEST, 0x00};
        ret = shtp_send(BNO08X_SHTP_CH_COMMAND, pid_req, sizeof(pid_req));
        if (ret < 0) {
            LOG_ERR("  PID request send failed: %d", ret);
            bno.init_err = ret;
            goto unlock;
        }

        uint8_t ch;
        ret = shtp_wait_for_channel(pkt_buf, &payload, &payload_len,
                                    BNO08X_SHTP_CH_COMMAND, 500);
        if (ret < 0) {
            LOG_WRN("  No response to PID request");
            /* Non-fatal: proceed anyway */
        } else {
            LOG_INF("  PID response: ch=%u len=%u pld[0]=0x%02X",
                    BNO08X_SHTP_CH_COMMAND, payload_len,
                    payload_len > 0 ? payload[0] : 0);
            if (payload_len >= 4) {
                LOG_HEXDUMP_INF(payload, payload_len < 32 ? payload_len : 32,
                               "  raw");
            }
        }
    }

    /* Step 3: Enable Game Rotation Vector via SET_FEATURE on CONTROL channel. */
    /* Compute GRV interval from requested ODR */
    float desired_odr = 1.0f / (accel_time < gyro_time ? accel_time : gyro_time);
    if (desired_odr < 1.0f) desired_odr = 1.0f;
    if (desired_odr > 400.0f) desired_odr = 400.0f;
    uint32_t interval_us = (uint32_t)(1e6f / desired_odr);
    if (interval_us < 2500) interval_us = 2500;

    LOG_INF("[init step 3] SET_FEATURE GRV (interval=%u us)...", interval_us);
    bno.init_step = 3;

    ret = bno08x_set_report(BNO08X_REPORT_GAME_ROTATION_VECTOR, interval_us);
    if (ret < 0) {
        LOG_ERR("  Failed to enable GRV: %d", ret);
        bno.init_err = ret;
        goto unlock;
    }
    LOG_INF("  GRV enabled");

    /* Step 4: Enable temperature report (optional) */
    LOG_INF("[init step 4] SET_FEATURE temperature...");
    bno.init_step = 4;

    ret = bno08x_set_report(BNO08X_REPORT_TEMPERATURE, 1000000); /* 1 Hz */
    if (ret == 0) {
        bno.temp_enabled = true;
        LOG_INF("  Temperature report enabled");
    } else {
        bno.temp_enabled = false;
        LOG_INF("  Temperature report skipped (ret=%d)", ret);
    }

    /* Step 5: Wait for first sensor report on INPUT channel. */
    LOG_INF("[init step 5] Waiting for first INPUT report...");
    bno.init_step = 5;

    {
        /* Scan ALL channels for first sensor report (any report ID >= 0x02) */
        int64_t deadline = k_uptime_get() + 2000;
        bool got_report = false;
        LOG_INF("[init step 5] Waiting for first sensor report on any channel...");
        while (k_uptime_get() < deadline && !got_report) {
            uint8_t ch;
            int rv = shtp_recv(pkt_buf, &payload, &payload_len, &ch);
            if (rv < 0 || payload_len == 0) {
                k_msleep(10);
                continue;
            }
            uint8_t rid = payload_len > 0 ? payload[0] : 0;
            printk("[step5] ch=%u len=%u id=0x%02X\n", ch, payload_len, rid);
            if (rid >= 0x02 && rid <= 0x20) {
                LOG_INF("  First report: ch=%u id=0x%02X len=%u", ch, rid, payload_len);
                got_report = true;
            }
        }
        if (!got_report) {
            LOG_WRN("  No sensor report within 2s (only non-data packets)");
        }
    }

    /* Step 6: Mark init done. */
    LOG_INF("[init step 6] Finalizing...");
    bno.init_step = 6;

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

    LOG_INF("BNO08x init done: GRV=%.1fHz temp=%s", (double)actual_odr,
            bno.temp_enabled ? "yes" : "no");
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
        LOG_INF("ODR updated to %.1f Hz", (double)(1.0f / bno.actual_time));
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

    /* Diagnostic: count calls and log first few */
    static int call_count;
    call_count++;
    if (call_count <= 10) {
        printk("[FIFO] call #%d\n", call_count);
    }

    int64_t start = k_uptime_get();
    while (samples < max_samples) {
        uint8_t pkt_buf[BNO08X_SHTP_MAX_PACKET];
        uint8_t *payload;
        uint32_t payload_len;
        uint8_t channel;

        int err = shtp_recv(pkt_buf, &payload, &payload_len, &channel);
        if (err < 0) {
            static int recv_errs;
            recv_errs++;
            if (recv_errs <= 3) {
                printk("[FIFO] shtp_recv FAIL #%d\n", recv_errs);
            }
            break;
        }

        /* Accept sensor reports on ANY channel — not just INPUT (3).
         * The BNO08x may send GRV data on COMMAND (0) after FRS handshake.
         * Skip only when no valid report ID is found and it's clearly
         * not sensor data (empty or non-report). */
        if (payload_len == 0) {
            continue;
        }

        uint8_t report_id = payload[0];

        /* For non-INPUT channels, check if this looks like sensor data */
        if (channel != BNO08X_SHTP_CH_INPUT) {
            /* Known sensor report IDs: 0x02-0x15 range covers most reports */
            bool is_report = (report_id >= 0x02 && report_id <= 0x20);
            if (!is_report) {
                static int non_input;
                non_input++;
                if (non_input <= 5) {
                    printk("[FIFO] non-data ch=%u id=0x%02X len=%u\n",
                           channel, report_id, payload_len);
                }
                if (k_uptime_get() - start > 50) {
                    break;
                }
                continue;
            }
            /* Fall through — treat as sensor data */
        }

        /* DIAGNOSTIC: Log every sensor data packet */
        {
            static int input_count;
            if (++input_count <= 20) {
                LOG_INF("REPORT #%d: ch=%u id=0x%02X len=%u", input_count,
                        channel, report_id, payload_len);
                if (payload_len > 0 && payload_len <= 32) {
                    LOG_HEXDUMP_INF(payload, payload_len, "  data");
                }
            }
        }

        if (report_id == BNO08X_REPORT_TEMPERATURE && payload_len >= 7) {
            float t = decode_temperature(payload);
            bno.cached_temp = t;
            static int temp_count;
            if (++temp_count <= 3)
                LOG_INF("TEMP: %.2f C", (double)t);
            continue;
        }

        if (report_id != BNO08X_REPORT_GAME_ROTATION_VECTOR || payload_len < 24) {
            static int non_grv;
            if (++non_grv <= 5)
                LOG_WRN("skip INPUT id=0x%02X len=%u (not GRV)", report_id, payload_len);
            continue;
        }

        float q[4];
        decode_grv(payload, q);

        /* Delay is uint32 at payload[4..7], in microseconds */
        uint32_t delay_us = (uint32_t)payload[4]
                          | ((uint32_t)payload[5] << 8)
                          | ((uint32_t)payload[6] << 16)
                          | ((uint32_t)payload[7] << 24);
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

        /* Debug: log first few GRV samples */
        static int grv_sample_count;
        if (grv_sample_count < 5) {
            float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
            LOG_INF("GRV #%d: q=[%.3f %.3f %.3f %.3f] norm=%.3f dt=%.2fms dl=%u",
                    grv_sample_count, (double)q[0], (double)q[1],
                    (double)q[2], (double)q[3], (double)n,
                    (double)dt_ms, delay_us);
            grv_sample_count++;
        }

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
    uint16_t orig_addr = saved_addr;
    const struct device *bus = i2c_dev->bus;
    struct i2c_dt_spec tmp_dev;
    bool retried = false;

retry:
    for (int ai = 0; ai < ARRAY_SIZE(addrs); ai++) {
        uint8_t addr = addrs[ai];
        if (saved_addr >= 8 && saved_addr <= 119 && saved_addr != addr)
            continue;

        /* Quick write-only probe first — avoids consuming SHTP data
         * that the passive listen needs. */
        tmp_dev.bus = bus; tmp_dev.addr = addr;
        {
            uint8_t dummy = 0;
            if (i2c_write_dt(&tmp_dev, &dummy, 1) != 0)
                continue; /* no device at this address, skip to next */
        }

        LOG_INF("BNO08x probe at 0x%02X", addr);

        /* Passive detection: just listen for SHTP traffic for up to
         * 1.2 seconds.  Any valid SHTP packet from 0x4A or 0x4B proves
         * a BNO08x is present — no TX, no RESET.  bno08x_init handles
         * the full init sequence (RESET + advertisement wait) later.
         *
         * If the chip isn't powered yet when the probe starts, a longer
         * timeout (1200ms vs the original 600ms) gives it time to boot. */
        int64_t dl = k_uptime_get() + 1200;
        bool found = false;
        while (k_uptime_get() < dl) {
            uint8_t buf[BNO08X_SHTP_MAX_PACKET];
            int err = i2c_read_dt(&tmp_dev, buf, BNO08X_SHTP_MAX_PACKET);
            if (err < 0) { k_msleep(50); continue; }
            uint32_t pl = (uint32_t)buf[0] | ((uint32_t)(buf[1] & 0x3F) << 8);
            if (pl < 4 || pl > BNO08X_SHTP_MAX_PAYLOAD) { k_msleep(10); continue; }
            uint8_t ch = (buf[1] >> 6) & 0x03;
            uint8_t *pld = buf + 4; /* SHTP header is 4 bytes */

            if (pl > 100) {
                LOG_INF("BNO08x advertisement at 0x%02X (%u bytes) pld[0]=0x%02X", addr, pl, pld[0]);
                LOG_HEXDUMP_INF(pld, (pl < 48) ? pl : 48, "ad pld[0..47]");
                /* Parse TLV entries in the advertisement payload */
                uint32_t pos = 0;
                while (pos + 1 < pl) {
                    uint8_t tag = pld[pos]; uint8_t rlen = pld[pos + 1];
                    if (pos + 2 + rlen > pl) break;
                    if (tag == 0xF8 && rlen >= 2) {
                        uint16_t pid = ((uint16_t)pld[pos + 3] << 8) | pld[pos + 2];
                        LOG_INF("Product ID 0x%04X from advertisement at 0x%02X", pid, addr);
                        break;
                    }
                    pos += 2 + rlen;
                }
                found = true;
                break;
            } else {
                LOG_INF("BNO08x alive at 0x%02X (ch=%u len=%u pld[0]=0x%02X)", addr, ch, pl, pld[0]);
                bno08x_probe_mark_alive();
                found = true;
                break;
            }
        }

        if (!found) {
            /* Passive listen timed out — chip may be idle (retry
             * scenario).  Send a product ID request to wake the chip.
             * We do NOT send RESET here; bno08x_init handles that. */
            uint8_t cmd[] = {BNO08X_CMD_PRODUCT_ID_REQUEST, 0x00};
            uint8_t tx[BNO08X_SHTP_MAX_PACKET];
            uint32_t txlen = shtp_build_packet(tx, BNO08X_SHTP_CH_COMMAND, 0, cmd, sizeof(cmd));
            if (i2c_write_dt(&tmp_dev, tx, txlen) == 0) {
                int64_t wake_dl = k_uptime_get() + 400;
                while (k_uptime_get() < wake_dl) {
                    uint8_t buf[BNO08X_SHTP_MAX_PACKET];
                    if (i2c_read_dt(&tmp_dev, buf, BNO08X_SHTP_MAX_PACKET) < 0) {
                        k_msleep(20); continue;
                    }
                    uint32_t pl = (uint32_t)buf[0] | ((uint32_t)(buf[1] & 0x3F) << 8);
                    if (pl >= 4 && pl <= BNO08X_SHTP_MAX_PAYLOAD) {
                        LOG_INF("BNO08x woken at 0x%02X (len=%u)", addr, pl);
                        found = true;
                        break;
                    }
                    k_msleep(10);
                }
            }
        }

        if (found) {
            i2c_dev->addr = addr; *reg = 0x00;
            if (interface_register) sensor_interface_register_sensor_imu_i2c(i2c_dev);
            return IMU_BNO085;
        }
    }

    /* If a retained address from a previous boot filtered out both BNO08x
     * addresses, retry once with the filter cleared (matches the fallback
     * pattern in sensor_scan_i2c). */
    if (!retried && orig_addr >= 8 && orig_addr <= 119) {
        retried = true;
        saved_addr = 0;
        goto retry;
    }

    i2c_dev->addr = orig_addr;
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
