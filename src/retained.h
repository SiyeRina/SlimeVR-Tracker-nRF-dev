/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RETAINED_H_
#define RETAINED_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if CONFIG_SENSOR_USE_TCAL
// A single point in temperature calibration data
struct TempCalPoint {
	float temp;    // The temperature for this point
	float bias[3]; // The gyro bias (x, y, z)
};
#endif

struct retained_data {
	/* The build version of the firmware that last updated the
	 * retained data.
	 */
	uint32_t build_timestamp;

	/* The uptime from the current session the last time the
	 * retained data was updated.
	 */
	uint64_t uptime_latest;

	/* Cumulative uptime from all previous sessions up through
	 * uptime_latest of this session.
	 */
	uint64_t uptime_sum;

	/* Battery statistics.  Tracking for discharge curve only begins
	 * after ~3% discharged.  If battery_pptt has changed significantly
	 * compared to min_battery_pptt since the last update,
	 * the statistics are cleared and may be saved to NVS.
	 */
	int16_t max_battery_pptt;
	int16_t min_battery_pptt;

	/* Battery uptime from last retained update */
	uint64_t battery_uptime_latest;

	/* Cumulative runtime */
	uint64_t battery_runtime_sum;

	/* Last interval stored in NVS */
	int16_t battery_pptt_saved;
	uint64_t battery_runtime_saved;

	/* Calibrated discharge curve */
	int16_t battery_pptt_curve[18];

	uint8_t reboot_counter;
	uint8_t paired_addr[8];

	uint8_t sensor_data[128];

	float accelBias[3];
	float gyroBias[3];
	float magBias[3];
	float magBAinv[4][3];
	float accBAinv[4][3];
	float gyroSensScale[3]; // Gyro sensitivity


	uint8_t fusion_id; // fusion_data_stored
	uint8_t fusion_data[784];

	uint16_t imu_addr;
	uint16_t mag_addr;

	uint8_t imu_reg;
	uint8_t mag_reg;

	uint8_t rf_channel; // RF channel (0-100), 0xFF means use default

	bool mag_enabled;

	// Online magnetometer calibration runtime state.
	// Persists across WoM resumes so online mag cal does not re-enter
	// early bootstrap after every wake, but is cleared on full reboot/shutdown.
	struct {
		float last_buf_avg_norm;
		uint8_t update_count;
		uint8_t reserved[3];
	} onlineMagState;

#if CONFIG_SENSOR_USE_TCAL
	bool tcal_enabled; // Temperature calibration compensation enabled
	float gyroTemp;

	#define TCAL_BUFFER_SIZE                                                                                               \
		(int)((CONFIG_SENSOR_POLY_TEMP_MAX - CONFIG_SENSOR_POLY_TEMP_MIN) * CONFIG_SENSOR_POLY_STEPS_PER_DEGREE)
		struct TempCalPoint tempCalPoints[TCAL_BUFFER_SIZE];
	float tempCalCoeffs[3][CONFIG_SENSOR_POLY_DEGREE + 1];
	float tempCalCorrectionOffset[3];

	struct {
		uint16_t count;
		bool valid;
		uint8_t degree;
	} tempCalState;

	// Boot calibration state (runtime only, persists in WoM but resets on full reboot)
	struct {
		bool enabled;              // Feature enabled
		bool completed;            // Completed for this boot
		uint8_t attempt_count;     // Number of attempts
		float doffset[3];          // Calculated D_offset (runtime only)
		bool doffset_valid;        // D_offset is valid
	} bootCalState;
#endif

	/* CRC used to validate the retained data.  This must be
	 * stored little-endian, and covers everything up to but not
	 * including this field.
	 */
	uint32_t crc;

	/* ==== FIELDS BELOW ARE NOT INCLUDED IN CRC CALCULATION ==== */
	/* These fields are intentionally placed after the CRC so they can
	 * be modified without invalidating the CRC. This is important for
	 * watchdog state which must persist across unexpected resets.
	 */

	// Watchdog state (persists across WDT resets, outside CRC validation)
	struct {
		uint8_t reset_count;           // WDT consecutive reset count
		uint8_t last_failed_channel;   // Last channel that failed to feed
		uint32_t last_reset_uptime;    // System uptime at last WDT reset (ms)
		uint32_t total_wdt_resets;     // Cumulative WDT reset count (for debugging)
		uint32_t magic;                // Magic number to validate watchdog state
	} watchdog_state;

	// Last reset diagnostics (persists across all reset types, outside CRC)
	struct {
		uint32_t resetreas;            // RESETREAS register value from last reset
		uint8_t  reboot_counter;       // reboot_counter at time of last reset
		uint8_t  sreq_source;          // Source of last SREQ (if RESETREAS has SREQ bit)
		uint16_t sreq_flags;           // Flags: bit0=tagged by app, bit1=auto-detected by WDT
		uint32_t total_resets;         // Monotonically increasing reset counter
		uint32_t magic;                // Magic number to validate
	} last_reset_info;

	// Zephyr fatal error diagnostics (persists across resets, outside CRC)
	struct {
		uint32_t reason;               // k_sys_fatal_error_handler reason code
		uint32_t pc;                   // Program counter at fault (0 if unavailable)
		uint32_t assert_line;          // __LINE__ of failed assertion (0 if not an assert)
		uint32_t assert_file_addr;     // Flash address of __FILE__ string (0 if not an assert)
		uint32_t magic;                // Magic number to validate
	} fatal_error_info;
};

/* sreq_flags bit definitions */
#define SREQ_FLAG_TAGGED    (1U << 0)  /* lastreset_tag_sreq_source() was called */
#define SREQ_FLAG_DETECTED  (1U << 1)  /* watchdog_early_check auto-detected untagged SREQ */

/* Magic number to validate watchdog state */
#define WATCHDOG_STATE_MAGIC 0x57445447  /* "WDTG" in ASCII */

/* Magic number to validate last_reset_info */
#define LAST_RESET_INFO_MAGIC 0x4C525354  /* "LRST" in ASCII */

/* Magic number to validate fatal_error_info */
#define FATAL_ERROR_INFO_MAGIC 0x4641544C  /* "FATL" in ASCII */

/* Sources for software-requested resets (SREQ) */
enum sreq_source {
	SREQ_SRC_UNKNOWN = 0,          /* untagged (legacy / cold boot) */
	SREQ_SRC_CONSOLE_REBOOT,       /* console.c "reboot" command */
	SREQ_SRC_CONSOLE_SENS_RESET,   /* console.c "sens reset" */
	SREQ_SRC_CONSOLE_DFU_CMD,      /* console.c dfu commands */
	SREQ_SRC_WDT_DFU_ENTER,        /* watchdog.c DFU threshold exceeded */
	SREQ_SRC_SENSOR_INIT_FAIL,     /* sensor.c sensor init failure */
	SREQ_SRC_SENSOR_MAG_TOGGLE,    /* sensor.c mag enable/disable reboot */
	SREQ_SRC_SENSOR_PKT_ERR,       /* sensor.c packet error threshold */
	SREQ_SRC_ESB_REMOTE_REBOOT,    /* esb.c remote REBOOT command */
	SREQ_SRC_ESB_REMOTE_DFU,       /* esb.c remote DFU command */
	SREQ_SRC_ESB_REMOTE_DFU_OTA,   /* esb.c remote DFU_OTA command */
	SREQ_SRC_BTN_SINGLE_CLICK,     /* system.c button single click */
	SREQ_SRC_USER_SHUTDOWN_ALT,    /* system.c user_shutdown fallback to reboot */
	SREQ_SRC_SYSTEM_DFU_UF2,       /* system.c DFU UF2 mode */
	SREQ_SRC_SYSTEM_DFU_OTA,       /* system.c DFU OTA mode */
	SREQ_SRC_POWER_IMMEDIATE,      /* power.c immediate reboot request */
	SREQ_SRC_ESB_OTA_COMPLETE,     /* esb_ota.c OTA complete */
	SREQ_SRC_ESB_OTA_DFU,          /* esb_ota.c DFU entry */
	SREQ_SRC_ESB_OTA_FAIL,         /* esb_ota.c OTA failure */
	SREQ_SRC_COUNT
};

/* Up to 4 KB of retained data allowed right now.
 */
#define RETAINED_SIZE (sizeof(struct retained_data))

/* For simplicity in the sample just allow anybody to see and
 * manipulate the retained state.
 */
extern struct retained_data *retained;

/* Check whether the retained data is valid, and if not reset it.
 *
 * @return true if and only if the data was valid and reflects state
 * from previous sessions.
 */
bool retained_validate(void);

/* Update any generic retained state and recalculate its checksum so
 * subsequent boots can verify the retained state.
 */
void retained_update(void);

#endif /* RETAINED_H_ */
