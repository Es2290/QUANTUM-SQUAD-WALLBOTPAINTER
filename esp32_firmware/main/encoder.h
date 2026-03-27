/**
 * @file encoder.h
 * @brief High-resolution quadrature encoder interface.
 *
 * Each wheel is fitted with a 4096-tick-per-revolution encoder.  The ESP32
 * PCNT (pulse counter) peripheral is used for hardware-assisted counting,
 * avoiding any CPU overhead at high speeds.
 */

#pragma once

#include <stdint.h>

/** Number of encoders (one per motor). */
#define ENCODER_COUNT 4U

/**
 * @brief Initialise all encoder peripherals.
 * @return 0 on success, negative error code otherwise.
 */
int encoder_init(void);

/**
 * @brief Read the absolute tick count for a single encoder.
 *
 * The counter wraps at INT32_MAX / INT32_MIN.
 *
 * @param channel  Encoder channel index (0–3, matching motor_id_t).
 * @return Signed tick count since last reset.
 */
int32_t encoder_read(uint8_t channel);

/**
 * @brief Read all encoder tick counts into an output array.
 *
 * @param ticks  Output array of ENCODER_COUNT int32_t values.
 */
void encoder_read_all(int32_t ticks[ENCODER_COUNT]);

/**
 * @brief Reset all encoder counters to zero.
 */
void encoder_reset_all(void);

/** Ticks per full wheel revolution (matches hardware encoder). */
#define ENCODER_TICKS_PER_REV 4096
