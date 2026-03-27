/**
 * @file motor_driver.h
 * @brief FOC ESC motor driver interface for four brushless motors.
 *
 * Each motor is driven by a separate FOC ESC controller connected via UART.
 * The driver exposes a simple RPM set-point API; the ESC handles the
 * field-oriented control internally.
 */

#pragma once

#include <stdint.h>

/** Number of brushless motors on the robot. */
#define MOTOR_COUNT 4U

/** Motor indices. */
typedef enum {
    MOTOR_FL = 0, /**< Front-left  */
    MOTOR_FR = 1, /**< Front-right */
    MOTOR_RL = 2, /**< Rear-left   */
    MOTOR_RR = 3, /**< Rear-right  */
} motor_id_t;

/**
 * @brief Initialise the motor driver (UART ports, GPIO).
 * @return 0 on success, negative error code otherwise.
 */
int motor_driver_init(void);

/**
 * @brief Set the target RPM for a single motor.
 *
 * @param id    Motor identifier (0–3).
 * @param rpm   Target speed in RPM.  Positive = forward, negative = reverse.
 *              Values are clamped to ±MOTOR_MAX_RPM.
 * @return 0 on success, negative error code otherwise.
 */
int motor_set_rpm(motor_id_t id, float rpm);

/**
 * @brief Set target RPM for all four motors simultaneously.
 *
 * @param rpms  Array of MOTOR_COUNT RPM values (FL, FR, RL, RR).
 * @return 0 on success, negative error code otherwise.
 */
int motor_set_all_rpm(const float rpms[MOTOR_COUNT]);

/**
 * @brief Emergency-stop all motors (sets RPM to 0 immediately).
 */
void motor_estop(void);

/** Maximum allowable RPM (hardware limit). */
#define MOTOR_MAX_RPM 300.0f
