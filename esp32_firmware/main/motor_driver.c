/**
 * @file motor_driver.c
 * @brief FOC ESC motor driver implementation (ESP-IDF / ESP32).
 *
 * Communication protocol:
 *   Each ESC is connected to a dedicated UART port.  Commands are sent as
 *   ASCII frames:  "RPM <value>\r\n"
 *   The ESC replies with an ACK: "OK\r\n"
 *
 * UART pin mapping (configurable via menuconfig / sdkconfig):
 *   MOTOR_FL  →  UART1  TX=GPIO17  RX=GPIO16
 *   MOTOR_FR  →  UART2  TX=GPIO32  RX=GPIO33
 *   MOTOR_RL  →  UART1  TX=GPIO17  RX=GPIO16  (shared bus, address 2)
 *   MOTOR_RR  →  UART2  TX=GPIO32  RX=GPIO33  (shared bus, address 3)
 */

#include "motor_driver.h"

#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"

#define TAG "motor_driver"

/* UART configuration */
#define UART_FL  UART_NUM_1
#define UART_FR  UART_NUM_2
#define UART_RL  UART_NUM_1
#define UART_RR  UART_NUM_2

#define UART_TX_FL 17
#define UART_RX_FL 16
#define UART_TX_FR 32
#define UART_RX_FR 33

#define UART_BAUD    115200
#define UART_BUF_SZ  256

/** Motor addresses on shared UART buses (ESC firmware convention). */
static const uint8_t k_motor_addr[MOTOR_COUNT] = {1, 1, 2, 2};

/** UART ports for each motor. */
static const uart_port_t k_uart[MOTOR_COUNT] = {
    UART_FL, UART_FR, UART_RL, UART_RR
};

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

static float _clamp_rpm(float rpm)
{
    if (rpm >  MOTOR_MAX_RPM) return  MOTOR_MAX_RPM;
    if (rpm < -MOTOR_MAX_RPM) return -MOTOR_MAX_RPM;
    return rpm;
}

static int _send_rpm(motor_id_t id, float rpm)
{
    char frame[32];
    int len = snprintf(frame, sizeof(frame),
                       "ADDR %u RPM %.1f\r\n", k_motor_addr[id], rpm);
    if (len < 0 || (size_t)len >= sizeof(frame)) {
        ESP_LOGE(TAG, "Frame formatting error for motor %d", id);
        return -1;
    }
    int ret = uart_write_bytes(k_uart[id], frame, (size_t)len);
    if (ret < 0) {
        ESP_LOGE(TAG, "UART write error for motor %d: %d", id, ret);
        return ret;
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

int motor_driver_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    /* UART1 — FL + RL motors */
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,
                                 UART_TX_FL, UART_RX_FL,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_BUF_SZ, 0, 0, NULL, 0));

    /* UART2 — FR + RR motors */
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2,
                                 UART_TX_FR, UART_RX_FR,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, UART_BUF_SZ, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "Motor driver initialised — %d motors ready.", MOTOR_COUNT);
    return 0;
}

int motor_set_rpm(motor_id_t id, float rpm)
{
    if (id >= MOTOR_COUNT) {
        ESP_LOGE(TAG, "Invalid motor id: %d", id);
        return -1;
    }
    return _send_rpm(id, _clamp_rpm(rpm));
}

int motor_set_all_rpm(const float rpms[MOTOR_COUNT])
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        int ret = _send_rpm((motor_id_t)i, _clamp_rpm(rpms[i]));
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

void motor_estop(void)
{
    ESP_LOGW(TAG, "Emergency stop — all motors → 0 RPM");
    const float zero[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    motor_set_all_rpm(zero);
}
