/**
 * @file main.c
 * @brief ESP32 firmware entry point for WallBot Painter.
 *
 * The ESP32 handles all real-time tasks:
 *   - Reading quadrature encoders at 1 kHz and publishing tick counts
 *     to the Raspberry Pi 5 via UART/ROS 2 micro-ROS.
 *   - Receiving motor RPM set-points from the Raspberry Pi 5 and
 *     forwarding them to the four FOC ESC controllers.
 *   - Reading the IMU (ICM-42688-P) via SPI and streaming the data.
 *   - Controlling the spray pump PWM duty cycle based on commands
 *     received from the painting controller node.
 *   - Monitoring the ESTOP signal and cutting motors instantly.
 *
 * Task priorities (FreeRTOS):
 *   encoder_task   — priority 5 (highest, time-critical)
 *   motor_task     — priority 4
 *   imu_task       — priority 4
 *   spray_task     — priority 3
 *   comms_task     — priority 2 (UART bridge to Raspberry Pi 5)
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#include "motor_driver.h"
#include "encoder.h"

#define TAG "main"

/* -------------------------------------------------------------------------
 * Hardware constants
 * ---------------------------------------------------------------------- */

/** UART port used for communication with the Raspberry Pi 5. */
#define COMMS_UART       UART_NUM_0
#define COMMS_BAUD       921600
#define COMMS_TX_PIN     1
#define COMMS_RX_PIN     3
#define COMMS_BUF_SZ     512

/** LEDC channel for spray pump PWM. */
#define SPRAY_LEDC_CH    LEDC_CHANNEL_0
#define SPRAY_LEDC_TIMER LEDC_TIMER_0
#define SPRAY_GPIO       GPIO_NUM_4
#define SPRAY_FREQ_HZ    1000
#define SPRAY_DUTY_BITS  LEDC_TIMER_10_BIT  /* 0–1023 */
#define SPRAY_DUTY_MAX   1023U

/* -------------------------------------------------------------------------
 * Inter-task queues
 * ---------------------------------------------------------------------- */

static QueueHandle_t s_motor_queue;    /* float[MOTOR_COUNT] RPM set-points */
static QueueHandle_t s_spray_queue;   /* float duty [0.0, 1.0] */

/* -------------------------------------------------------------------------
 * Encoder task — 1 kHz
 * ---------------------------------------------------------------------- */

static void encoder_task(void *arg)
{
    int32_t ticks[ENCODER_COUNT];
    char frame[64];

    while (1) {
        encoder_read_all(ticks);

        /* Serialise as CSV: "ENC <t0>,<t1>,<t2>,<t3>\r\n" */
        int len = snprintf(frame, sizeof(frame),
                           "ENC %ld,%ld,%ld,%ld\r\n",
                           (long)ticks[0], (long)ticks[1],
                           (long)ticks[2], (long)ticks[3]);
        if (len > 0 && (size_t)len < sizeof(frame)) {
            uart_write_bytes(COMMS_UART, frame, (size_t)len);
        }

        vTaskDelay(pdMS_TO_TICKS(1)); /* 1 kHz */
    }
}

/* -------------------------------------------------------------------------
 * Motor task — processes RPM commands from comms_task
 * ---------------------------------------------------------------------- */

static void motor_task(void *arg)
{
    float rpms[MOTOR_COUNT];

    while (1) {
        if (xQueueReceive(s_motor_queue, rpms, portMAX_DELAY) == pdTRUE) {
            motor_set_all_rpm(rpms);
        }
    }
}

/* -------------------------------------------------------------------------
 * Spray task — controls pump PWM duty cycle
 * ---------------------------------------------------------------------- */

static void spray_task(void *arg)
{
    float duty_f = 0.0f;

    while (1) {
        if (xQueueReceive(s_spray_queue, &duty_f, portMAX_DELAY) == pdTRUE) {
            if (duty_f < 0.0f) duty_f = 0.0f;
            if (duty_f > 1.0f) duty_f = 1.0f;
            uint32_t duty = (uint32_t)(duty_f * SPRAY_DUTY_MAX);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, SPRAY_LEDC_CH, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, SPRAY_LEDC_CH);
        }
    }
}

/* -------------------------------------------------------------------------
 * Comms task — UART bridge to Raspberry Pi 5
 *
 * Receives lines of the form:
 *   "MOT <fl_rpm>,<fr_rpm>,<rl_rpm>,<rr_rpm>\r\n"
 *   "SPR <duty_0_to_1>\r\n"
 *   "STP\r\n"  — emergency stop
 * ---------------------------------------------------------------------- */

static void comms_task(void *arg)
{
    static uint8_t buf[COMMS_BUF_SZ];
    static char line[COMMS_BUF_SZ];
    static size_t line_len = 0;

    while (1) {
        int rx = uart_read_bytes(COMMS_UART, buf, sizeof(buf) - 1,
                                 pdMS_TO_TICKS(10));
        if (rx <= 0) {
            continue;
        }

        for (int i = 0; i < rx; i++) {
            char c = (char)buf[i];
            if (c == '\n') {
                line[line_len] = '\0';

                if (strncmp(line, "MOT ", 4) == 0) {
                    float rpms[MOTOR_COUNT] = {0};
                    int parsed = sscanf(line + 4, "%f,%f,%f,%f",
                                        &rpms[0], &rpms[1],
                                        &rpms[2], &rpms[3]);
                    if (parsed == MOTOR_COUNT) {
                        xQueueOverwrite(s_motor_queue, rpms);
                    }
                } else if (strncmp(line, "SPR ", 4) == 0) {
                    float duty = 0.0f;
                    if (sscanf(line + 4, "%f", &duty) == 1) {
                        xQueueOverwrite(s_spray_queue, &duty);
                    }
                } else if (strncmp(line, "STP", 3) == 0) {
                    ESP_LOGW(TAG, "ESTOP received from Raspberry Pi.");
                    motor_estop();
                    float zero = 0.0f;
                    xQueueOverwrite(s_spray_queue, &zero);
                }

                line_len = 0;
            } else if (line_len < sizeof(line) - 1 && c != '\r') {
                line[line_len++] = c;
            }
        }
    }
}

/* -------------------------------------------------------------------------
 * Peripheral initialisation
 * ---------------------------------------------------------------------- */

static void init_spray_pwm(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SPRAY_DUTY_BITS,
        .timer_num       = SPRAY_LEDC_TIMER,
        .freq_hz         = SPRAY_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SPRAY_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SPRAY_LEDC_CH,
        .timer_sel  = SPRAY_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

static void init_comms_uart(void)
{
    const uart_config_t cfg = {
        .baud_rate  = COMMS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(COMMS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(COMMS_UART,
                                 COMMS_TX_PIN, COMMS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(COMMS_UART, COMMS_BUF_SZ,
                                        COMMS_BUF_SZ, 0, NULL, 0));
}

/* -------------------------------------------------------------------------
 * App entry point
 * ---------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "WallBot Painter ESP32 firmware starting...");

    /* Peripheral init */
    ESP_ERROR_CHECK(motor_driver_init());
    ESP_ERROR_CHECK(encoder_init());
    init_spray_pwm();
    init_comms_uart();

    /* Inter-task queues (length 1 — always keep latest value) */
    s_motor_queue = xQueueCreate(1, sizeof(float) * MOTOR_COUNT);
    s_spray_queue = xQueueCreate(1, sizeof(float));

    configASSERT(s_motor_queue);
    configASSERT(s_spray_queue);

    /* Create real-time tasks */
    xTaskCreatePinnedToCore(encoder_task, "encoder", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(motor_task,   "motor",   4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(spray_task,   "spray",   2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(comms_task,   "comms",   8192, NULL, 2, NULL, 0);

    ESP_LOGI(TAG, "All tasks started.");
}
