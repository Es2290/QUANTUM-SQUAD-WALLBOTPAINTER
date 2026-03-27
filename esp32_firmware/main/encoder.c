/**
 * @file encoder.c
 * @brief High-resolution quadrature encoder implementation (ESP-IDF / ESP32).
 *
 * Uses the ESP32 PCNT (Pulse Counter) peripheral for hardware counting.
 *
 * GPIO pin mapping:
 *   Channel 0 (FL): A=GPIO34  B=GPIO35
 *   Channel 1 (FR): A=GPIO36  B=GPIO39
 *   Channel 2 (RL): A=GPIO25  B=GPIO26
 *   Channel 3 (RR): A=GPIO27  B=GPIO14
 */

#include "encoder.h"

#include <string.h>

#include "driver/pcnt.h"
#include "esp_log.h"

#define TAG "encoder"

/* GPIO pin pairs for each encoder channel */
static const int k_pin_a[ENCODER_COUNT] = {34, 36, 25, 27};
static const int k_pin_b[ENCODER_COUNT] = {35, 39, 26, 14};

/* Accumulated 32-bit tick counters (PCNT hardware is 16-bit, so we extend) */
static volatile int32_t s_accum[ENCODER_COUNT] = {0};
static volatile int16_t s_prev_cnt[ENCODER_COUNT] = {0};

/* -------------------------------------------------------------------------
 * PCNT overflow ISR — extends hardware 16-bit counter to 32-bit
 * ---------------------------------------------------------------------- */
static void IRAM_ATTR _pcnt_isr(void *arg)
{
    uint32_t status = 0;
    pcnt_get_event_status(0, &status); /* clears all unit events */

    for (uint8_t ch = 0; ch < ENCODER_COUNT; ch++) {
        int16_t cnt = 0;
        pcnt_get_counter_value((pcnt_unit_t)ch, &cnt);
        s_accum[ch] += (int32_t)(cnt - s_prev_cnt[ch]);
        s_prev_cnt[ch] = cnt;
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

int encoder_init(void)
{
    for (uint8_t ch = 0; ch < ENCODER_COUNT; ch++) {
        pcnt_config_t cfg = {
            .pulse_gpio_num  = k_pin_a[ch],
            .ctrl_gpio_num   = k_pin_b[ch],
            .channel         = PCNT_CHANNEL_0,
            .unit            = (pcnt_unit_t)ch,
            .pos_mode        = PCNT_COUNT_INC,
            .neg_mode        = PCNT_COUNT_DEC,
            .lctrl_mode      = PCNT_MODE_REVERSE,
            .hctrl_mode      = PCNT_MODE_KEEP,
            .counter_h_lim   = 32767,
            .counter_l_lim   = -32768,
        };
        ESP_ERROR_CHECK(pcnt_unit_config(&cfg));

        /* Second channel for quadrature phase B */
        cfg.pulse_gpio_num = k_pin_b[ch];
        cfg.ctrl_gpio_num  = k_pin_a[ch];
        cfg.channel        = PCNT_CHANNEL_1;
        cfg.pos_mode       = PCNT_COUNT_DEC;
        cfg.neg_mode       = PCNT_COUNT_INC;
        ESP_ERROR_CHECK(pcnt_unit_config(&cfg));

        pcnt_counter_pause((pcnt_unit_t)ch);
        pcnt_counter_clear((pcnt_unit_t)ch);

        /* Enable overflow interrupt to extend to 32-bit */
        pcnt_event_enable((pcnt_unit_t)ch, PCNT_EVT_H_LIM);
        pcnt_event_enable((pcnt_unit_t)ch, PCNT_EVT_L_LIM);

        pcnt_counter_resume((pcnt_unit_t)ch);
    }

    pcnt_isr_register(_pcnt_isr, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT_0); /* ISR shared across all units */

    ESP_LOGI(TAG, "Encoders initialised — %d channels, %d ticks/rev.",
             ENCODER_COUNT, ENCODER_TICKS_PER_REV);
    return 0;
}

int32_t encoder_read(uint8_t channel)
{
    if (channel >= ENCODER_COUNT) {
        ESP_LOGE(TAG, "Invalid encoder channel: %d", channel);
        return 0;
    }
    int16_t hw_cnt = 0;
    pcnt_get_counter_value((pcnt_unit_t)channel, &hw_cnt);
    return s_accum[channel] + (int32_t)(hw_cnt - s_prev_cnt[channel]);
}

void encoder_read_all(int32_t ticks[ENCODER_COUNT])
{
    for (uint8_t ch = 0; ch < ENCODER_COUNT; ch++) {
        ticks[ch] = encoder_read(ch);
    }
}

void encoder_reset_all(void)
{
    for (uint8_t ch = 0; ch < ENCODER_COUNT; ch++) {
        pcnt_counter_clear((pcnt_unit_t)ch);
        s_accum[ch] = 0;
        s_prev_cnt[ch] = 0;
    }
    ESP_LOGI(TAG, "All encoder counters reset.");
}
