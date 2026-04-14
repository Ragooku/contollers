#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_err.h"
#include "hal/adc_types.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"

// ====================== НАСТРОЙКИ ======================

#define ADC_UNIT_SEL        ADC_UNIT_1
#define ADC_CHANNEL_SEL     ADC_CHANNEL_6
#define ADC_ATTEN_SEL       ADC_ATTEN_DB_12

#define SAMPLE_RATE_HZ      24000U
#define WINDOW_MS           5U

#define SAMPLES_PER_WINDOW  ((SAMPLE_RATE_HZ * WINDOW_MS) / 1000U)

#define CALIBRATION_MS      2000U

#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9
#define DISPLAY_WIDTH       128
#define DISPLAY_HEIGHT      64
#define HEADER_H            10

// === НОВЫЙ ПАРАМЕТР: как часто обновлять экран ===
#define DISPLAY_UPDATE_MS   50U          // ← Изменяй здесь!
// Чем больше значение — тем медленнее обновляется картинка на экране

#define BYTES_PER_SAMPLE    SOC_ADC_DIGI_DATA_BYTES_PER_CONV
#define FRAME_BYTES         (SAMPLES_PER_WINDOW * BYTES_PER_SAMPLE)
#define DMA_RING_BYTES      (FRAME_BYTES * 10U)

#define AMP_QUEUE_DEPTH     DISPLAY_WIDTH

static const char *TAG = "LAB4_MIC";

static adc_continuous_handle_t s_adc_handle = NULL;
static u8g2_t                  s_u8g2;
static QueueHandle_t           s_amp_queue  = NULL;

static int32_t  s_mean_value  = 2048;
static uint16_t s_graph[DISPLAY_WIDTH];
static uint8_t  s_graph_head  = 0;

// ====================== ФУНКЦИИ ======================

static void display_init(void)
{
    u8g2_esp32_hal_t hal = U8G2_ESP32_HAL_DEFAULT;
    hal.bus.i2c.sda = I2C_SDA_PIN;
    hal.bus.i2c.scl = I2C_SCL_PIN;
    u8g2_esp32_hal_init(hal);

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&s_u8g2, U8G2_R0,
        u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    
    u8g2_InitDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0);
    u8g2_ClearDisplay(&s_u8g2);
}

static void adc_init(void)
{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = DMA_RING_BYTES,
        .conv_frame_size    = FRAME_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &s_adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten     = ADC_ATTEN_SEL,
        .channel   = ADC_CHANNEL_SEL,
        .unit      = ADC_UNIT_SEL,
        .bit_width = ADC_BITWIDTH_12,
    };

    adc_continuous_config_t cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num    = 1,
        .adc_pattern    = &pattern,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc_handle, &cfg));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));
}

// ====================== ОТРИСОВКА ======================

static void display_calibration(uint32_t elapsed_ms)
{
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&s_u8g2, 14, 18, "Calibrating...");

    int bar_fill = (int)((elapsed_ms * 100U) / CALIBRATION_MS);
    if (bar_fill > 100) bar_fill = 100;
    u8g2_DrawFrame(&s_u8g2, 13, 26, 102, 12);
    if (bar_fill > 0) u8g2_DrawBox(&s_u8g2, 14, 27, bar_fill, 10);

    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%lu / %u ms", (unsigned long)elapsed_ms, CALIBRATION_MS);
    u8g2_DrawStr(&s_u8g2, 22, 52, tmp);
    u8g2_SendBuffer(&s_u8g2);
}

static void display_graph(uint16_t cur_amp)
{
    static const int GRAPH_H = DISPLAY_HEIGHT - HEADER_H;

    uint16_t max_val = 64;
    for (int i = 0; i < DISPLAY_WIDTH; i++) {
        if (s_graph[i] > max_val) max_val = s_graph[i];
    }

    u8g2_ClearBuffer(&s_u8g2);

    u8g2_SetFont(&s_u8g2, u8g2_font_5x7_tf);
    char hdr[32];
    snprintf(hdr, sizeof(hdr), "A:%4u  M:%4u", cur_amp, max_val);
    u8g2_DrawStr(&s_u8g2, 0, 8, hdr);

    u8g2_DrawHLine(&s_u8g2, 0, HEADER_H, DISPLAY_WIDTH);

    for (int x = 0; x < DISPLAY_WIDTH; x++) {
        int idx   = (s_graph_head + x) % DISPLAY_WIDTH;
        int bar_h = ((int)s_graph[idx] * GRAPH_H) / max_val;
        if (bar_h == 0 && s_graph[idx] > 0) bar_h = 1;

        if (bar_h > 0) {
            u8g2_DrawVLine(&s_u8g2, x, DISPLAY_HEIGHT - bar_h, bar_h);
        }
    }

    u8g2_SendBuffer(&s_u8g2);
}

// ====================== ЗАДАЧИ ======================

static void adc_task(void *pvParam)
{
    static uint8_t frame_buf[FRAME_BYTES];

    while (1) {
        uint32_t out_len = 0;
        esp_err_t ret = adc_continuous_read(s_adc_handle, frame_buf, FRAME_BYTES,
                                            &out_len, pdMS_TO_TICKS(20));

        if (ret != ESP_OK) {
            vTaskDelay(1);
            continue;
        }

        int32_t max_amp = 0;
        for (uint32_t i = 0; i + BYTES_PER_SAMPLE <= out_len; i += BYTES_PER_SAMPLE) {
            const adc_digi_output_data_t *s = (const adc_digi_output_data_t *)&frame_buf[i];
            if (s->type2.channel != ADC_CHANNEL_SEL) continue;

            int32_t amp = abs((int32_t)s->type2.data - s_mean_value);
            if (amp > max_amp) max_amp = amp;
        }

        uint16_t u_amp = (uint16_t)max_amp;

        // Если очередь полная — выкидываем старое значение
        if (uxQueueSpacesAvailable(s_amp_queue) == 0) {
            uint16_t dummy;
            xQueueReceive(s_amp_queue, &dummy, 0);
        }
        xQueueSend(s_amp_queue, &u_amp, 0);

        vTaskDelay(1);   // небольшая пауза, чтобы не грузить CPU на 100%
    }
}

static void display_task(void *pvParam)
{
    uint16_t amp = 0;
    TickType_t last_update = xTaskGetTickCount();

    while (1) {
        // Ждём новое значение амплитуды
        if (xQueueReceive(s_amp_queue, &amp, portMAX_DELAY) == pdTRUE) {
            // Обновляем график в любом случае (чтобы не терять данные)
            s_graph[s_graph_head] = amp;
            s_graph_head = (uint8_t)((s_graph_head + 1U) % DISPLAY_WIDTH);

            // Но рисуем на экран только с заданной частотой
            if (xTaskGetTickCount() - last_update >= pdMS_TO_TICKS(DISPLAY_UPDATE_MS)) {
                display_graph(amp);
                last_update = xTaskGetTickCount();
            }
        }
    }
}

// ====================== MAIN ======================

void app_main(void)
{
    display_init();
    adc_init();
    memset(s_graph, 0, sizeof(s_graph));

    // Калибровка (оставлена без изменений)
    ESP_LOGI(TAG, "Начало калибровки (%u мс)...", CALIBRATION_MS);

    static uint8_t cal_buf[FRAME_BYTES];
    int64_t sum = 0;
    int64_t count = 0;

    const TickType_t t_start = xTaskGetTickCount();
    const TickType_t t_cal = pdMS_TO_TICKS(CALIBRATION_MS);

    while ((xTaskGetTickCount() - t_start) < t_cal) {
        uint32_t elapsed = (uint32_t)((xTaskGetTickCount() - t_start) * portTICK_PERIOD_MS);
        display_calibration(elapsed);

        uint32_t out_len = 0;
        adc_continuous_read(s_adc_handle, cal_buf, FRAME_BYTES, &out_len, pdMS_TO_TICKS(10));

        for (uint32_t i = 0; i + BYTES_PER_SAMPLE <= out_len; i += BYTES_PER_SAMPLE) {
            const adc_digi_output_data_t *s = (const adc_digi_output_data_t *)&cal_buf[i];
            if (s->type2.channel != ADC_CHANNEL_SEL) continue;
            sum += s->type2.data;
            count++;
        }
    }

    if (count > 0) s_mean_value = (int32_t)(sum / count);

    ESP_LOGI(TAG, "Калибровка завершена. Среднее = %ld", (long)s_mean_value);

    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&s_u8g2, 10, 25, "Calibration OK");
    char mean_str[24];
    snprintf(mean_str, sizeof(mean_str), "Mean = %ld", (long)s_mean_value);
    u8g2_DrawStr(&s_u8g2, 20, 42, mean_str);
    u8g2_SendBuffer(&s_u8g2);
    vTaskDelay(pdMS_TO_TICKS(800));

    // Создаём очередь и задачи
    s_amp_queue = xQueueCreate(AMP_QUEUE_DEPTH, sizeof(uint16_t));

    xTaskCreatePinnedToCore(adc_task,    "adc_task",  4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(display_task,"disp_task", 4096, NULL, 4, NULL, 1);
}
