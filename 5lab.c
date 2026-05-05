#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_adc/adc_continuous.h"

#include "led_strip.h"
#include "led_strip_rmt.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define SAMPLE_RATE_HZ          24000
#define FFT_SIZE                4096

#define FFT_BIN_HZ              ((float)SAMPLE_RATE_HZ / (float)FFT_SIZE)
#define CALIBRATION_SECONDS     2
#define CALIBRATION_SAMPLES     (SAMPLE_RATE_HZ * CALIBRATION_SECONDS)

#define ADC_INPUT_GPIO          2
#define WS2812_GPIO             18

#define MATRIX_WIDTH            10
#define MATRIX_HEIGHT           10
#define MATRIX_LED_COUNT        (MATRIX_WIDTH * MATRIX_HEIGHT)

#define ADC_FRAME_SAMPLES       256

#define RMS_GATE                55.0f

#define BAND_SMOOTH_ALPHA       0.82f
#define BAND_FLOOR              700.0f
#define BAR_GAIN                2.2f
#define BAND_SCALE              7.0f
#define BAND_NOISE_THRESHOLD    0.08f

#define LED_MAX_R               50
#define LED_MAX_G               50

/* Чем больше значение — тем острее выделяется доминирующая частота.
   При 4.0: полоса в 50% от максимума получает лишь ~3% яркости.
   При 6.0: подавление ещё жёстче. */
#define DOMINANT_SHARPNESS      4.0f

#if CONFIG_IDF_TARGET_ESP32
#define LAB5_ADC_OUTPUT_FORMAT  ADC_DIGI_OUTPUT_FORMAT_TYPE1
/* Для ESP32 используется type1-часть union */
#define ADC_DATA_STRUCT         type1
#else
#define LAB5_ADC_OUTPUT_FORMAT  ADC_DIGI_OUTPUT_FORMAT_TYPE2
/* Для остальных (S2, S3 и т.д.) – type2 */
#define ADC_DATA_STRUCT         type2
#endif

typedef struct {
    float re;
    float im;
} complex_t;

static adc_continuous_handle_t s_adc_handle = NULL;
static led_strip_handle_t s_strip = NULL;

static adc_unit_t s_adc_unit;
static adc_channel_t s_adc_channel;

/* Буфер для сырых данных АЦП в байтах (размер фрейма) */
static uint8_t s_adc_frame_buf[ADC_FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES];

static float s_input_samples[FFT_SIZE];
static float s_window[FFT_SIZE];
static complex_t s_fft_buf[FFT_SIZE];
static float s_band_levels[MATRIX_WIDTH];

static bool s_dc_ready = false;
static uint32_t s_calib_count = 0;
static double s_calib_sum = 0.0;
static float s_dc_offset = 0.0f;

static uint32_t s_sample_pos = 0;

// Ограничивает значение диапазоном от lo до hi.
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Создаёт окно для уменьшения спектральных искажений.
static void make_hann_window(void)
{
    for (int i = 0; i < FFT_SIZE; i++) {
        s_window[i] = 0.5f * (1.0f - cosf((2.0f * (float)M_PI * i) / (FFT_SIZE - 1)));
    }
}

// Преобразует координаты матрицы x/y в индекс светодиода.
static int matrix_index(int x, int y)
{
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) {
        return -1;
    }

    int row = MATRIX_HEIGHT - 1 - y;
    return row * MATRIX_WIDTH + x;
}

// Очищает всю светодиодную матрицу.
static void clear_matrix_buffer(void)
{
    for (int i = 0; i < MATRIX_LED_COUNT; i++) {
        led_strip_set_pixel(s_strip, i, 0, 0, 0);
    }
}

// Устанавливает цвет одного пикселя матрицы.
static void set_matrix_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    int index = matrix_index(x, y);

    if (index >= 0 && index < MATRIX_LED_COUNT) {
        led_strip_set_pixel(s_strip, index, r, g, b);
    }
}

// Рисует столбчатую диаграмму спектра на матрице.
static void draw_frequency_bars(float *levels)
{
    clear_matrix_buffer();

    for (int x = 0; x < MATRIX_WIDTH; x++) {
        float ratio = clampf(levels[x] / BAND_SCALE, 0.0f, 1.0f);
        int height = (int)lroundf(ratio * MATRIX_HEIGHT);

        if (height > MATRIX_HEIGHT) {
            height = MATRIX_HEIGHT;
        }

        float freq_t = (float)x / (float)(MATRIX_WIDTH - 1); /* 0.0 = низкие, 1.0 = высокие */
        uint8_t r = (uint8_t)(LED_MAX_R * freq_t);
        uint8_t g = (uint8_t)(LED_MAX_G * (1.0f - freq_t));

        for (int y = 0; y < height; y++) {
            set_matrix_pixel(x, y, r, g, 0);
        }
    }

    led_strip_refresh(s_strip);
}

// Выполняет быстрое преобразование Фурье на месте (Cooley–Tukey, DIT).
static void fft_inplace(complex_t *a, int n)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;

        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }

        j ^= bit;

        if (i < j) {
            complex_t tmp = a[i];
            a[i] = a[j];
            a[j] = tmp;
        }
    }

    for (int len = 2; len <= n; len <<= 1) {
        float angle = -2.0f * (float)M_PI / (float)len;
        float wlen_re = cosf(angle);
        float wlen_im = sinf(angle);

        for (int i = 0; i < n; i += len) {
            float w_re = 1.0f;
            float w_im = 0.0f;

            for (int j = 0; j < len / 2; j++) {
                complex_t u = a[i + j];
                complex_t v = a[i + j + len / 2];

                float vr = v.re * w_re - v.im * w_im;
                float vi = v.re * w_im + v.im * w_re;

                a[i + j].re = u.re + vr;
                a[i + j].im = u.im + vi;

                a[i + j + len / 2].re = u.re - vr;
                a[i + j + len / 2].im = u.im - vi;

                float next_w_re = w_re * wlen_re - w_im * wlen_im;
                float next_w_im = w_re * wlen_im + w_im * wlen_re;

                w_re = next_w_re;
                w_im = next_w_im;
            }
        }
    }
}

static const float band_edges_hz[MATRIX_WIDTH + 1] = {
    80.0f,
    200.0f,
    400.0f,
    700.0f,
    1000.0f,
    1500.0f,
    2200.0f,
    3200.0f,
    4500.0f,
    6500.0f,
    9000.0f
};

// Обрабатывает накопленные семплы, считает FFT и обновляет матрицу.
static void process_fft_and_update_matrix(void)
{
    float rms_acc = 0.0f;

    for (int i = 0; i < FFT_SIZE; i++) {
        rms_acc += s_input_samples[i] * s_input_samples[i];
    }

    float rms = sqrtf(rms_acc / FFT_SIZE);

    if (rms < RMS_GATE) {
        /* Тишина — плавно гасим все полосы без выделения доминанты */
        for (int i = 0; i < MATRIX_WIDTH; i++) {
            s_band_levels[i] *= 0.85f;

            if (s_band_levels[i] < 0.02f) {
                s_band_levels[i] = 0.0f;
            }
        }

        draw_frequency_bars(s_band_levels);
        return;
    }

    for (int i = 0; i < FFT_SIZE; i++) {
        s_fft_buf[i].re = s_input_samples[i] * s_window[i];
        s_fft_buf[i].im = 0.0f;
    }

    fft_inplace(s_fft_buf, FFT_SIZE);

    const int useful_bins = FFT_SIZE / 2;

    for (int x = 0; x < MATRIX_WIDTH; x++) {
        int start_bin = (int)(band_edges_hz[x] / FFT_BIN_HZ);
        int end_bin = (int)(band_edges_hz[x + 1] / FFT_BIN_HZ);

        if (start_bin < 1) {
            start_bin = 1;
        }
        if (end_bin <= start_bin) {
            end_bin = start_bin + 1;
        }
        if (end_bin > useful_bins) {
            end_bin = useful_bins;
        }

        float sum_amp = 0.0f;
        int count = 0;

        for (int k = start_bin; k < end_bin; k++) {
            float re = s_fft_buf[k].re;
            float im = s_fft_buf[k].im;

            sum_amp += sqrtf(re * re + im * im);
            count++;
        }

        float band_amp = count > 0 ? sum_amp / (float)count : 0.0f;
        band_amp -= BAND_FLOOR;

        if (band_amp < 0.0f) {
            band_amp = 0.0f;
        }

        float log_amp = log10f(1.0f + band_amp) * BAR_GAIN;

        if (!isfinite(log_amp) || log_amp < 0.0f) {
            log_amp = 0.0f;
        }

        s_band_levels[x] = BAND_SMOOTH_ALPHA * s_band_levels[x] +
                           (1.0f - BAND_SMOOTH_ALPHA) * log_amp;

        if (s_band_levels[x] < BAND_NOISE_THRESHOLD) {
            s_band_levels[x] = 0.0f;
        }
    }

    /* --- Выделение доминирующей частоты ---
     *
     * Находим максимальный уровень среди всех полос.
     * Нормируем каждую полосу относительно максимума (0..1).
     * Применяем степенную функцию pow(norm, DOMINANT_SHARPNESS):
     *   - доминирующая полоса (norm≈1) → вес ≈ 1, яркость не меняется
     *   - соседние полосы (norm < 1)   → вес резко падает, яркость гасится
     *
     * Пример при DOMINANT_SHARPNESS=4:
     *   norm 1.0 → вес 1.000 → 100% яркости
     *   norm 0.7 → вес 0.240 → 17%  яркости
     *   norm 0.5 → вес 0.063 → 3%   яркости
     *   norm 0.3 → вес 0.008 → ~0%  яркости
     */
    float display_levels[MATRIX_WIDTH];

    float max_level = 0.0f;
    for (int x = 0; x < MATRIX_WIDTH; x++) {
        if (s_band_levels[x] > max_level) {
            max_level = s_band_levels[x];
        }
    }

    if (max_level > 0.0f) {
        for (int x = 0; x < MATRIX_WIDTH; x++) {
            float norm   = s_band_levels[x] / max_level;
            float weight = powf(norm, DOMINANT_SHARPNESS);
            display_levels[x] = s_band_levels[x] * weight;
        }
    } else {
        memcpy(display_levels, s_band_levels, sizeof(display_levels));
    }

    draw_frequency_bars(display_levels);
}

// Инициализирует светодиодную матрицу WS2812
static void init_led_matrix(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = MATRIX_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip));
    ESP_ERROR_CHECK(led_strip_clear(s_strip));
}

// Инициализирует АЦП
static void init_adc_continuous(void)
{
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(ADC_INPUT_GPIO, &s_adc_unit, &s_adc_channel));

    adc_digi_convert_mode_t conv_mode =
        (s_adc_unit == ADC_UNIT_1) ? ADC_CONV_SINGLE_UNIT_1 : ADC_CONV_SINGLE_UNIT_2;

    adc_continuous_handle_cfg_t adc_handle_config = {
        .max_store_buf_size = ADC_FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES * 8,
        .conv_frame_size = ADC_FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES,
        .flags = {
            .flush_pool = true,
        },
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_handle_config, &s_adc_handle));

    adc_digi_pattern_config_t adc_pattern[1] = {0};
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = s_adc_channel;
    adc_pattern[0].unit = s_adc_unit;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t adc_config = {
        .pattern_num = 1,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = conv_mode,
        .format = LAB5_ADC_OUTPUT_FORMAT,
    };

    ESP_ERROR_CHECK(adc_continuous_config(s_adc_handle, &adc_config));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));
}

/* Проверяет, что семпл относится к нужному каналу АЦП.
   Теперь использует новый union adc_digi_output_data_t.
   Поле valid отсутствует, поэтому проверяем только unit и channel. */
static bool adc_sample_matches(const adc_digi_output_data_t *sample)
{
    return (sample->ADC_DATA_STRUCT.unit == s_adc_unit &&
            sample->ADC_DATA_STRUCT.channel == s_adc_channel);
}

// Обрабатывает один семпл АЦП и запускает FFT после заполнения буфера.
static void handle_adc_sample(uint32_t raw)
{
    if (!s_dc_ready) {
        s_calib_sum += (double)raw;
        s_calib_count++;

        if (s_calib_count >= CALIBRATION_SAMPLES) {
            s_dc_offset = (float)(s_calib_sum / (double)s_calib_count);
            s_dc_ready = true;
            s_sample_pos = 0;
            memset(s_band_levels, 0, sizeof(s_band_levels));
        }

        return;
    }

    s_input_samples[s_sample_pos++] = (float)raw - s_dc_offset;

    if (s_sample_pos >= FFT_SIZE) {
        s_sample_pos = 0;
        process_fft_and_update_matrix();
    }
}

// Точка входа программы.
void app_main(void)
{
    make_hann_window();
    init_led_matrix();
    init_adc_continuous();

    while (1) {
        uint32_t bytes_read = 0;

        /* Новый API: adc_continuous_read возвращает сырые байты */
        esp_err_t ret = adc_continuous_read(
            s_adc_handle,
            s_adc_frame_buf,
            sizeof(s_adc_frame_buf),
            &bytes_read,
            1000
        );

        if (ret == ESP_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Количество семплов = байты / размер одного результата */
        uint32_t num_samples = bytes_read / SOC_ADC_DIGI_RESULT_BYTES;

        /* Приводим буфер к массиву adc_digi_output_data_t */
        adc_digi_output_data_t *samples = (adc_digi_output_data_t *)s_adc_frame_buf;

        for (uint32_t i = 0; i < num_samples; i++) {
            if (adc_sample_matches(&samples[i])) {
                /* Сырое 12‑битное значение находится в поле .data нужной структуры */
                uint32_t raw_adc = samples[i].ADC_DATA_STRUCT.data;
                handle_adc_sample(raw_adc);
            }
        }
    }
}