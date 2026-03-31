#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "u8g2_esp32_hal.h"

// Пины I2C
#define PIN_SDA GPIO_NUM_8
#define PIN_SCL GPIO_NUM_9
#define I2C_PORT I2C_NUM_0

// Адреса устройств
#define BME280_ADDR 0x76
#define SSD1306_ADDR 0x3C

// Регистры BME280
#define BME280_REG_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_TEMP_MSB 0xFA

static const char *TAG = "BME280_SSD1306";

// Структура для калибровочных данных BME280
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_data_t;

bme280_calib_data_t calib;

// Инициализация I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    
    esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %d", ret);
        return ret;
    }
    
    ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %d", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

// Проверка наличия устройства на I2C шине
static esp_err_t i2c_probe_device(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Чтение данных по I2C
static esp_err_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, dev_addr, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
}

// Запись данных по I2C
static esp_err_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_PORT, dev_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

// Чтение калибровочных данных BME280
static esp_err_t bme280_read_calibration(void) {
    uint8_t buf[32];
    
    ESP_LOGI(TAG, "Reading calibration data...");
    
    // Чтение калибровки температуры и давления (0x88 - 0xA1)
    esp_err_t ret = i2c_read(BME280_ADDR, 0x88, buf, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature/pressure calibration");
        return ret;
    }
    
    // Выводим сырые данные для отладки
    ESP_LOGI(TAG, "Raw calibration data (0x88-0x9F):");
    for (int i = 0; i < 24; i += 8) {
        ESP_LOGI(TAG, "  %02x %02x %02x %02x %02x %02x %02x %02x", 
                 buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
    }
    
    // Правильное чтение калибровочных данных (little-endian)
    calib.dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
    calib.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
    calib.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);
    
    calib.dig_P1 = (uint16_t)((buf[7] << 8) | buf[6]);
    calib.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
    calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);
    
    // Чтение калибровки влажности H1
    ret = i2c_read(BME280_ADDR, 0xA1, &calib.dig_H1, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read humidity calibration H1");
        return ret;
    }
    
    // Чтение остальной калибровки влажности (0xE1 - 0xE7)
    ret = i2c_read(BME280_ADDR, 0xE1, buf, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read humidity calibration H2-H6");
        return ret;
    }
    
    ESP_LOGI(TAG, "Raw humidity calibration (0xE1-0xE7): %02x %02x %02x %02x %02x %02x %02x",
             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
    
    calib.dig_H2 = (int16_t)((buf[1] << 8) | buf[0]);
    calib.dig_H3 = buf[2];
    calib.dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
    calib.dig_H5 = (int16_t)((buf[5] << 4) | ((buf[4] >> 4) & 0x0F));
    calib.dig_H6 = (int8_t)buf[6];
    
    // Выводим считанные калибровочные данные
    ESP_LOGI(TAG, "Calibration data:");
    ESP_LOGI(TAG, "  T1=%u (0x%04X), T2=%d (0x%04X), T3=%d (0x%04X)", 
             calib.dig_T1, calib.dig_T1, calib.dig_T2, (uint16_t)calib.dig_T2, calib.dig_T3, (uint16_t)calib.dig_T3);
    ESP_LOGI(TAG, "  P1=%u (0x%04X), P2=%d, P3=%d, P4=%d, P5=%d, P6=%d, P7=%d, P8=%d, P9=%d", 
             calib.dig_P1, calib.dig_P1, calib.dig_P2, calib.dig_P3, calib.dig_P4, calib.dig_P5, 
             calib.dig_P6, calib.dig_P7, calib.dig_P8, calib.dig_P9);
    
    return ESP_OK;
}

// Инициализация BME280
static esp_err_t bme280_init(void) {
    uint8_t chip_id = 0;
    esp_err_t ret = i2c_read(BME280_ADDR, BME280_REG_ID, &chip_id, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }
    
    ESP_LOGI(TAG, "BME280 chip ID: 0x%02X (expected 0x60)", chip_id);
    
    if (chip_id != 0x60) {
        ESP_LOGE(TAG, "Invalid BME280 chip ID!");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Сброс датчика
    ESP_LOGI(TAG, "Resetting BME280...");
    ret = i2c_write(BME280_ADDR, BME280_REG_RESET, 0xB6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset sensor");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Чтение калибровочных данных
    ret = bme280_read_calibration();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Настройка режима работы
    ESP_LOGI(TAG, "Configuring BME280...");
    
    // Устанавливаем oversampling
    ret = i2c_write(BME280_ADDR, BME280_REG_CTRL_HUM, 0x01);  // Humidity oversampling x1
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set humidity config");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ret = i2c_write(BME280_ADDR, BME280_REG_CTRL_MEAS, 0x27); // Temp/Press oversampling x1, normal mode
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set measurement config");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ret = i2c_write(BME280_ADDR, BME280_REG_CONFIG, 0xA0);    // Standby 1000ms, filter off
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "BME280 initialized successfully");
    return ESP_OK;
}

// Компенсация температуры
static int32_t bme280_compensate_temperature(int32_t adc_T, int32_t *t_fine) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    
    *t_fine = var1 + var2;
    T = (*t_fine * 5 + 128) >> 8;
    
    return T;
}

// Компенсация давления (по спецификации BME280)
static uint32_t bme280_compensate_pressure(int32_t adc_P, int32_t t_fine) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0;
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    
    return (uint32_t)p;
}

// Чтение данных с BME280
static esp_err_t bme280_read_data(float *temperature, float *pressure, float *humidity) {
    uint8_t data[8];
    int32_t adc_T, adc_P;
    int32_t t_fine;
    
    // Чтение данных давления и температуры (0xF7 - 0xFE)
    esp_err_t ret = i2c_read(BME280_ADDR, 0xF7, data, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }
    
    // Выводим сырые данные для отладки
    ESP_LOGD(TAG, "Raw data: %02x %02x %02x %02x %02x %02x %02x %02x",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    
    // Формируем 20-битные значения
    adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));
    adc_T = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4));
    
    ESP_LOGD(TAG, "ADC: T=%ld, P=%ld", adc_T, adc_P);
    
    // Компенсация температуры
    int32_t temp_raw = bme280_compensate_temperature(adc_T, &t_fine);
    *temperature = temp_raw / 100.0f;
    
    // Компенсация давления
    uint32_t press_raw = bme280_compensate_pressure(adc_P, t_fine);
    *pressure = press_raw / 25600.0f;  // Pa to hPa
    
    ESP_LOGD(TAG, "Compensated: T=%ld, P=%lu", temp_raw, press_raw);
    
    return ESP_OK;
}

// U8g2 объект
static u8g2_t u8g2;

// Кастомная функция для I2C с использованием существующего драйвера
static uint8_t u8g2_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[32];
    static uint8_t buf_idx = 0;
    
    switch(msg) {
        case U8X8_MSG_BYTE_INIT:
            break;
            
        case U8X8_MSG_BYTE_SET_DC:
            break;
            
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
            
        case U8X8_MSG_BYTE_SEND:
            for(int i = 0; i < arg_int; i++) {
                buffer[buf_idx++] = ((uint8_t*)arg_ptr)[i];
            }
            break;
            
        case U8X8_MSG_BYTE_END_TRANSFER:
            if (buf_idx > 0) {
                uint8_t addr = u8x8_GetI2CAddress(u8x8);
                
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, addr, true);
                i2c_master_write(cmd, buffer, buf_idx, true);
                i2c_master_stop(cmd);
                
                esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
                i2c_cmd_link_delete(cmd);
                
                if (ret != ESP_OK) {
                    return 0;
                }
            }
            break;
            
        default:
            return 0;
    }
    return 1;
}

// Инициализация U8g2 для SSD1306
static void u8g2_init(void) {
    // Проверяем наличие дисплея
    esp_err_t ret = i2c_probe_device(SSD1306_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 not found at address 0x%02X", SSD1306_ADDR);
        return;
    }
    ESP_LOGI(TAG, "SSD1306 found at address 0x%02X", SSD1306_ADDR);
    
    // Инициализация u8g2 с нашей кастомной функцией
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    
    // Устанавливаем адрес I2C (8-битный адрес для записи)
    u8x8_SetI2CAddress(&u8g2.u8x8, SSD1306_ADDR << 1);
    
    // Инициализация дисплея
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "SSD1306 initialized successfully");
}

// Отображение температуры
static void display_temperature(float temp) {
    char temp_str[16];
    snprintf(temp_str, sizeof(temp_str), "%.1f", temp);
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_logisoso32_tf);
    
    int16_t width = u8g2_GetStrWidth(&u8g2, temp_str);
    int16_t x = (128 - width) / 2;
    int16_t y = 50;
    
    u8g2_DrawStr(&u8g2, x, y, temp_str);
    u8g2_SendBuffer(&u8g2);
}

// Отображение давления
static void display_pressure(float press) {
    char press_str[16];
    snprintf(press_str, sizeof(press_str), "%.0f", press);
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_logisoso32_tf);
    
    int16_t width = u8g2_GetStrWidth(&u8g2, press_str);
    int16_t x = (128 - width) / 2;
    int16_t y = 50;
    
    u8g2_DrawStr(&u8g2, x, y, press_str);
    u8g2_SendBuffer(&u8g2);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting BME280 + SSD1306 on ESP32-S3");
    
    // Инициализация I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Поиск устройств на I2C шине
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_probe_device(addr) == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
        }
    }
    
    // Инициализация BME280
    ret = bme280_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME280 initialization failed!");
    }
    
    // Инициализация дисплея
    u8g2_init();
    
    float temperature = 0.0f, pressure = 0.0f, humidity = 0.0f;
    int show_temp = 1;
    
    while (1) {
        // Чтение данных с датчика
        ret = bme280_read_data(&temperature, &pressure, &humidity);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %.2f hPa", temperature, pressure);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }
        
        // Отображение на дисплее
        if (show_temp) {
            display_temperature(temperature);
        } else {
            display_pressure(pressure);
        }
        
        show_temp = !show_temp;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}