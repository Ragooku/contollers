#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_chip_info.h"
#include "esp_pm.h"
#include "driver/uart.h"

#define UART_PORT_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define BUF_SIZE 256

static uint32_t interval_ms = 1000;           // начальный интервал 1000 мс
static TickType_t last_print_tick = 0;
static TickType_t start_tick = 0;

// Получение строки с названием модели чипа
static const char* get_chip_model_str(esp_chip_model_t model)
{
    switch (model)
    {
        case CHIP_ESP32:   return "ESP32";
        case CHIP_ESP32S2: return "ESP32-S2";
        case CHIP_ESP32S3: return "ESP32-S3";
        case CHIP_ESP32C2: return "ESP32-C2";
        case CHIP_ESP32C3: return "ESP32-C3";
        case CHIP_ESP32C6: return "ESP32-C6";
        case CHIP_ESP32H2: return "ESP32-H2";
        default:           return "Unknown";
    }
}

// Вывод поддерживаемых беспроводных интерфейсов
static void print_wireless_features(uint32_t features)
{
    printf("Беспроводные интерфейсы:\n");
    if (features & CHIP_FEATURE_WIFI_BGN) printf(" - Wi-Fi 2.4 GHz (802.11b/g/n)\n");
    if (features & CHIP_FEATURE_BLE)      printf(" - Bluetooth LE (BLE)\n");
    if (features & CHIP_FEATURE_BT)       printf(" - Bluetooth Classic\n");
    if (features & CHIP_FEATURE_IEEE802154) printf(" - IEEE 802.15.4 (Thread/Zigbee)\n");
    if (!(features & (CHIP_FEATURE_WIFI_BGN | CHIP_FEATURE_BLE | CHIP_FEATURE_BT | CHIP_FEATURE_IEEE802154)))
        printf(" - Нет беспроводных интерфейсов (или не определено)\n");
}

// Инициализация UART
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Вывод информации о чипе (один раз)
static void print_chip_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    esp_pm_config_t pm_config = {0};
    esp_pm_get_configuration(&pm_config);
    uint32_t cpu_freq_mhz = pm_config.max_freq_mhz;
    if (cpu_freq_mhz == 0) cpu_freq_mhz = 240;

    printf("\n=== Информация о микроконтроллере ===\n");
    printf("Модель чипа:           %s\n", get_chip_model_str(chip_info.model));
    printf("Ревизия чипа:          v%d.%d\n", chip_info.revision / 100, chip_info.revision % 100);
    printf("Частота CPU (max):     %lu MHz\n", cpu_freq_mhz);

    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK)
    {
        printf("MAC-адрес (WiFi STA):  %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else
    {
        printf("MAC-адрес:             (не удалось прочитать)\n");
    }

    print_wireless_features(chip_info.features);

    printf("\nТекущий интервал вывода: %lu мс\n", interval_ms);
    printf("Введите новое значение в миллисекундах и нажмите ENTER (≥ 100 мс)\n\n");
}

// Периодический вывод времени работы
static void print_uptime(void)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t uptime_ms = (now - start_tick) * portTICK_PERIOD_MS;
    uint32_t sec = uptime_ms / 1000;
    uint32_t ms  = uptime_ms % 1000;

    printf("Время работы: %lu сек %03lu мс | интервал: %lu мс\n",
           sec, ms, interval_ms);
}

// Главная функция
void app_main(void)
{
    uart_init();

    // Даём время открыть терминал
    vTaskDelay(pdMS_TO_TICKS(1500));

    print_chip_info();

    start_tick = xTaskGetTickCount();
    last_print_tick = start_tick;

    char rx_buf[32];
    int rx_idx = 0;

    while (1)
    {
        TickType_t now = xTaskGetTickCount();

        // Периодический вывод
        if ((now - last_print_tick) * portTICK_PERIOD_MS >= interval_ms)
        {
            last_print_tick = now;
            print_uptime();
        }

        // Чтение символов из UART
        uint8_t byte;
        if (uart_read_bytes(UART_PORT_NUM, &byte, 1, pdMS_TO_TICKS(10)) == 1)
        {
            // ← Эхо: отправляем каждый полученный символ обратно в терминал
            uart_write_bytes(UART_PORT_NUM, &byte, 1);

            if (byte == '\n' || byte == '\r')
            {
                if (rx_idx > 0)
                {
                    rx_buf[rx_idx] = '\0';
                    uint32_t new_val = 0;
                    if (sscanf(rx_buf, "%lu", &new_val) == 1)
                    {
                        if (new_val >= 100)
                        {
                            interval_ms = new_val;
                            printf("→ Интервал изменён на %lu мс\n\n", interval_ms);
                        }
                        else
                        {
                            printf("! Минимальное значение — 100 мс\n\n");
                        }
                    }
                    else
                    {
                        printf("! Введите корректное число\n\n");
                    }
                    rx_idx = 0;
                }
            }
            else if (rx_idx < sizeof(rx_buf) - 1)
            {
                // Принимаем только цифры
                if (byte >= '0' && byte <= '9')
                {
                    rx_buf[rx_idx++] = (char)byte;
                }
                // Остальные символы игнорируем (можно добавить поддержку Backspace позже)
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}