/*
 *      Monitor de qualidade de tensão - ESP32
 *      HACKATOHM - 2026
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_idf_version.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* ============================ CONFIGURAÇÕES ============================ */

/* Rede Wi-Fi e broker MQTT: altere antes de compilar. */
#define WIFI_SSID               "YOUR_WIFI_SSID"
#define WIFI_PASSWORD           "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_URI         "mqtt://192.168.1.100:1883"
#define MQTT_TOPIC_EVENTS       "power-monitor/events"

/* Conversão do sinal condicionado para valores da rede. */
#define GRID_NOMINAL_VRMS       127.0f
#define VOLTAGE_DIVIDER_RATIO   146.05f
#define ADC_DC_OFFSET_V         1.15f

/* Limites operacionais esperados; não limitam nem protegem fisicamente o ADC. */
#define PU_MIN_EXPECTED         0.85f
#define PU_MAX_EXPECTED         1.15f

/* Limiares da FSM com histerese para evitar comutação rápida perto do limite. */
#define PU_SAG_ENTER            0.90f
#define PU_SAG_EXIT             0.92f
#define PU_SWELL_ENTER          1.10f
#define PU_SWELL_EXIT           1.08f

/* ESP32 clássico: GPIO34 = ADC1_CHANNEL_6; ADC1 pode ser usado com Wi-Fi. */
#define ADC_UNIT_USED           ADC_UNIT_1
#define ADC_CHANNEL_USED        ADC_CHANNEL_6
#define ADC_GPIO_USED           GPIO_NUM_34
#define ADC_INPUT_ATTENUATION   ADC_ATTEN_DB_12

/* 
 * 1 MHz de resolução e alarme a cada 260 us -> cerca de 3,846 kS/s.
 * 64 amostras cobrem aproximadamente 16,64 ms, próximo de um ciclo de 60 Hz. 
 *
 */
#define TIMER_RESOLUTION_HZ     1000000ULL
#define SAMPLE_PERIOD_US        260ULL
#define SAMPLES_PER_WINDOW      64U
#define SAMPLE_SEMAPHORE_DEPTH  256U

/* LEDs externos, cada um com resistor em série apropriado. */
#define LED_NORMAL_GPIO         GPIO_NUM_27
#define LED_SAG_GPIO            GPIO_NUM_25
#define LED_SWELL_GPIO          GPIO_NUM_26

/* LCD 16x2 com backpack PCF8574 (mapa comum: P0=RS, P1=RW, P2=EN, P3=BL). */
#define I2C_PORT_USED           I2C_NUM_0
#define I2C_SDA_GPIO            GPIO_NUM_21
#define I2C_SCL_GPIO            GPIO_NUM_22
#define I2C_FREQUENCY_HZ        100000
#define LCD_I2C_ADDRESS         0x27

/* Cartão SD em modo SPI. */
#define SD_MOSI_GPIO            GPIO_NUM_23
#define SD_MISO_GPIO            GPIO_NUM_19
#define SD_SCLK_GPIO            GPIO_NUM_18
#define SD_CS_GPIO              GPIO_NUM_13
#define SD_MOUNT_POINT          "/sdcard"
#define SD_LOG_FILE             SD_MOUNT_POINT "/events.csv"

#define RMS_QUEUE_LENGTH        4U
#define EVENT_QUEUE_LENGTH      32U
#define LCD_NOTICE_QUEUE_LENGTH 8U
#define WIFI_CONNECTED_BIT      BIT0
#define MQTT_CONNECTED_BIT      BIT1

static const char *TAG = "power_monitor";

/* ================================ TIPOS ================================ */

typedef enum 
{
    STATE_NORMAL = 0,
    STATE_SAG,
    STATE_SWELL
} detector_state_t;

typedef struct 
{
    int64_t window_end_us;          /* Tempo monotônico desde a inicialização. */
    float mean_adc_v;               /* Média do pino ADC; inclui offset DC. */
    float adc_rms_ac_v;             /* RMS do componente AC, após remover a média. */
    float grid_rms_v;               /* RMS calculado na escala da rede elétrica. */
    float pu;                       /* grid_rms_v / GRID_NOMINAL_VRMS. */
    int adc_raw_last;
} measurement_t;

typedef struct 
{
    detector_state_t type;
    int64_t start_monotonic_us;
    int64_t end_monotonic_us;
    int64_t duration_ms;
    int64_t start_epoch_s;          /* 0 quando o relógio ainda não sincronizou. */
    int64_t end_epoch_s;
    float min_pu;
    float max_pu;
    float min_grid_rms_v;
    float max_grid_rms_v;
} power_event_t;

typedef struct 
{
    measurement_t measurement;
    detector_state_t state;
} system_view_t;

typedef struct 
{
    detector_state_t type;
    bool is_start;
    int64_t duration_ms;
    float min_pu;
    float max_pu;
} lcd_notice_t;

/* ============================ RECURSOS RTOS ============================ */

static adc_oneshot_unit_handle_t    adc_handle;
static adc_cali_handle_t            adc_cali_handle;
static gptimer_handle_t             sample_timer;
static SemaphoreHandle_t            sample_semaphore;
static QueueHandle_t                measurement_queue;
static QueueHandle_t                sd_event_queue;
static QueueHandle_t                mqtt_event_queue;
static QueueHandle_t                lcd_notice_queue;
static SemaphoreHandle_t            view_mutex;
static EventGroupHandle_t           network_event_group;

static system_view_t                latest_view = { .state = STATE_NORMAL };
static volatile uint32_t            timer_semaphore_overflows;
static bool sntp_started;
static esp_mqtt_client_handle_t     mqtt_client;
static i2c_master_bus_handle_t      lcd_i2c_bus;
static i2c_master_dev_handle_t      lcd_i2c_device;

/* ============================== UTILITÁRIOS ============================ */

static const char *state_name(detector_state_t state)
{
    switch(state) 
    {
        case STATE_SAG:   return "SAG";
        case STATE_SWELL: return "SWELL";
        default:          return "NORMAL";
    }
}

static int64_t get_epoch_seconds_or_zero(void)
{
    time_t now = 0;
    time(&now);

    /* Evita gravar como data válida um relógio que ainda esteja em 1970. */
    return (now >= 1700000000) ? (int64_t)now : 0;
}

static void update_system_view(const measurement_t *measurement, detector_state_t state)
{
    if(xSemaphoreTake(view_mutex, pdMS_TO_TICKS(20)) == pdTRUE) 
    {
        latest_view.measurement = *measurement;
        latest_view.state = state;
        xSemaphoreGive(view_mutex);
    }
}

static bool read_system_view(system_view_t *destination)
{
    if(xSemaphoreTake(view_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
   
    *destination = latest_view;
    xSemaphoreGive(view_mutex);

    return true;
}

static void set_state_leds(detector_state_t state)
{
    gpio_set_level(LED_NORMAL_GPIO, state == STATE_NORMAL);
    gpio_set_level(LED_SAG_GPIO, state == STATE_SAG);
    gpio_set_level(LED_SWELL_GPIO, state == STATE_SWELL);
}

static void send_lcd_notice(const lcd_notice_t *notice)
{
    if(xQueueSend(lcd_notice_queue, notice, 0) != pdTRUE) 
    {
        ESP_LOGW(TAG, "Fila de avisos do LCD cheia; aviso descartado");
    }
}

/* ================================ ADC ================================== */

static void initialize_adc(void)
{
    const adc_oneshot_unit_init_cfg_t unit_config = 
    {
        .unit_id = ADC_UNIT_USED,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

    const adc_oneshot_chan_cfg_t channel_config = 
    {
        .atten = ADC_INPUT_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_USED, &channel_config));

    const adc_cali_line_fitting_config_t calibration_config = 
    {
        .unit_id = ADC_UNIT_USED,
        .atten = ADC_INPUT_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    if(adc_cali_create_scheme_line_fitting(&calibration_config, &adc_cali_handle) == ESP_OK) 
    {
        ESP_LOGI(TAG, "Calibração ADC por line fitting habilitada");
    } 

    else 
    {
        adc_cali_handle = NULL;
        ESP_LOGW(TAG, "Sem calibração ADC; será usada aproximação por escala");
    }


    ESP_LOGI(TAG, "ADC configurado: GPIO%d, ADC1, canal 6", ADC_GPIO_USED);
}

static esp_err_t read_adc_voltage(int *raw_value, float *voltage_v)
{
    int raw = 0;

    esp_err_t error = adc_oneshot_read(adc_handle, ADC_CHANNEL_USED, &raw);

    if(error != ESP_OK) return error;
    
    int voltage_mv = 0;

    if(adc_cali_handle != NULL && adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv) == ESP_OK) 
    {
        *voltage_v = (float)voltage_mv / 1000.0f;
    } 

    else 
    {
        *voltage_v = ((float)raw / 4095.0f) * 3.3f;
    }

    *raw_value = raw;

    return ESP_OK;
}

static bool IRAM_ATTR sample_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_context)
{
    (void)timer;
    (void)event_data;
    (void)user_context;

    BaseType_t high_priority_task_woken = pdFALSE;

    if(sample_semaphore != NULL) 
    {
        if(xSemaphoreGiveFromISR(sample_semaphore, &high_priority_task_woken) != pdTRUE) 
        {
            timer_semaphore_overflows++;
        }
    }

    return high_priority_task_woken == pdTRUE;
}

static void initialize_sample_timer(void)
{
    const gptimer_config_t timer_config = 
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &sample_timer));

    const gptimer_event_callbacks_t callbacks = 
    {
        .on_alarm = sample_timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(sample_timer, &callbacks, NULL));

    const gptimer_alarm_config_t alarm_config = 
    {
        .alarm_count = SAMPLE_PERIOD_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(sample_timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(sample_timer));
}

static void adc_task(void *argument)
{
    (void)argument;

    float samples[SAMPLES_PER_WINDOW];
    uint32_t sample_index = 0;
    uint32_t window_counter = 0;
    int last_raw = 0;

    ESP_LOGI(TAG, "adc_task iniciada: %u amostras/janela, período do timer %llu us", (unsigned)SAMPLES_PER_WINDOW, (unsigned long long)SAMPLE_PERIOD_US);

    while(true) 
    {
        if(xSemaphoreTake(sample_semaphore, portMAX_DELAY) != pdTRUE) continue;

        float voltage = 0.0f;

        if(read_adc_voltage(&last_raw, &voltage) != ESP_OK) 
        {
            ESP_LOGW(TAG, "Falha na leitura do ADC; amostra ignorada");
            continue;
        }

        samples[sample_index++] = voltage;
        if(sample_index < SAMPLES_PER_WINDOW) continue;
      
        double sum = 0.0;
        for(uint32_t i = 0; i < SAMPLES_PER_WINDOW; i++) 
        {
            sum += samples[i];
        }

        const double mean = sum / (double)SAMPLES_PER_WINDOW;

        double square_sum = 0.0;

        for(uint32_t i = 0; i < SAMPLES_PER_WINDOW; i++) 
        {
            const double centered = (double)samples[i] - mean;
            square_sum += centered * centered;
        }

        const float adc_rms_ac_v = (float)sqrt(square_sum / (double)SAMPLES_PER_WINDOW);
        const float grid_rms_v = adc_rms_ac_v * VOLTAGE_DIVIDER_RATIO;

        measurement_t measurement = 
        {
            .window_end_us = esp_timer_get_time(),
            .mean_adc_v = (float)mean,
            .adc_rms_ac_v = adc_rms_ac_v,
            .grid_rms_v = grid_rms_v,
            .pu = grid_rms_v / GRID_NOMINAL_VRMS,
            .adc_raw_last = last_raw,
        };

        if(xQueueSend(measurement_queue, &measurement, 0) != pdTRUE) 
        {
            measurement_t discarded;
            (void)xQueueReceive(measurement_queue, &discarded, 0);
            (void)xQueueSend(measurement_queue, &measurement, 0);
        }

        sample_index = 0;
        window_counter++;

        if((window_counter % 64U) == 0U) 
        {
            if(fabsf(measurement.mean_adc_v - ADC_DC_OFFSET_V) > 0.20f) 
            {
                ESP_LOGW(TAG, "Offset médio fora da referência: medido=%.3f V, esperado=%.3f V", measurement.mean_adc_v, ADC_DC_OFFSET_V);
            }

            if(measurement.pu < PU_MIN_EXPECTED || measurement.pu > PU_MAX_EXPECTED) 
            {
                ESP_LOGW(TAG, "p.u. fora da faixa operacional declarada [%.2f, %.2f]: %.4f", PU_MIN_EXPECTED, PU_MAX_EXPECTED, measurement.pu);
            }
        }

        if(timer_semaphore_overflows > 0) 
        {
            ESP_LOGW(TAG, "Backlog de amostragem observado: contador=%lu", (unsigned long)timer_semaphore_overflows);
            timer_semaphore_overflows = 0;
        }
    }
}

/* ============================= MÁQUINA DE ESTADOS ====================== */

static detector_state_t get_next_state(detector_state_t current, float pu)
{
    switch(current) 
    {
        case STATE_NORMAL:
            if(pu < PU_SAG_ENTER)      return STATE_SAG;
            if(pu > PU_SWELL_ENTER)    return STATE_SWELL;
            return STATE_NORMAL;

        case STATE_SAG:
            if(pu > PU_SWELL_ENTER)    return STATE_SWELL;
            if(pu >= PU_SAG_EXIT)      return STATE_NORMAL;
            return STATE_SAG;

        case STATE_SWELL:
            if(pu < PU_SAG_ENTER)      return STATE_SAG;
            if(pu <= PU_SWELL_EXIT)    return STATE_NORMAL;
            return STATE_SWELL;

        default:                        return STATE_NORMAL;
    }
}

static void begin_event(power_event_t *event, detector_state_t type, const measurement_t *measurement)
{
    memset(event, 0, sizeof(*event));
    event->type = type;
    event->start_monotonic_us = esp_timer_get_time();
    event->start_epoch_s = get_epoch_seconds_or_zero();
    event->min_pu = measurement->pu;
    event->max_pu = measurement->pu;
    event->min_grid_rms_v = measurement->grid_rms_v;
    event->max_grid_rms_v = measurement->grid_rms_v;

    set_state_leds(type);
    lcd_notice_t notice = { .type = type, .is_start = true };
    send_lcd_notice(&notice);
    ESP_LOGW(TAG, "EVENT_START,type=%s,pu=%.4f,Vrms=%.2f", state_name(type), measurement->pu, measurement->grid_rms_v);
}

static void update_event_peaks(power_event_t *event, const measurement_t *measurement)
{
    if(measurement->pu < event->min_pu)                 event->min_pu = measurement->pu;
    if(measurement->pu > event->max_pu)                 event->max_pu = measurement->pu;
    if(measurement->grid_rms_v < event->min_grid_rms_v) event->min_grid_rms_v = measurement->grid_rms_v;
    if(measurement->grid_rms_v > event->max_grid_rms_v) event->max_grid_rms_v = measurement->grid_rms_v;
}

static void finish_event(power_event_t *event)
{
    event->end_monotonic_us = esp_timer_get_time();
    event->end_epoch_s = get_epoch_seconds_or_zero();
    event->duration_ms = (event->end_monotonic_us - event->start_monotonic_us) / 1000;

    if(xQueueSend(sd_event_queue, event, 0) != pdTRUE) 
    {
        ESP_LOGE(TAG, "Fila SD cheia: evento %s não enfileirado", state_name(event->type));
    }

    if(xQueueSend(mqtt_event_queue, event, 0) != pdTRUE) 
    {
        ESP_LOGE(TAG, "Fila MQTT cheia: evento %s não enfileirado", state_name(event->type));
    }

    lcd_notice_t notice = 
    {
        .type = event->type,
        .is_start = false,
        .duration_ms = event->duration_ms,
        .min_pu = event->min_pu,
        .max_pu = event->max_pu,
    };
    send_lcd_notice(&notice);

    ESP_LOGW(TAG, "EVENT_END,type=%s,duration_ms=%lld,min_pu=%.4f,max_pu=%.4f", state_name(event->type), (long long)event->duration_ms, event->min_pu, event->max_pu);
}

static void fsm_task(void *argument)
{
    (void)argument;
    detector_state_t current_state = STATE_NORMAL;
    power_event_t active_event = {0};
    bool event_active = false;

    ESP_LOGI(TAG, "fsm_task iniciada; thresholds SAG<%.2f / SWELL>%.2f pu", PU_SAG_ENTER, PU_SWELL_ENTER);
    set_state_leds(current_state);

    while(true) 
    {
        measurement_t measurement;

        if(xQueueReceive(measurement_queue, &measurement, portMAX_DELAY) != pdTRUE) continue;

        detector_state_t next_state = get_next_state(current_state, measurement.pu);

        if(event_active) 
        {
            update_event_peaks(&active_event, &measurement);
        }

        if(next_state != current_state) 
        {
            if(event_active) 
            {
                finish_event(&active_event);
                event_active = false;
            }

            current_state = next_state;
            set_state_leds(current_state);

            if(current_state != STATE_NORMAL) 
            {
                begin_event(&active_event, current_state, &measurement);
                event_active = true;
            } 

            else 
            {
                ESP_LOGI(TAG, "STATE_CHANGE,NORMAL,pu=%.4f,Vrms=%.2f",
                         measurement.pu, measurement.grid_rms_v);
            }
        }

        update_system_view(&measurement, current_state);

        ESP_LOGD(TAG, "Vrms=%.2f V, pu=%.4f, mean_adc=%.3f V, raw=%d", measurement.grid_rms_v, measurement.pu, measurement.mean_adc_v, measurement.adc_raw_last);
    }
}

/* =============================== LCD I2C ================================ */

#define LCD_RS_BIT       0x01
#define LCD_RW_BIT       0x02
#define LCD_EN_BIT       0x04
#define LCD_BACKLIGHT    0x08

static esp_err_t lcd_write_expander(uint8_t value)
{
    return i2c_master_transmit(lcd_i2c_device, &value, 1, 100);
}

static void lcd_write_nibble(uint8_t nibble, bool is_data)
{
    uint8_t output = (nibble & 0xF0) | LCD_BACKLIGHT | (is_data ? LCD_RS_BIT : 0);
    (void)lcd_write_expander(output | LCD_EN_BIT);
    esp_rom_delay_us(1);
    (void)lcd_write_expander(output & (uint8_t)~LCD_EN_BIT);
    esp_rom_delay_us(50);
}

static void lcd_send_byte(uint8_t value, bool is_data)
{
    lcd_write_nibble(value & 0xF0, is_data);
    lcd_write_nibble((uint8_t)(value << 4), is_data);
}

static void lcd_command(uint8_t command)
{
    lcd_send_byte(command, false);
    if(command == 0x01 || command == 0x02) vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_init(void)
{
    const i2c_master_bus_config_t bus_config = 
    {
        .i2c_port = I2C_PORT_USED,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &lcd_i2c_bus));

    const i2c_device_config_t device_config = 
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_I2C_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(lcd_i2c_bus, &device_config, &lcd_i2c_device));

    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x30, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30, false);
    esp_rom_delay_us(150);
    lcd_write_nibble(0x30, false);
    lcd_write_nibble(0x20, false);      /* Seleciona interface de 4 bits.       */
    lcd_command(0x28);                  /* 2 linhas, fonte 5x8.                 */
    lcd_command(0x0C);                  /* Display ligado, cursor desligado.    */
    lcd_command(0x06);                  /* Incrementa cursor.                   */
    lcd_command(0x01);                  /* Limpa display.                       */
    ESP_LOGI(TAG, "LCD I2C inicializado no endereço 0x%02X", LCD_I2C_ADDRESS);
}

static void lcd_set_cursor(uint8_t column, uint8_t row)
{
    const uint8_t row_offset[] = {0x00, 0x40};
    lcd_command((uint8_t)(0x80 | (column + row_offset[row > 0 ? 1 : 0])));
}

static void lcd_write_line(uint8_t row, const char *text)
{
    char line[17];
    memset(line, ' ', 16);
    line[16] = '\0';

    if(text != NULL) 
    {
        size_t length = strlen(text);

        if(length > 16) length = 16;

        memcpy(line, text, length);
    }

    lcd_set_cursor(0, row);
    for(size_t i = 0; i < 16; i++) lcd_send_byte((uint8_t)line[i], true);
}

static void lcd_task(void *argument)
{
    (void)argument;
    lcd_init();

    while(true) 
    {
        lcd_notice_t notice;
        if(xQueueReceive(lcd_notice_queue, &notice, pdMS_TO_TICKS(300)) == pdTRUE) 
        {
            char line2[17];
            if(notice.is_start) 
            {
                char line1[17];
                snprintf(line1, sizeof(line1), "%s EVENT START", state_name(notice.type));
                lcd_write_line(0, line1);
                lcd_write_line(1, "Monitoring...");
            } 

            else 
            {
                char line1[17];
                snprintf(line1, sizeof(line1), "%s EVENT END", state_name(notice.type));
                snprintf(line2, sizeof(line2), "Dur:%5.1f s", (double)notice.duration_ms / 1000.0);
                lcd_write_line(0, line1);
                lcd_write_line(1, line2);
            }

            vTaskDelay(pdMS_TO_TICKS(1600));
            continue;
        }

        system_view_t view;
        if(read_system_view(&view)) 
        {
            char line1[17];
            char line2[17];
            snprintf(line1, sizeof(line1), "V:%5.1f PU:%1.2f", (double)view.measurement.grid_rms_v, (double)view.measurement.pu);
            snprintf(line2, sizeof(line2), "STATE: %-6s", state_name(view.state));
            lcd_write_line(0, line1);
            lcd_write_line(1, line2);
        }
    }
}

/* ============================== CARTÃO SD ============================== */

static sdmmc_card_t *sd_card;
static bool sd_mounted;

static esp_err_t mount_sd_card(void)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_config = 
    {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t error = spi_bus_initialize(host.slot, &bus_config, SPI_DMA_CH_AUTO);

    if(error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_GPIO;
    slot_config.host_id = host.slot;

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = 
    {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    error = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card);

    if(error == ESP_OK) 
    {
        sdmmc_card_print_info(stdout, sd_card);
        sd_mounted = true;
    }

    return error;
}

static esp_err_t write_event_to_sd(const power_event_t *event)
{
    struct stat file_stat;
    bool write_header = (stat(SD_LOG_FILE, &file_stat) != 0 || file_stat.st_size == 0);

    FILE *file = fopen(SD_LOG_FILE, "a");

    if(file == NULL) 
    {
        ESP_LOGE(TAG, "Não foi possível abrir %s", SD_LOG_FILE);
        return ESP_FAIL;
    }

    if(write_header) 
    {
        fprintf(file, "start_epoch_s,end_epoch_s,duration_ms,type,min_pu,max_pu,min_vrms,max_vrms\n");
    }

    fprintf(file, "%lld,%lld,%lld,%s,%.5f,%.5f,%.3f,%.3f\n",
            (long long)event->start_epoch_s,
            (long long)event->end_epoch_s,
            (long long)event->duration_ms,
            state_name(event->type),
            event->min_pu, event->max_pu,
            event->min_grid_rms_v, 
            event->max_grid_rms_v
    );

    bool write_ok = (fflush(file) == 0);
    if(fclose(file) != 0) write_ok = false;

    return write_ok ? ESP_OK : ESP_FAIL;
}

static void sd_card_task(void *argument)
{
    (void)argument;
    TickType_t next_mount_attempt = 0;

    while(true) 
    {
        if(!sd_mounted) 
        {
            if(xTaskGetTickCount() < next_mount_attempt) 
            {
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            esp_err_t error = mount_sd_card();

            if(error != ESP_OK) 
            {
                ESP_LOGE(TAG, "Montagem do SD falhou: %s; nova tentativa em 10 s",
                         esp_err_to_name(error));
                next_mount_attempt = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            ESP_LOGI(TAG, "Cartão SD montado em %s", SD_MOUNT_POINT);
        }

        power_event_t event;

        if(xQueuePeek(sd_event_queue, &event, pdMS_TO_TICKS(500)) != pdTRUE) continue;
      
        esp_err_t error = write_event_to_sd(&event);

        if(error == ESP_OK) 
        {
            (void)xQueueReceive(sd_event_queue, &event, 0);
            ESP_LOGI(TAG, "Evento %s gravado no SD", state_name(event.type));
        } 
        
        else 
        {
            ESP_LOGE(TAG, "Falha de gravação; desmontando SD para tentar novamente");
            (void)esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
            sd_card = NULL;
            sd_mounted = false;
            next_mount_attempt = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
        }
    }
}

/* ================================ WI-FI ================================ */

static void wifi_event_handler(void *argument, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)argument;
    (void)event_data;

    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } 
    
    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        xEventGroupClearBits(network_event_group, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi desconectado; tentando reconectar");
        (void)esp_wifi_connect();
    } 
    
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        xEventGroupSetBits(network_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi conectado e IP obtido");

        if(!sntp_started) 
        {
            esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "pool.ntp.org");
            esp_sntp_init();
            sntp_started = true;
        }
    }
}

static void initialize_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void)esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ================================ MQTT ================================ */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;

    switch((esp_mqtt_event_id_t)event_id) 
    {
        case MQTT_EVENT_CONNECTED:
            xEventGroupSetBits(network_event_group, MQTT_CONNECTED_BIT);
            ESP_LOGI(TAG, "MQTT conectado");
            break;
        case MQTT_EVENT_DISCONNECTED:
            xEventGroupClearBits(network_event_group, MQTT_CONNECTED_BIT);
            ESP_LOGW(TAG, "MQTT desconectado");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Erro reportado pelo cliente MQTT");
            break;
        default:
            break;
    }
}

static void format_event_csv(const power_event_t *event, char *buffer, size_t buffer_size)
{
    snprintf(buffer, buffer_size, "%lld,%lld,%lld,%s,%.5f,%.5f,%.3f,%.3f",
             (long long)event->start_epoch_s,
             (long long)event->end_epoch_s,
             (long long)event->duration_ms,
             state_name(event->type),
             event->min_pu, 
             event->max_pu,
             event->min_grid_rms_v, event->max_grid_rms_v
    );
}

static void mqtt_task(void *argument)
{
    (void)argument;
    xEventGroupWaitBits(network_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    const esp_mqtt_client_config_t mqtt_config = 
    {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    if(mqtt_client == NULL) 
    {
        ESP_LOGE(TAG, "Não foi possível criar cliente MQTT");
        vTaskDelete(NULL);
        return;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                                    mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

    while(true) 
    {
        power_event_t event;
        if(xQueuePeek(mqtt_event_queue, &event, pdMS_TO_TICKS(500)) != pdTRUE) continue;
       
        EventBits_t bits = xEventGroupWaitBits(network_event_group, MQTT_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(1000));
        if((bits & MQTT_CONNECTED_BIT) == 0) continue; 
        
        char payload[192];
        format_event_csv(&event, payload, sizeof(payload));
        int message_id = esp_mqtt_client_enqueue(mqtt_client, MQTT_TOPIC_EVENTS, payload, 0, 1, 0, true);
        
        if(message_id >= 0) 
        {
            (void)xQueueReceive(mqtt_event_queue, &event, 0);
            ESP_LOGI(TAG, "Evento enviado como CSV, msg_id=%d", message_id);
        } 

        else 
        {
            ESP_LOGW(TAG, "Publicação MQTT falhou; evento permanece na fila");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/* ============================== INICIALIZAÇÃO ========================== */

static void initialize_leds(void)
{
    const uint64_t led_mask = (1ULL << LED_NORMAL_GPIO) | (1ULL << LED_SAG_GPIO) | (1ULL << LED_SWELL_GPIO);

    const gpio_config_t config = 
    {
        .pin_bit_mask = led_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(LED_NORMAL_GPIO, 1);
    gpio_set_level(LED_SAG_GPIO, 0);
    gpio_set_level(LED_SWELL_GPIO, 0);
}

void app_main(void)
{
    esp_err_t error = nvs_flash_init();
    if(error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    else 
    {
        ESP_ERROR_CHECK(error);
    }

    initialize_leds();
    initialize_adc();

    sample_semaphore = xSemaphoreCreateCounting(SAMPLE_SEMAPHORE_DEPTH, 0);
    measurement_queue = xQueueCreate(RMS_QUEUE_LENGTH, sizeof(measurement_t));
    sd_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(power_event_t));
    mqtt_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(power_event_t));
    lcd_notice_queue = xQueueCreate(LCD_NOTICE_QUEUE_LENGTH, sizeof(lcd_notice_t));
    view_mutex = xSemaphoreCreateMutex();
    network_event_group = xEventGroupCreate();

    if(sample_semaphore == NULL || measurement_queue == NULL || sd_event_queue == NULL || mqtt_event_queue == NULL ||
       lcd_notice_queue == NULL || view_mutex == NULL || network_event_group == NULL
    ) 
    {
        ESP_LOGE(TAG, "Falha ao criar um ou mais recursos FreeRTOS");
        abort();
    }

    initialize_sample_timer();
    initialize_wifi();

    BaseType_t created = pdPASS;
    created &= xTaskCreate(adc_task, "adc_task", 4096, NULL, 18, NULL);
    created &= xTaskCreate(fsm_task, "fsm_task", 4096, NULL, 12, NULL);
    created &= xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 5, NULL);
    created &= xTaskCreate(sd_card_task, "sd_card_task", 6144, NULL, 6, NULL);
    created &= xTaskCreate(mqtt_task, "mqtt_task", 6144, NULL, 5, NULL);
    if(created != pdPASS) 
    {
        ESP_LOGE(TAG, "Falha ao criar uma ou mais tarefas FreeRTOS");
        abort();
    }

    ESP_ERROR_CHECK(gptimer_start(sample_timer));
    ESP_LOGI(TAG, "Monitor iniciado: nominal=%.1f Vrms, fator=%.2f, offset=%.2f V", GRID_NOMINAL_VRMS, VOLTAGE_DIVIDER_RATIO, ADC_DC_OFFSET_V);
    ESP_LOGW(TAG, "Confirme a faixa elétrica real no pino GPIO34 antes de usar a entrada");
}
