#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"

// ======= Definição de pinos =======
#define BOTAO_A       37      // Pino do botão A
#define BOTAO_B       36      // Pino do botão B
#define BUZZER        18      // Pino do buzzer
#define LED1          5       // Pino LED1
#define LED2          6       // Pino LED2
#define LED3          7       // Pino LED3
#define LED4          15      // Pino LED4

#define I2C_SCL       21      // Pino SCL do I2C
#define I2C_SDA       20      // Pino SDA do I2C
#define I2C_NUM       I2C_NUM_0
#define I2C_FREQ      100000  // Frequência do I2C
#define LCD_ADDR      0x27    // Endereço do LCD

// ======= ADC1 =======
#define ADC_CANAL     ADC1_CHANNEL_7
#define ADC_ATEN      ADC_ATTEN_DB_11
#define ADC_LARGURA   ADC_WIDTH_BIT_12

// ======= Parâmetros do NTC =======
#define R_FIXO        10000.0
#define NTC_R0        10000.0
#define NTC_T0        25.0
#define NTC_BETA      3950.0

// ======= Controle de alarme =======
int temp_alarme = 25;         // Temperatura limite do alarme
#define PASSO_ALARME  5         // Incremento/decremento do alarme
#define DEBOUNCE_MS   200       // Debounce dos botões

float temp_atual = 0.0;       // Armazena a temperatura atual
static QueueHandle_t fila_gpio = NULL;  // Fila para ISR dos botões
static const char *TAG = "DAQ_LCD_NTC";

// ================= LCD VIA I2C =================
#define PCF8574_BACKLIGHT  (1 << 3)
#define PCF8574_EN         (1 << 2)
#define PCF8574_RS         (1 << 0)

// ================= Máquina de estados =================
typedef enum {
    ESTADO_BOTOES,
    ESTADO_PWM,
    ESTADO_LCD,
    ESTADO_LEDS,
    ESTADO_SDCARD
} estado_t;

estado_t estado_atual = ESTADO_BOTOES;

// ================= SDCard =================
#define MOUNT_POINT "/sdcard"
sdmmc_card_t* card;
bool sd_montado = false;

// Inicializa e monta o SDCard via SPI
void inicializa_sdcard() {
    esp_err_t ret;
    sdmmc_card_t* card;
    const char* mount_point = MOUNT_POINT;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = host.slot;
    slot_config.gpio_cs = 10;  // CS (Chip Select)

    // Configuração do barramento SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 11,
        .miso_io_num = 13,
        .sclk_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o bus SPI (%s)", esp_err_to_name(ret));
        return;
    }

    // Configuração de montagem do filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao montar SDCard (%s)", esp_err_to_name(ret));
        sd_montado = false;
        return;
    }

    sd_montado = true;
    ESP_LOGI(TAG, "SDCard montado com sucesso!");
}

// ================= I2C / LCD =================
// Inicializa o barramento I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    esp_err_t err = i2c_param_config(I2C_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
}

// Escreve um byte no PCF8574 (LCD)
static esp_err_t pcf_write_byte(uint8_t data) {
    return i2c_master_write_to_device(I2C_NUM, LCD_ADDR, &data, 1, 100 / portTICK_PERIOD_MS);
}

// Gera pulso de enable no LCD
static void lcd_pulse_enable(uint8_t data) {
    pcf_write_byte(data | PCF8574_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    pcf_write_byte(data & ~PCF8574_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// Envia 4 bits para o LCD com flags
static void lcd_write4bits(uint8_t nibble, uint8_t flags) {
    uint8_t data = (nibble & 0xF0) | (flags & (PCF8574_RS | PCF8574_BACKLIGHT));
    pcf_write_byte(data);
    lcd_pulse_enable(data);
}

// Envia comando de 8 bits para o LCD
static void lcd_send_cmd_byte(uint8_t cmd) {
    uint8_t back = PCF8574_BACKLIGHT;
    lcd_write4bits(cmd & 0xF0, back);
    lcd_write4bits((cmd << 4) & 0xF0, back);
}

// Envia dado (caractere) para o LCD
static void lcd_send_data_byte(uint8_t data) {
    uint8_t back_rs = PCF8574_BACKLIGHT | PCF8574_RS;
    lcd_write4bits(data & 0xF0, back_rs);
    lcd_write4bits((data << 4) & 0xF0, back_rs);
}

// Envia string completa para o LCD
static void lcd_send_string(const char *str) {
    while (*str) lcd_send_data_byte((uint8_t)*str++);
}

// Define posição do cursor no LCD
static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t row_addr[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send_cmd_byte(0x80 | (row_addr[row] + col));
}

// Inicializa o LCD em modo 4 bits
static void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t back = PCF8574_BACKLIGHT;
    lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write4bits(0x20, back); vTaskDelay(pdMS_TO_TICKS(1));

    lcd_send_cmd_byte(0x28);
    lcd_send_cmd_byte(0x08);
    lcd_send_cmd_byte(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_cmd_byte(0x06);
    lcd_send_cmd_byte(0x0C);
}

// ================= Funções de temperatura =================
// Lê valor do NTC via ADC e converte para temperatura Celsius
float ler_ntc() {
    int leitura = adc1_get_raw(ADC_CANAL);
    float v = leitura * (3.3 / 4095.0);
    float r_ntc = (R_FIXO * v) / (3.3 - v);

    float T0 = NTC_T0 + 273.15;
    float invT = (1.0 / T0) + (1.0 / NTC_BETA) * log(r_ntc / NTC_R0);
    float T = (1.0 / invT) - 273.15;
    return T;
}

// ================= PWM (buzzer) =================
// Inicializa PWM para controle do buzzer
void pwm_iniciar() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t canal = {
        .gpio_num = BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&canal);
}

// Liga o buzzer
void buzzer_on() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Desliga o buzzer
void buzzer_off() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ================= Interrupção e botões =================
// ISR dos botões envia GPIO para fila
static void IRAM_ATTR isr_botao(void* arg) {
    uint32_t gpio = (uint32_t)arg;
    xQueueSendFromISR(fila_gpio, &gpio, NULL);
}

// Tarefa que trata botões e ajusta temperatura de alarme
void tarefa_botoes(void* arg) {
    uint32_t gpio_num;
    int64_t ultimo_a = 0, ultimo_b = 0;
    while (1) {
        if (xQueueReceive(fila_gpio, &gpio_num, portMAX_DELAY)) {
            int64_t agora = esp_timer_get_time() / 1000;
            if (gpio_num == BOTAO_A && (agora - ultimo_a > DEBOUNCE_MS)) {
                ultimo_a = agora;
                temp_alarme += PASSO_ALARME;
                if (temp_alarme > 100) temp_alarme = 100;
            } else if (gpio_num == BOTAO_B && (agora - ultimo_b > DEBOUNCE_MS)) {
                ultimo_b = agora;
                temp_alarme -= PASSO_ALARME;
                if (temp_alarme < 0) temp_alarme = 0;
            }
        }
    }
}

// ================= Máquina de estados =================
// Tarefa principal: lê temperatura, aciona buzzer, LCD, LEDs e SDCard
void tarefa_principal(void* arg) {
    bool buzzer_state = false;
    bool blink = false;
    char linha1[20], linha2[20];
    char sd_buffer[64];

    while (1) {
        switch (estado_atual) {

            case ESTADO_BOTOES:
                // Botões tratados na tarefa separada
                estado_atual = ESTADO_PWM;
                break;

            case ESTADO_PWM:
                temp_atual = ler_ntc();
                if (temp_atual >= temp_alarme) {
                    if (!buzzer_state) {
                        buzzer_on();
                        buzzer_state = true;
                    }
                } else {
                    if (buzzer_state) {
                        buzzer_off();
                        buzzer_state = false;
                    }
                }
                estado_atual = ESTADO_LCD;
                break;

            case ESTADO_LCD:
                sprintf(linha1, "Temp: %.1f C", temp_atual);
                sprintf(linha2, "Alarm: %d C", temp_alarme);
                lcd_send_cmd_byte(0x01);   // Limpa tela
                vTaskDelay(pdMS_TO_TICKS(2));
                lcd_set_cursor(0, 0);
                lcd_send_string(linha1);
                lcd_set_cursor(1, 0);
                lcd_send_string(linha2);
                estado_atual = ESTADO_LEDS;
                break;

            case ESTADO_LEDS: {
                float dif = temp_alarme - temp_atual;
                if (temp_atual >= temp_alarme) {
                    blink = !blink;
                    gpio_set_level(LED1, blink);
                    gpio_set_level(LED2, blink);
                    gpio_set_level(LED3, blink);
                    gpio_set_level(LED4, blink);
                } else {
                    gpio_set_level(LED1, (dif <= 20));
                    gpio_set_level(LED2, (dif <= 15));
                    gpio_set_level(LED3, (dif <= 10));
                    gpio_set_level(LED4, (dif <= 2));
                }
                estado_atual = ESTADO_SDCARD;
                break;
            }

            case ESTADO_SDCARD:
                if (sd_montado) {
                    snprintf(sd_buffer, sizeof(sd_buffer), "Temp: %.2f Alarm: %d\n", temp_atual, temp_alarme);
                    FILE* f = fopen(MOUNT_POINT"/log.txt", "a");
                    if (f != NULL) {
                        fputs(sd_buffer, f);
                        fclose(f);
                    }
                }
                estado_atual = ESTADO_BOTOES;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ================= Função principal =================
// Inicializa periféricos, filas, interrupções e tarefas
void app_main() {
    ESP_LOGI(TAG, "Inicializando sistema...");

    // Configura ADC
    adc1_config_width(ADC_LARGURA);
    adc1_config_channel_atten(ADC_CANAL, ADC_ATEN);

    // Inicializa PWM do buzzer
    pwm_iniciar();

    // Configura LEDs como saída
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);

    // Configura botões como entrada com pull-up e interrupção
    gpio_config_t conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BOTAO_A) | (1ULL<<BOTAO_B),
        .pull_up_en = 1
    };
    gpio_config(&conf);

    // Inicializa I2C e LCD
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_init();

    // Inicializa SDCard
    inicializa_sdcard();

    // Cria fila para ISR dos botões
    fila_gpio = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_A, isr_botao, (void*)BOTAO_A);
    gpio_isr_handler_add(BOTAO_B, isr_botao, (void*)BOTAO_B);

    // Cria tarefas
    xTaskCreate(tarefa_botoes, "Botoes", 2048, NULL, 10, NULL);
    xTaskCreate(tarefa_principal, "Principal", 4096, NULL, 5, NULL);
}
