#include <stdio.h>
#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"

// ======= Definição de pinos =======
#define BOTAO_A       37   // Botão A (aumenta alarme)
#define BOTAO_B       36   // Botão B (diminui alarme)
#define BUZZER        18   // Buzzer PWM
#define LED1          5
#define LED2          6
#define LED3          7
#define LED4          15

#define I2C_SCL       21   // Pino SCL do I2C
#define I2C_SDA       20   // Pino SDA do I2C
#define I2C_NUM       I2C_NUM_0
#define I2C_FREQ      100000
#define LCD_ADDR      0x27

// ======= ADC1 =======
#define ADC_CANAL     ADC1_CHANNEL_7  // GPIO 8 (ADC1)
#define ADC_ATEN      ADC_ATTEN_DB_11
#define ADC_LARGURA   ADC_WIDTH_BIT_12

// ======= Parâmetros do NTC =======
#define R_FIXO        10000.0
#define NTC_R0        10000.0
#define NTC_T0        25.0
#define NTC_BETA      3950.0

// ======= Controle =======
int temp_alarme = 25;
#define PASSO_ALARME  5
#define DEBOUNCE_MS   200

float temp_atual = 0.0;
static QueueHandle_t fila_gpio = NULL;

static const char *TAG = "DAQ_LCD_NTC";

// ================= LCD VIA I2C =================
#define PCF8574_BACKLIGHT  (1 << 3)
#define PCF8574_EN         (1 << 2)
#define PCF8574_RS         (1 << 0)

// Inicializa o barramento I2C para comunicação com o display LCD
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

// Envia um byte para o expansor PCF8574 (controla o LCD)
static esp_err_t pcf_write_byte(uint8_t data) {
    return i2c_master_write_to_device(I2C_NUM, LCD_ADDR, &data, 1, 100 / portTICK_PERIOD_MS);
}

// Gera um pulso no pino EN do LCD para confirmar comando
static void lcd_pulse_enable(uint8_t data) {
    pcf_write_byte(data | PCF8574_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    pcf_write_byte(data & ~PCF8574_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// Envia 4 bits para o LCD (modo de 4 bits)
static void lcd_write4bits(uint8_t nibble, uint8_t flags) {
    uint8_t data = (nibble & 0xF0) | (flags & (PCF8574_RS | PCF8574_BACKLIGHT));
    pcf_write_byte(data);
    lcd_pulse_enable(data);
}

// Envia um comando para o LCD (como limpar tela, mover cursor, etc.)
static void lcd_send_cmd_byte(uint8_t cmd) {
    uint8_t back = PCF8574_BACKLIGHT;
    lcd_write4bits(cmd & 0xF0, back);
    lcd_write4bits((cmd << 4) & 0xF0, back);
}

// Envia um caractere para o LCD
static void lcd_send_data_byte(uint8_t data) {
    uint8_t back_rs = PCF8574_BACKLIGHT | PCF8574_RS;
    lcd_write4bits(data & 0xF0, back_rs);
    lcd_write4bits((data << 4) & 0xF0, back_rs);
}

// Envia uma string de texto para o LCD
static void lcd_send_string(const char *str) {
    while (*str) lcd_send_data_byte((uint8_t)*str++);
}

// Define a posição do cursor no LCD (linha e coluna)
static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t row_addr[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send_cmd_byte(0x80 | (row_addr[row] + col));
}

// Inicializa o display LCD em modo 4 bits
static void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t back = PCF8574_BACKLIGHT;
    lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write4bits(0x20, back); vTaskDelay(pdMS_TO_TICKS(1));

    lcd_send_cmd_byte(0x28); // 4-bit, 2 linhas
    lcd_send_cmd_byte(0x08); // Display off
    lcd_send_cmd_byte(0x01); // Limpa
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_cmd_byte(0x06); // Modo entrada
    lcd_send_cmd_byte(0x0C); // Display on, cursor off
}

// ================= Funções de temperatura =================
// Lê o valor do NTC no ADC e converte para temperatura em °C
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
// Configura o PWM para controlar o buzzer
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

// Liga o buzzer (ativa PWM)
void buzzer_on() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Desliga o buzzer (zera PWM)
void buzzer_off() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ================= Interrupção e botões =================
// Interrupção que é chamada quando um botão é pressionado
static void IRAM_ATTR isr_botao(void* arg) {
    uint32_t gpio = (uint32_t)arg;
    xQueueSendFromISR(fila_gpio, &gpio, NULL);
}

// Tarefa que trata os botões e ajusta o valor do alarme
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

// ================= Tarefa principal =================
// Lê temperatura, atualiza LCD e controla LEDs/buzzer
void tarefa_principal(void* arg) {
    bool buzzer_state = false;
    bool blink = false;

    while (1) {
        temp_atual = ler_ntc();
        float dif = temp_alarme - temp_atual;

        char linha1[20], linha2[20];
        sprintf(linha1, "Temp: %.1f C", temp_atual);
        sprintf(linha2, "Alarm: %d C", temp_alarme);

        // Atualiza o LCD com temperatura e alarme
        lcd_send_cmd_byte(0x01);
        vTaskDelay(pdMS_TO_TICKS(2));
        lcd_set_cursor(0, 0);
        lcd_send_string(linha1);
        lcd_set_cursor(1, 0);
        lcd_send_string(linha2);

        // Liga buzzer e pisca LEDs se temperatura >= alarme
        if (temp_atual >= temp_alarme) {
            if (!buzzer_state) {
                buzzer_on();
                buzzer_state = true;
            }
            blink = !blink;
            gpio_set_level(LED1, blink);
            gpio_set_level(LED2, blink);
            gpio_set_level(LED3, blink);
            gpio_set_level(LED4, blink);
        } else {
            // Desliga buzzer e mostra nível da diferença com LEDs
            if (buzzer_state) {
                buzzer_off();
                buzzer_state = false;
            }
            gpio_set_level(LED1, (dif <= 20));
            gpio_set_level(LED2, (dif <= 15));
            gpio_set_level(LED3, (dif <= 10));
            gpio_set_level(LED4, (dif <= 2));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Atualiza a cada 1s
    }
}

// ================= Função principal =================
// Configura tudo e inicia as tarefas do sistema
void app_main() {
    ESP_LOGI(TAG, "Inicializando sistema...");

    // Configuração do ADC1
    adc1_config_width(ADC_LARGURA);
    adc1_config_channel_atten(ADC_CANAL, ADC_ATEN);

    // PWM
    pwm_iniciar();

    // LEDs
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);

    // Botões
    gpio_config_t conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BOTAO_A) | (1ULL<<BOTAO_B),
        .pull_up_en = 1
    };
    gpio_config(&conf);

    // I2C + LCD
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_init();

    // Fila e interrupções
    fila_gpio = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_A, isr_botao, (void*)BOTAO_A);
    gpio_isr_handler_add(BOTAO_B, isr_botao, (void*)BOTAO_B);

    // Tarefas
    xTaskCreate(tarefa_botoes, "Botoes", 2048, NULL, 10, NULL);
    xTaskCreate(tarefa_principal, "Principal", 4096, NULL, 5, NULL);
}
