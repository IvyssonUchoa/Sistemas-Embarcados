#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#define LED1_GPIO 4
#define LED2_GPIO 5
int led_state = 0;
int led_state2 = 0;

#define BUTTON_A_GPIO 21  // Alterna Led 1
#define BUTTON_B_GPIO 19  // Liga buzzer

#define BUZZER_GPIO 18
#define BUZZER_FREQ    1000
#define BUZZER_RES     LEDC_TIMER_10_BIT

#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024
int estado_var = 1; // controle de botão B ativo

//// CONFIGURAÇÕES ////

void config_leds() {
    gpio_config_t config;
    config.intr_type = GPIO_INTR_DISABLE;
    config.mode = GPIO_MODE_OUTPUT;
    config.pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO);
    config.pull_down_en = 0;
    config.pull_up_en = 0;
    gpio_config(&config);
    gpio_set_level(LED1_GPIO, 0);
    gpio_set_level(LED2_GPIO, 0);
}

void config_buzzer(){
    ledc_timer_config_t buzzer_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = BUZZER_RES,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = BUZZER_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t led_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0, // inicia desligado
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    };
    ledc_channel_config(&led_channel);
}

void config_botoes() {
    gpio_config_t config;
    config.intr_type = GPIO_INTR_DISABLE;
    config.mode = GPIO_MODE_INPUT;
    config.pin_bit_mask = (1ULL << BUTTON_A_GPIO) | (1ULL << BUTTON_B_GPIO);
    config.pull_up_en = 0;
    config.pull_down_en = 0;
    gpio_config(&config);
}

void alterna_led2() {
    led_state2 = !led_state2;
    gpio_set_level(LED2_GPIO, led_state2);
}

void config_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &alterna_led2,
        .arg = NULL,
        .name = "led2_timer"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, 2000000);
}

void config_uart() {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

//// FUNÇÕES UTILITÁRIAS ////

void chamar_buzzer(int on) {
    if(estado_var){ // só permite buzzer se botão B ativo
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, on ? 512 : 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void checa_botoes(){
    int estado_A = gpio_get_level(BUTTON_A_GPIO);
    int estado_B = gpio_get_level(BUTTON_B_GPIO);

    if (estado_A == 0) {
        led_state = !led_state;
        gpio_set_level(LED1_GPIO, led_state);
        vTaskDelay(pdMS_TO_TICKS(200)); // debounce
    }

    if (estado_B == 0 && estado_var) { // só permite se estado_var == 1
        chamar_buzzer(1);
        vTaskDelay(pdMS_TO_TICKS(1500));
        chamar_buzzer(0);
    }
}

void ler_uart() {
  // Leo caracter recebido pela uart

    uint8_t data[UART_BUF_SIZE];
    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(100));

    if (len > 0) {
        data[len] = '\0';
        if (data[len-1] == '\n' || data[len-1] == '\r') data[len-1] = '\0';

        if (strcmp((char*)data, "a") == 0) {
            estado_var = 0; // desativa botão B
            ESP_LOGI("UART", "Botão B desativado");
        } else if (strcmp((char*)data, "b") == 0) {
            estado_var = 1; // ativa botão B
            ESP_LOGI("UART", "Botão B ativado");
        }
    }
}

void app_main(void) {
    config_leds();
    config_botoes();
    config_buzzer();
    config_timer();
    config_uart();

    while (1) {
        checa_botoes();
        ler_uart();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
