#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// GPIOs conectadas aos LEDs
#define LED1_GPIO 4
#define LED2_GPIO 5
#define LED3_GPIO 6
#define LED4_GPIO 7
#define DELAY_MS 1000

void config_leds() {
  // Função para configurar os GPIOS dos LEDs como saídas

    gpio_config_t config;
    config.intr_type = GPIO_INTR_DISABLE;
    config.mode = GPIO_MODE_OUTPUT;
    config.pin_bit_mask = (1ULL << LED1_GPIO) |
                          (1ULL << LED2_GPIO) |
                          (1ULL << LED3_GPIO) |
                          (1ULL << LED4_GPIO);
    config.pull_down_en = 0;
    config.pull_up_en = 0;
    gpio_config(&config);
}

void set_led(int index, int state) {
  // Função para controlar os LEDS
    // index: indica o LED controlado
    // state: indica o estado setado (1:ON/ 0:OFF)

    int gpio_num;
    switch(index) {
        case 0: gpio_num = LED1_GPIO; break;
        case 1: gpio_num = LED2_GPIO; break;
        case 2: gpio_num = LED3_GPIO; break;
        case 3: gpio_num = LED4_GPIO; break;
        default: return;
    }
    gpio_set_level(gpio_num, state);
}

void set_leds_binary(uint8_t value) {
  // Recebe um valor de 0 a 15 (binário de 4 bits) 
  // Acende os LEDs para representar esse valor

    for (int j = 0; j < 4; j++) {
        int bit = (value >> j) & 0x01;
        set_led(3 - j, bit);  // Inverte a ordem dos LEDs
    }
}

void fase1_binario() {
  // Realiza a contagem de 0 a 15 (0000 até 1111)
  // Aciona os LEDs respectivos para a contagem

    for (int i = 0; i < 16; i++) {
        set_leds_binary(i);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void fase2_varredura() {
  //→ Os LEDs acendem em sequência

    // Frente (LED1 → LED2 → LED3 → LED4)
    for (int i = 0; i < 4; i++) {
        set_leds_binary(0);  // Apaga todos
        set_led(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    // Voltar (LED4 → LED3 → LED2 → LED1).
    for (int i = 2; i >= 0; i--) {
        set_leds_binary(0);
        set_led(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void app_main(void) {
    config_leds();

    while (1) {
        fase1_binario();
        fase2_varredura();
    }
}
