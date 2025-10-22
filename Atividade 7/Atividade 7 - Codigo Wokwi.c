#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define I2C_MASTER_SCL_IO           20
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

#define SSD1306_ADDR                0x3C
#define MPU6050_ADDR                0x68

#define LED_GPIO                    42
#define NUM_MEDIDAS                 10

float acc_x[NUM_MEDIDAS], acc_y[NUM_MEDIDAS], acc_z[NUM_MEDIDAS];
int idx = 0;

// Fonte 5x7 para dígitos 0-9
// Bitmap para converter digitos em valores para display
const uint8_t font5x7[][5] = {
    {0x7E,0x81,0x81,0x81,0x7E}, // 0
    {0x00,0x82,0xFF,0x80,0x00}, // 1
    {0xE2,0x91,0x91,0x91,0x8E}, // 2
    {0x42,0x81,0x89,0x89,0x76}, // 3
    {0x1C,0x12,0x11,0xFF,0x10}, // 4
    {0x4F,0x89,0x89,0x89,0x71}, // 5
    {0x7E,0x89,0x89,0x89,0x72}, // 6
    {0x01,0x01,0xF1,0x09,0x07}, // 7
    {0x76,0x89,0x89,0x89,0x76}, // 8
    {0x4E,0x91,0x91,0x91,0x7E}, // 9
};

// -------------------- Funções I2C --------------------
esp_err_t i2c_write(uint8_t addr, uint8_t *data, size_t len) {
  // Imprime uma saida através do I2C
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, data, len, 1000 / portTICK_PERIOD_MS);
}

esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
  // Realiza uma leitura através do I2C
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

void i2c_master_init() {
  // FUnção que inicializa e configura o I2C do esp
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// -------------------- MPU6050 --------------------
void mpu6050_init() {
  // Função que inicializa o MPU6050
    uint8_t data[2] = {0x6B, 0x00}; // acorda sensor
    i2c_write(MPU6050_ADDR, data, 2);
}

void mpu6050_read(float *x, float *y, float *z) {
  // Ler os valores dos eixos x, y, z
    uint8_t buf[6];
    i2c_read(MPU6050_ADDR, 0x3B, buf, 6);

    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];

    *x = raw_x / 16384.0;
    *y = raw_y / 16384.0;
    *z = raw_z / 16384.0;
}

// -------------------- SSD1306 --------------------
void ssd1306_send_cmd(uint8_t cmd) {
  // Imprime uma mensagem no display
    uint8_t data[2] = {0x00, cmd};
    i2c_write(SSD1306_ADDR, data, 2);
}

void ssd1306_init() {
  // Inicializa o ssd1306
    uint8_t init_cmds[] = {
        0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,0x40,
        0x81,0xFF,0xA1,0xA6,0xA8,0x3F,0xD3,0x00,
        0xD5,0xF0,0xD9,0x22,0xDA,0x12,0xDB,0x20,
        0x8D,0x14,0xAF
    };
    for(int i=0;i<sizeof(init_cmds);i++) ssd1306_send_cmd(init_cmds[i]);
}

void ssd1306_clear() {
  // FUnção para limpar a tela do ssd1306
    for(uint8_t page=0; page<8; page++){
        ssd1306_send_cmd(0xB0 + page);
        ssd1306_send_cmd(0x00);
        ssd1306_send_cmd(0x10);
        uint8_t buf[129];
        buf[0] = 0x40;
        memset(buf+1,0,128);
        i2c_write(SSD1306_ADDR, buf, 129);
    }
}

void ssd1306_draw_digit(uint8_t digit, uint8_t col, uint8_t page){
  // Função que desenha no display a mensagem
    if(digit>9) return;
    uint8_t buf[6];
    buf[0] = 0x40;
    for(int i=0;i<5;i++) buf[i+1] = font5x7[digit][i];

    ssd1306_send_cmd(0xB0 + page);      // page
    ssd1306_send_cmd(0x00 + col);       // low col
    ssd1306_send_cmd(0x10 + (col>>4));  // high col
    i2c_write(SSD1306_ADDR, buf, 6);
}


void ssd1306_draw_number(float num, uint8_t page){
  // Converte float para dígitos e desenha no display
    if(num<0) num=0;
    int val = (int)(num*10); // 1 casa decimal
    ssd1306_draw_digit(val/10, 0, page);
    ssd1306_draw_digit(val%10, 6, page);
}

// -------------------- Main --------------------
void app_main() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    i2c_master_init();

    // Detecta dispositivos
    uint8_t test;
    if(i2c_read(MPU6050_ADDR, 0x75, &test, 1) == ESP_OK)
        printf("MPU6050 OK\n");
    else
        printf("MPU6050 nao detectado!\n");

    if(i2c_write(SSD1306_ADDR, (uint8_t[]){0x00, 0xAF}, 2) == ESP_OK)
        printf("SSD1306 OK\n");
    else
        printf("SSD1306 nao detectado!\n");

    // Inicia os dispositivos
    ssd1306_init();
    ssd1306_clear();
    mpu6050_init();

    float last_x=0, last_y=0, last_z=0;

    while(1){
        float x,y,z;
        mpu6050_read(&x,&y,&z); // le o sensor

        acc_x[idx]=x; acc_y[idx]=y; acc_z[idx]=z; // guarda a leitura dos eixos
        idx = (idx+1)%NUM_MEDIDAS;

        float avg_x=0, avg_y=0, avg_z=0;
        for(int i=0;i<NUM_MEDIDAS;i++){ // Calcula as médias
            avg_x+=acc_x[i]; avg_y+=acc_y[i]; avg_z+=acc_z[i];
        }
        avg_x/=NUM_MEDIDAS; avg_y/=NUM_MEDIDAS; avg_z/=NUM_MEDIDAS;

        printf("X: %.2f Y: %.2f Z: %.2f\n", avg_x, avg_y, avg_z);

        // Pisca o led para variações maiores que 0.5 em qualquer um dos eixos
        if(fabs(avg_x-last_x)>0.5 || fabs(avg_y-last_y)>0.5 || fabs(avg_z-last_z)>0.5){
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 0);
        }
        else {
            gpio_set_level(LED_GPIO,0);
        }

        last_x=avg_x; last_y=avg_y; last_z=avg_z;

        ssd1306_clear();
        ssd1306_draw_number(avg_x,0); // Exibe media de X
        ssd1306_draw_number(avg_y,1); // Exibe media de Y
        ssd1306_draw_number(avg_z,2); // Exibe media de Z

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
