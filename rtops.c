
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"

// Biblioteca gerada pelo arquivo .pio durante compilação.
#include "ws2818b.pio.h"

// Definição do número de LEDs e pino.
#define LED_COUNT 25
#define LED_PIN 7

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define LED_RED 13
#define LED_GREEN  11
#define tam_quad 10


#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// Configuração do pino do buzzer
#define BUZZER_PIN 21

// Configuração da frequência do buzzer (em Hz)
#define BUZZER_FREQUENCY 4000

// Definição de uma função para inicializar o PWM no pino do buzzer
void pwm_init_buzzer(uint pin) {
    // Configurar o pino como saída de PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o PWM com frequência desejada
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096)); // Divisor de clock
    pwm_init(slice_num, &config, true);

    // Iniciar o PWM no nível baixo
    pwm_set_gpio_level(pin, 0);
}

// Definição de uma função para emitir um beep com duração especificada
void beep(uint pin, uint duration_ms) {
    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o duty cycle para 50% (ativo)
    pwm_set_gpio_level(pin, 2048);

    // Temporização
    sleep_ms(duration_ms);

    // Desativar o sinal PWM (duty cycle 0)
    pwm_set_gpio_level(pin, 0);

    // Pausa entre os beeps
    sleep_ms(100); // Pausa de 100ms
}

// Definição de pixel GRB
struct pixel_t {
  uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

// Declaração do buffer de pixels que formam a matriz.
npLED_t leds[LED_COUNT];

// Variáveis para uso da máquina PIO.
PIO np_pio;
uint sm;

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {

  // Cria programa PIO.
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  // Toma posse de uma máquina PIO.
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
  }

  // Inicia programa na máquina PIO obtida.
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Limpa buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  leds[index].R = r;
  leds[index].G = g;
  leds[index].B = b;
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {
  // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}





typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

QueueHandle_t xQueueJoystickData;

void vJoystickTask(void *params)
{
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;

    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

void vDisplayTask(void *params)
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, 128, 64, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    joystick_data_t joydata;
    bool cor = true;
    char text_buffer[32]; // Buffer para o texto
    
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            uint8_t x = (joydata.x_pos * (128 - tam_quad)) / 4095;
            uint8_t y = (joydata.y_pos * (64 - tam_quad)) / 4095;
            y = (64 - tam_quad) - y; // Inverte o eixo Y
            
            // Limpa a tela
    ssd1306_fill(&ssd, !cor);

    // Linha 0: Status (topo da tela)
    ssd1306_draw_string(&ssd, "Normal", 0, 0);  // X=0, Y=0
    
    // Linha 1: Water
    snprintf(text_buffer, sizeof(text_buffer), "Water:%4u", joydata.x_pos);
    ssd1306_draw_string(&ssd, text_buffer, 0, 10);  // X=0, Y=10
    
    // Linha 2: Rain
    snprintf(text_buffer, sizeof(text_buffer), "Rain:%4u", joydata.y_pos);
    ssd1306_draw_string(&ssd, text_buffer, 0, 20); // X=0, Y=20
    
    // Atualiza a tela
    ssd1306_send_data(&ssd);
        } 
        
        
        if((joydata.x_pos > 3500) || (joydata.y_pos > 3500)){
                // Limpa a tela
    ssd1306_fill(&ssd, !cor);

    // Linha 0: Status (topo da tela)
    ssd1306_draw_string(&ssd, "PERIGO", 0, 0);  // X=0, Y=0
    
    // Linha 1: Water
    snprintf(text_buffer, sizeof(text_buffer), "Water:%4u", joydata.x_pos);
    ssd1306_draw_string(&ssd, text_buffer, 0, 10);  // X=0, Y=10
    
    // Linha 2: Rain
    snprintf(text_buffer, sizeof(text_buffer), "Rain:%4u", joydata.y_pos);
    ssd1306_draw_string(&ssd, text_buffer, 0, 20); // X=0, Y=20
    
    // Atualiza a tela
    ssd1306_send_data(&ssd);

            }
    }
}

void vLedGreenTask(void *params)
{
   gpio_init(LED_GREEN);
   gpio_set_dir(LED_GREEN, GPIO_OUT);
   gpio_init(LED_RED);
   gpio_set_dir(LED_RED, GPIO_OUT);

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if((joydata.x_pos > 3500) || (joydata.y_pos > 3500)){
                gpio_put(LED_RED, true);
                gpio_put(LED_GREEN, false);

            } else {
                gpio_put(LED_GREEN, true);
                gpio_put(LED_RED, false);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

void vBuzzweTask(void *params)
{
     // Inicializar o PWM no pino do buzzer
    pwm_init_buzzer(BUZZER_PIN);

        joystick_data_t joydata;
        while (true)
        {
            if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
            {
                if((joydata.x_pos > 3500) || (joydata.y_pos > 3500)){
                    beep(BUZZER_PIN, 500); // Bipe de 500ms
            }

        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Atualiza a cada 50ms



}
}

void vLedBlueTask(void *params)
{
     // Inicializa matriz de LEDs NeoPixel.
  npInit(LED_PIN);
  npClear();

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if((joydata.x_pos > 3500) || (joydata.y_pos > 3500)){
                //npClear();
                //npWrite();
                //vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura


                npSetLED(2, 255, 0, 0); // Define o LED de índice 0 para vermelho.
                npSetLED(7, 255, 0, 0);
                npSetLED(12, 255, 0, 0);
                npSetLED(17, 255, 0, 0);
                npSetLED(22, 255, 0, 0);
                npWrite();


            } else {
                //npClear();
                //npWrite();
                //vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura


                npSetLED(2, 0, 255, 0); // Define o LED de índice 0 para vermelho.
                npSetLED(7, 0, 255, 0);
                npSetLED(12, 0, 255, 0);
                npSetLED(17, 0, 255, 0);
                npSetLED(22, 0, 255, 0);
                npWrite();
            }




        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}


// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Ativa BOOTSEL via botão
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    // Cria a fila para compartilhamento de valor do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vLedGreenTask, "LED red Task", 256, NULL, 1, NULL);
    xTaskCreate(vLedBlueTask, "LED blue Task", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzweTask, "Buzzer Task", 256, NULL, 1, NULL);

    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}