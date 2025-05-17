#include "pico/stdlib.h"    // Definições padrões do rasp pi pico
#include "pico/bootrom.h"   // Para acelerar o desenvolvimento ao entrar no modo Bootsel apertando o botão A da bitDogLab
#include "hardware/gpio.h"  // Funções padrões para manusear as Gpios
#include "hardware/adc.h"   // Para ler os valores do joystick com o conversor analógico para digital
#include "hardware/i2c.h"   // Para realizar a conexão e comunicação com o display oled 
#include "hardware/pwm.h"   // Para controlar o buzzer da gpio 21
#include "hardware/pio.h"   // Para controlar a matriz de leds da gpio 7
#include "lib/ssd1306.h"    // Para desenhar no diplay oled
#include "lib/font.h"       // Fonte usada no display oled
#include "FreeRTOS.h"       // Configurações basicas do FreeRtos
#include "task.h"           // Para criar e manuser o scheduler e as tasks
#include "queue.h"          // Para criar as filas que farão a comunicação entre tasks


// Biblioteca gerada pelo arquivo .pio durante compilação.
#include "ws2812.pio.h"

// Constantes para display ssd1306 e comunicação i2c
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C // Endereço do display ssd1306
ssd1306_t ssd; // Inicializa a estrutura do display

// Gpios usadas
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define GPIO_LED_VERMELHO 13    
#define GPIO_LED_AZUL 12
#define GPIO_PIN_BUZZER 21
#define GPIO_MATRIZ_LEDS 7
#define QUANTIDADE_LEDS 25 // Quantidade de leds presentes na matriz, necessário definir para poder percorrer toda a matriz com for()
#define JOYSTICK_X_PIN 26  
#define JOYSTICK_Y_PIN 27  
#define BOTAO_A_PARA_BOOTSEL 5 

typedef struct
{
    // VALOR MINIMO LIDO PELO JOYSTICK = 24
    // VALOR MÁXIMO LIDO PELO JOYSTICK = 4088
    // Quando leio o valor bruto do adc, eu ja converto em porcentagem e armazeno na fila
    uint16_t x_pos_nivel_agua_rios_porcentagem;
    uint16_t y_pos_volume_chuvas_porcentagem;
} joystick_data_t;

// Enumeration usada tanto para sincronização, quanto para separar as funcionalidades de cada modo de operação
typedef enum {
    NORMAL,
    ALERTA
}Modo;

volatile Modo modo_operacao = NORMAL;

// Filas usadas
QueueHandle_t xQueueDisplay;
QueueHandle_t xQueueModoOperacao;
QueueHandle_t xQueueLeds;


// Buffers para formar os leds na matriz de leds baseados na porcentagem do nivel de agua e chuva
bool leds_ate_0_nivel_agua[QUANTIDADE_LEDS] = {
    1, 1, 0, 0, 0,
    0, 0, 0, 1, 1,
    1, 1, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};

bool leds_ate_0_volume_chuva[QUANTIDADE_LEDS] = {
    0, 0, 0, 1, 1,
    1, 1, 0, 0, 0,
    0, 0, 0, 1, 1,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};


bool leds_ate_20_nivel_agua[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    0, 0, 0, 1, 1,
    1, 1, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};

bool leds_ate_20_volume_chuva[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 0, 0,
    0, 0, 0, 1, 1,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};


bool leds_ate_40_volume_chuva[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    0, 0, 0, 1, 1,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};


bool leds_ate_40_nivel_agua[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};

bool leds_ate_60_nivel_agua_e_volume_chuva[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};

bool leds_ate_80_nivel_agua[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 0, 0,
    0, 0, 0, 0, 0
};

bool leds_ate_80_volume_chuva[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    0, 0, 0, 1, 1,
    0, 0, 0, 0, 0
};


bool leds_ate_100_nivel_agua[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 0, 0,
    0, 0, 0, 1, 1
};

bool leds_ate_100_volume_chuva[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    0, 0, 0, 1, 1,
    1, 1, 0, 0, 0
};

bool leds_ate_100_volume_chuva_e_nivel_agua[QUANTIDADE_LEDS] = {
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1,
    1, 1, 0, 1, 1
};






// FUNÇÕES NORMAIS ======================================================================================================


void gpio_irq_handler(uint gpio, uint32_t events){
    reset_usb_boot(0, 0);
}


// Para mandar um valor grb de 32bits(mas so 24 sendo usados) para a maquina de estado 0 do bloco 0 do PIO
static inline void put_pixel(uint32_t pixel_grb){
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// cria um valor grb de 32 bits
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b){
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void set_one_led(uint8_t r, uint8_t g, uint8_t b, bool desenho[]){
    // Define a cor com base nos parâmetros fornecidos
    uint32_t color = urgb_u32(r, g, b);

    // Define todos os LEDs com a cor especificada
    for (int i = 0; i < QUANTIDADE_LEDS; i++)
    {
        if (desenho[i])
        {
            put_pixel(color); // Liga o LED com um no buffer
        }
        else
        {
            put_pixel(0);  // Desliga os LEDs com zero no buffer
        }
    }
}

// Função que um número entre 0 e 99(incluso), e os tranforma em "String" colocando no final "%" para printar no display oled
void int_to_str(int value, char *str) {
    if (value < 10) {
        str[0] = '0';
        str[1] = (value % 10) + '0';
        str[2] = '%';
        str[3] = '\0';
    } else if (value < 100) {
        str[0] = (value / 10) + '0';
        str[1] = (value % 10) + '0';
        str[2] = '%';
        str[3] = '\0';
    }
}







// TASKS ======================================================================================================

// Task que controla a matriz de leds e led RGB
void vMatrizLedELedRgbTask(void * params){
    // Inicializa o bloco pio,sua state machine e o programa em assembly
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, GPIO_MATRIZ_LEDS, 800000, false);
    // Inicializa as gpios usadas para o led rgb e define como saída
    gpio_init(GPIO_LED_VERMELHO);
    gpio_init(GPIO_LED_AZUL);
    gpio_set_dir(GPIO_LED_VERMELHO,GPIO_OUT);
    gpio_set_dir(GPIO_LED_AZUL,GPIO_OUT);

    // Variavel local do tipo "joystick_data" , para armazenar os valores lidos na fila "xQueueLeds"
    joystick_data_t joydata;


    while (true){
        // Se tiver dados na fila, armazene na variável 
        if (xQueueReceive(xQueueLeds, &joydata, portMAX_DELAY) == pdTRUE){
            // Altera entre os modos de operação
            switch (modo_operacao){
                // Modo normal ele alterá duas barras verticais na matriz baseado na porcentagem lida de volume de chuva ou nivel de agua no rio
                // Como a matriz é 5x5 cada linha representa 20% 
                // Com o joystick para tanto o eixo x quanto o eixo y ficam abaixo de 60%, deixando ligados somente os 3 primeiros leds
                // Caso o valor lido seja 0% , todos os leds da colunam que representam as porcentagens são apagados
                case NORMAL:
                    gpio_put(GPIO_LED_VERMELHO,0);
                    gpio_put(GPIO_LED_AZUL,1);
                    if (joydata.x_pos_nivel_agua_rios_porcentagem == 0 && joydata.y_pos_volume_chuvas_porcentagem <= 60){
                        set_one_led(0,0,255,leds_ate_0_nivel_agua);
                    }       
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 60 && joydata.y_pos_volume_chuvas_porcentagem == 0){
                        set_one_led(0,0,255,leds_ate_0_volume_chuva);
                    }           
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 20 && joydata.y_pos_volume_chuvas_porcentagem <= 60){
                        set_one_led(0,0,255,leds_ate_20_nivel_agua);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 60  && joydata.y_pos_volume_chuvas_porcentagem <= 20){
                        set_one_led(0,0,255,leds_ate_20_volume_chuva);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 40 && joydata.y_pos_volume_chuvas_porcentagem <= 60){
                        set_one_led(0,0,255,leds_ate_40_nivel_agua);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 60 && joydata.y_pos_volume_chuvas_porcentagem <= 40){
                        set_one_led(0,0,255,leds_ate_40_volume_chuva);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 60 && joydata.y_pos_volume_chuvas_porcentagem <= 60){
                        set_one_led(0,0,255,leds_ate_60_nivel_agua_e_volume_chuva);
                    }
                    break;
                case ALERTA:
                    // Unica coisa que muda no modo alerta é que os leds ficam vermelhos
                    // E somente os dois ultimos leds de cima da coluna são alterados, representando os 80 e 100%
                    gpio_put(GPIO_LED_AZUL,0);
                    gpio_put(GPIO_LED_VERMELHO,1);
                    if (joydata.x_pos_nivel_agua_rios_porcentagem >= 90 && joydata.y_pos_volume_chuvas_porcentagem >= 90){
                        set_one_led(255,0,0,leds_ate_100_volume_chuva_e_nivel_agua);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem >= 90 && joydata.y_pos_volume_chuvas_porcentagem <= 60){
                        set_one_led(255,0,0,leds_ate_100_nivel_agua);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem >= 70 && joydata.y_pos_volume_chuvas_porcentagem <= 60){
                        set_one_led(255,0,0,leds_ate_80_nivel_agua);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 60 && joydata.y_pos_volume_chuvas_porcentagem >= 90){
                        set_one_led(255,0,0,leds_ate_100_volume_chuva);
                    }
                    else if (joydata.x_pos_nivel_agua_rios_porcentagem <= 60 && joydata.y_pos_volume_chuvas_porcentagem >= 70){
                        set_one_led(255,0,0,leds_ate_80_volume_chuva);
                    }
                    break;
            }
        }
    }
        
}

// Task para controlar o buzzer
void vEmiteSinalBuzzerTask(void *params){
    // Configurações necessárias para controle do pwm
    uint32_t wrap = 12500;
    gpio_set_function(GPIO_PIN_BUZZER, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(GPIO_PIN_BUZZER);
    pwm_config config = pwm_get_default_config();
    pwm_set_wrap(slice_num, wrap - 1);
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys)/(850 * wrap)); 
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(GPIO_PIN_BUZZER, 0);

    // Pega a contagem de ticks desde que o sistema foi iniciado
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Cria uma contagem de ticks precisa de 100ms
    const TickType_t xPeriod = pdMS_TO_TICKS(100);
    
    // Não vi necessidade desta task usar filas
    while (true){
        switch (modo_operacao){
            case NORMAL:
                // No modo normal não faz nada, pois na atividade é especificado que o buzzer deve ficar sem som
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
            case ALERTA:
                // No modo alerta o buzzer emite um sinal a cada 100ms
                pwm_set_gpio_level(GPIO_PIN_BUZZER,wrap/2);
                xTaskDelayUntil(&xLastWakeTime,xPeriod);
                pwm_set_gpio_level(GPIO_PIN_BUZZER,0);
                xTaskDelayUntil(&xLastWakeTime,xPeriod);
                break;
        }
    }
    
}

// Task que controla a leitura do joystick
void vJoystickTask(void *params){
    // Inicializa o adc e os pinos responsáveis pelo eixo x e y do joystick
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    // Para armazenar os valores lidos da fila
    joystick_data_t joydata;

    while (true){
        // ja estou armazenando na fila a porcentagem em relação ao valor lido
        // 0% == 0 lido pelo adc
        // 100% == 4095
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos_volume_chuvas_porcentagem = adc_read() * 100 / 4095; //  faz a conversão para porcentagem 0 a 99%

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos_nivel_agua_rios_porcentagem = adc_read() * 100 / 4095; //  faz a conversão para porcentagem 0 a 99%

        xQueueSend(xQueueDisplay, &joydata, 0); // Envia o valor do joystick para a fila do display
        xQueueSend(xQueueModoOperacao, &joydata, 0); // Envia o valor do joystick para a fila da task que altera os modos de operação
        xQueueSend(xQueueLeds, &joydata, 0); // Envia o valor do joystick para a fila dos leds
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}


// Task que controla o diplay oled
void vDisplayTask(void *params){
    // Inicializa a conexao i2c e gpios necessárias
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o display oled sem nada desenhado
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd,false);
    ssd1306_send_data(&ssd);

    // Para armazenar os valores lidos da fila
    joystick_data_t joydata;
    // Para armazenar a conversão da porcentagem de inteiro para char
    char porcentagem_agua[4];
    char porcentagem_chuva[4];

    while (true){
        // Se houver dados na fila
        if (xQueueReceive(xQueueDisplay, &joydata, portMAX_DELAY) == pdTRUE){
            // Faça a conversão do valor lido em int para char
            int_to_str(joydata.x_pos_nivel_agua_rios_porcentagem,porcentagem_agua);
            int_to_str(joydata.y_pos_volume_chuvas_porcentagem,porcentagem_chuva);
            switch (modo_operacao){
                case NORMAL:
                    // No modo normal so é mostrado continuamente o nivel de água e volume de chuvas
                    ssd1306_rect(&ssd,3,3,122,60,false,true);
                    ssd1306_rect(&ssd,4,4,120,58,false,true);
                    ssd1306_draw_string(&ssd,"NIVEL DE AGUA",10,10);
                    ssd1306_draw_string(&ssd,"VOLUME DE CHUVA",6,40);
                    ssd1306_draw_string(&ssd,porcentagem_agua,50,20);
                    ssd1306_draw_string(&ssd,porcentagem_chuva,50,50);
                    ssd1306_send_data(&ssd);
                    break;
                case ALERTA:
                    // Desenha uma borda no display para chamar atenção, além de mostrar uma mensagem de nivel  ou volume elevado baseado nos valores lidos
                    if (joydata.x_pos_nivel_agua_rios_porcentagem > 70){
                        ssd1306_draw_string(&ssd,"NIVEL ELEVADO",10,10);
                    }
                    else{
                        ssd1306_draw_string(&ssd,"NIVEL DE AGUA",10,10);
                    }
                    if (joydata.y_pos_volume_chuvas_porcentagem > 70){
                        ssd1306_draw_string(&ssd,"VOLUME ELEVADO ",7,40);
                    }
                    else{
                        ssd1306_draw_string(&ssd,"VOLUME DE CHUVA",6,40);
                    }
                    ssd1306_draw_string(&ssd,porcentagem_agua,50,20);
                    ssd1306_draw_string(&ssd,porcentagem_chuva,50,50);
                    ssd1306_rect(&ssd,3,3,122,60,true,false);
                    ssd1306_rect(&ssd,4,4,120,58,true,false);
                    ssd1306_send_data(&ssd);
                    break;
            }
        }
    }
}


// Task para controlar os modos de operação do sistma
void vControlaModos(void * params){
    // Para armazenar os valores lidos da vila
    joystick_data_t joydata;
    while (true){
        // Se houver dados para ler na fila
        if (xQueueReceive(xQueueModoOperacao,&joydata,portMAX_DELAY) == pdTRUE){
            // Se o valor de x ou y(nivel da agua ou volume das chuvas) for maior que 70 altere o modo de operação para alerta
            // Caso contrário deixe no modo normal
            if (joydata.x_pos_nivel_agua_rios_porcentagem > 70 || joydata.y_pos_volume_chuvas_porcentagem > 70){
                modo_operacao = ALERTA;
            }
            else{
                modo_operacao = NORMAL;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    
}


int main(){
    // Ativa BOOTSEL via botão
    gpio_init(BOTAO_A_PARA_BOOTSEL);
    gpio_set_dir(BOTAO_A_PARA_BOOTSEL, GPIO_IN);
    gpio_pull_up(BOTAO_A_PARA_BOOTSEL);
    gpio_set_irq_enabled_with_callback(BOTAO_A_PARA_BOOTSEL, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    // Cria as filas para compartilhamento de valor do joystick
    xQueueLeds = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueDisplay = xQueueCreate(5, sizeof(joystick_data_t));
    xQueueModoOperacao = xQueueCreate(5, sizeof(joystick_data_t));
    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    xTaskCreate(vEmiteSinalBuzzerTask, "Sinal de alerta Buzzer", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    xTaskCreate(vMatrizLedELedRgbTask, "Alertas visuais emitidos pela matriz e led Rgb", configMINIMAL_STACK_SIZE *  2, NULL, 1, NULL);
    xTaskCreate(vControlaModos, "Controla se a tasks vão estar no modo normal ou alerta",configMINIMAL_STACK_SIZE * 2, NULL,1, NULL);
    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}
