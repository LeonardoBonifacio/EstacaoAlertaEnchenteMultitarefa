# Projeto: Estação Alerta de enchente multitarefa

Este projeto implementa um sistema embarcado utilizando o Raspberry Pi Pico (RP2040) com FreeRTOS na placa BitDogLab. O objetivo é ler valores de nível de água e volume de chuva a partir de um joystick analógico, exibir porcentagens em um display OLED SSD1306, e sinalizar alertas através de buzzer e matriz de LEDs WS2812.

## Funcionalidades

* **Leitura de joystick**: Conversão dos valores brutos de ADC em porcentagem (0–100%).
* **Display OLED (SSD1306)**:

  * Modo Normal: Exibe continuamente nível de água e volume de chuva.
  * Modo Alerta: Destaca mensagens de "Nível Elevado" ou "Volume Elevado" e contorna o display.
* **Matriz de LEDs WS2812**:

  * Barra de LEDs vertical que representa faixas de 20%.
  * Mudança de cor (azul em Normal, vermelho em Alerta).
* **Buzzer**:

  * Sem som em modo Normal.
  * Emite beep periódico (100 ms ON/OFF) em modo Alerta.
* **Modo de operação**:

  * NORMAL: valores abaixo dos limiares (nível ≤70% e chuva ≤80%).
  * ALERTA: quando nível >70% ou chuva >80%.
* **Botão BOOTSEL**: Pressione o botão A da placa BitDogLab para entrar em modo de boot (Bootsel) e programar via USB.

## Hardware Utilizado

* Raspberry Pi Pico (RP2040)
* Placa BitDogLab
* Display OLED SSD1306 (I2C)
* Joystick analógico (eixos X=GPIO26, Y=GPIO27)
* Buzzer (GPIO21, PWM)
* Matriz de LEDs WS2812 (GPIO7, PIO)
* Botão A (BOOTSEL) conectado em GPIO5

## Conexões (Pinagem)

| Dispositivo        | Placa RP2040 (BitDogLab) | Função                  |
| ------------------ | ------------------------ | ----------------------- |
| Joystick X         | GP26                     | ADC0                    |
| Joystick Y         | GP27                     | ADC1                    |
| Display OLED SDA   | GP14                     | I2C SDA                 |
| Display OLED SCL   | GP15                     | I2C SCL                 |
| Buzzer             | GP21                     | PWM                     |
| Matriz LEDs WS2812 | GP7                      | PIO State Machine 0     |
| LED RGB Vermelho   | GP13                     | Saída digital           |
| LED RGB Azul       | GP12                     | Saída digital           |
| Botão BOOTSEL (A)  | GP5                      | Entr. com pull-up e IRQ |

## Software

* SDK Raspberry Pi Pico C/C++
* FreeRTOS (incluso após intalar o kernel e apontar o caminho até ele dentro do cmakelist)
* Biblioteca `ssd1306` para OLED
* Programa PIO `ws2812.pio` (gerado em tempo de compilação)

## Estrutura de Código

* **main.c**: Configuração das filas e criação das *tasks*.
* **vJoystickTask**: Leitura de ADC e envio de porcentagens.
* **vDisplayTask**: Atualização do display OLED.
* **vMatrizLedELedRgbTask**: Controle da matriz WS2812 e LEDs RGB.
* **vEmiteSinalBuzzerTask**: Emissão de sinal sonoro no modo Alerta.
* **vControlaModos**: Verificação de limiares e mudança de modo (NORMAL/ALERTA).
* **int\_to\_str**: Converte inteiro (0–99) em string com `%`.


## Uso

1. Conecte a placa via USB.
2. Pressione o botão A para entrar em modo BOOTSEL e programe o firmware.
3. Remova e reconecte normalmente.
4. Gire o joystick e observe as porcentagens no display e as barras de LEDs.
5. Se o nível ou volume ultrapassar o limiar, o sistema entra em modo Alerta.

## Ajustes e Calibração

* Os valores mínimos e máximos do joystick podem variar. Ajuste no código os comentários:

  * Valor mínimo lido: \~24
  * Valor máximo lido: \~4088
* Para alterar limiares de alerta, modifique os valores em `vControlaModos`.

