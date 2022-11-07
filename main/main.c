#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <esp_system.h>
#include <math.h>

#include <mpu6050.c>
#include "ds18b20.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

#define PAGINA_PIN 2                        // botao de mudar pagina está no GPI O2
#define RESET_PIN 4                         // botao de reset está no GPI O4
#define TEMP_PIN 23                         // Sensor de temperatua está no GPIO 23
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y)) // função calulo Valor MAXIMO
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y)) // função calulo Valor MINIMO

float calcRMS(float *, int); // Função valor RMS

DeviceAddress tempSensors[2];

void app_main(void)
{
    int ON = 1; // variavel que muda de estado quando botao PAGE é pressionado
    int cont = 0;
    int i = 0;

    float Temperatura[100];                    // Temperatura é o valor da temperatura obtido no sensor
    float TemperaturaMAX = ds18b20_get_temp(); // TemperaturaMAX é o valor máximo da temperatura obtido no sensor
    float TemperaturaMIN;                      // TemperaturaMIN é o valor mínimo da temperatura obtido no sensor
    char temperatura[13];                      // char temperatura (o display so le char)
    char temperaturaMAX[13];                   // char da temperatura MAX
    char temperaturaMIN[13];                   // char da temperatura MIN
    char charRMS[14];

    float AcelX = 1;     // AcelX é o valor do acelerometo no eixo X obtido no sensor (valor teste)
    float AcelY = 2;     // AcelY é o valor do acelerometo no eixo Y obtido no sensor (valor teste)
    float AcelZ = 3;     // AcelZ é o valor do acelerometo no eixo Z obtido no sensor (valor teste)
    float RMS_Acel = 4;  // (valor teste)
    char char_AcelX[17]; // char aceleração eixo X
    char char_AcelY[17]; // char aceleração eixo Y
    char char_AcelZ[17]; // char aceleração eixo Z
    char charRMSAcel[17];

    gpio_set_direction(RESET_PIN, GPIO_MODE_INPUT);  // botao RESET, setado como output
    gpio_set_direction(PAGINA_PIN, GPIO_MODE_INPUT); // botao PAGINA, setado como output

    SSD1306_t dev;                                                              // address of the SSD1306_t structure
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO); // iniciar o barramento I2C
    ssd1306_init(&dev, 128, 64);                                                // iniciar o display OLED.  This takes in three parameters. The first is the address of the SSD1306_t structure, the second parameter is the width and the third parameter is the height of the display in pixels.
    ssd1306_clear_screen(&dev, false);                                          // Clear do display.        This takes in two parameters. The first is the address of the SSD1306_t structure and the second parameter is invert which is a bool variable.
    ssd1306_contrast(&dev, 0xff);                                               // Configura o contraste do display OLED
    ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);                        // Exibe "HELLO" grande por 3 seg

    ds18b20_init(TEMP_PIN);                    // iniciar o sensor de Temperatura (GPIO 23)
    ds18b20_setResolution(tempSensors, 2, 10); // resolução do sensor

    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay de 3 segundos
    ssd1306_clear_screen(&dev, false);     // Clear do display

    Temperatura[cont] = ds18b20_get_temp();
    TemperaturaMIN = ds18b20_get_temp();

    while (1) // O que fica aqui, entra em loop
    {
        if (gpio_get_level(RESET_PIN) == 1) // Se o botao de reset for pressionado, zera os valores de MAX e MIN
        {
            TemperaturaMAX = ds18b20_get_temp();
            TemperaturaMIN = ds18b20_get_temp();
            printf("RESET PRESSIONADO \n");    // printf teste
            ssd1306_clear_screen(&dev, false); // Clear do display.
        }
        if (gpio_get_level(PAGINA_PIN) == 1) // Se o botao de reset for pressionado, zera os valores de MAX e MIN
        {
            printf("PAGE PRESSIONADO \n");     // printf teste
            ON = !ON;                          // muda estado de ON
            ssd1306_clear_screen(&dev, false); // Clear do display.
        }

        Temperatura[cont] = ds18b20_get_temp();
        TemperaturaMAX = MAX(Temperatura[cont], TemperaturaMAX); // Temperatura maxima é o valor retornado da função MAX
        TemperaturaMIN = MIN(Temperatura[cont], TemperaturaMIN); // Temperatura minima é o valor retornado da função MIN

        sprintf(temperatura, "     %.2f C", Temperatura[cont]); // printf pa salvar o valor da temperatura no char
        sprintf(temperaturaMAX, "MAX  %.2f C", TemperaturaMAX); // printf pa salvar o valor da temperaturaMAX no char
        sprintf(temperaturaMIN, "MIN  %.2f C", TemperaturaMIN); // printf pa salvar o valor da temperaturaMIN no char
        sprintf(charRMS, "RMS  %.2f", calcRMS(Temperatura, cont));

        sprintf(char_AcelX, "Eixo X  %.2fm/s", AcelX); // printf pa salvar o valor da aceleração do eixo X no char
        sprintf(char_AcelY, "Eixo Y  %.2fm/s", AcelY); // printf pa salvar o valor da aceleração do eixo Y no char
        sprintf(char_AcelZ, "Eixo Z  %.2fm/s", AcelZ); // printf pa salvar o valor da aceleração do eixo Z char
        sprintf(charRMSAcel, "RMS  %.2f", RMS_Acel);

        if (ON == 1) // Se o botao 2 for pressionado, exibe em tela os dados da temperatura
        {
            ssd1306_display_text(&dev, 0, "  Temperatura", 16, false); // Exibe no display "Temperatura"
            ssd1306_display_text(&dev, 1, temperatura, 12, false);     // Exibe no display o valor da temperatura
            ssd1306_display_text(&dev, 3, charRMS, 10, false);         // Exibe no display "Max Temp"
            ssd1306_display_text(&dev, 5, temperaturaMAX, 13, false);  // Aqui é reservado pra exibir a temperatura maxima
            ssd1306_display_text(&dev, 7, temperaturaMIN, 13, false);  // Aqui é reservado pra exibir a temperatura mínima
        }
        else // quando ON mudar de estado, exibe os dados da acelerometro
        {
            ssd1306_display_text(&dev, 0, "  ACELEROMETRO", 16, false); // Exibe no display "ACELEROMETRO"
            ssd1306_display_text(&dev, 1, charRMSAcel, 10, false);      // Exibe no display o valor RMS
            ssd1306_display_text(&dev, 3, char_AcelX, 15, false);       // Exibe no display Aceleração no eixo X
            ssd1306_display_text(&dev, 5, char_AcelY, 15, false);       // Exibe no display Aceleração no eixo Y
            ssd1306_display_text(&dev, 7, char_AcelZ, 15, false);       // Exibe no display Aceleração no eixo Z
        }

        if (cont == 20)
        {
            // for (i = 0; i < cont; i++)
            //{
            //    Temperatura[i] = 0;
            //}
            cont = 1;
        }

        printf("cont: %i\n", cont);                                // aqui exibe a temperatura no terminal, só pra testes
        printf("TemperaturaATUAL: %0.2f C\n", ds18b20_get_temp()); // aqui exibe a temperatura no terminal, só pra testes
        printf("TemperaturaMAX: %0.2f C\n", TemperaturaMAX);       // aqui exibe a temperatura no terminal, só pra testes
        printf("TemperaturaMIN: %0.2f C\n", TemperaturaMIN);       // aqui exibe a temperatura no terminal, só pra testes
        vTaskDelay(50);                                            // delay de 500ms

        cont++;
    }
}

float calcRMS(float *Temperatura, int cont)
{ // Renomear valores10s pelo vetor
    float rms, soma = 0, quantidade = 20;
    int i;

    for (i = 0; i < cont; i++)
    {
        soma = (powf(Temperatura[i], 2)) + soma;
    }

    rms = sqrtf(soma / cont);

    return rms;
}
