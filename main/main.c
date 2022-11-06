#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <esp_system.h>

#include <mpu6050.c>
#include "ds18b20.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

// Temp Sensors are on GPIO23
#define TEMP_BUS 23

DeviceAddress tempSensors[2];

void app_main(void)
{
    SSD1306_t dev;                                                              // address of the SSD1306_t structure
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO); // iniciar o barramento I2C
    ssd1306_init(&dev, 128, 64);                                                // iniciar o display OLED.  This takes in three parameters. The first is the address of the SSD1306_t structure, the second parameter is the width and the third parameter is the height of the display in pixels.
    ssd1306_clear_screen(&dev, false);                                          // Clear do display.        This takes in two parameters. The first is the address of the SSD1306_t structure and the second parameter is invert which is a bool variable.
    ssd1306_contrast(&dev, 0xff);                                               // Configura o contraste do display OLED
    ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);                        // Exibe "HELLO" grande por 3 seg

    ds18b20_init(TEMP_BUS);                    // iniciar o sensor de Temperatura (GPIO 23)
    ds18b20_setResolution(tempSensors, 2, 10); // resolução do sensor

    vTaskDelay(3000 / portTICK_PERIOD_MS); // delay de 3 segundos
    ssd1306_clear_screen(&dev, false);     // Clear do display

    while (1) // O que fica aqui, entra em loop
    {

        float Temperatura = ds18b20_get_temp();       // Temperatura é o valor da temperatura obtido no sensor
        char temperatura[12];                         // char temperatura (o display so le char)
        sprintf(temperatura, " %.2f C", Temperatura); // printf pa salvar o valor da temperatua no char

        ssd1306_display_text(&dev, 0, "Temperatura", 11, false); // Exibe no display "Temperatura"
        ssd1306_display_text(&dev, 1, temperatura, 8, false);    // Exibe no display o valor da temperatura
        ssd1306_display_text(&dev, 3, "Max Temp.", 9, false);    // Exibe no display "Max Temp"
        ssd1306_display_text(&dev, 4, temperatura, 8, false);    // Aqui é reservado pra exibir a temperatura maxima
        ssd1306_display_text(&dev, 6, "Min Temp.", 9, false);    // Exibe no display "Min Temp"
        ssd1306_display_text(&dev, 1, temperatura, 8, false);    // Aqui é reservado pra exibir a temperatura mínima

        printf("Temperature: %0.2f C\n", ds18b20_get_temp()); // aqui exibe a temperatura no terminal, só pra testes
        vTaskDelay(500 / portTICK_PERIOD_MS);                 // delay de 500ms
    }
}
