#include <stdio.h>
#include "ds18b20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

// Temp Sensors are on GPIO23
#define TEMP_BUS 23

DeviceAddress tempSensors[2];

void app_main(void)
{
    SSD1306_t dev;
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);

    ds18b20_init(TEMP_BUS);
    ds18b20_setResolution(tempSensors, 2, 10);

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ssd1306_clear_screen(&dev, false);

    while (1)
    {
        float TemperaturaC = ds18b20_get_temp();
        char temperatura[12];
        sprintf(temperatura, "%.2f C", TemperaturaC);

        ssd1306_display_text(&dev, 0, "Temperatura", 11, false);
        ssd1306_display_text(&dev, 1, temperatura, 8, false);
        ssd1306_display_text(&dev, 3, "Max Temp.", 9, false);
        ssd1306_display_text(&dev, 4, temperatura, 8, false);
        ssd1306_display_text(&dev, 6, "Min Temp.", 9, false);
        // float cTemp = ds18b20_get_temp();
        printf("Temperature: %0.2f C\n", ds18b20_get_temp());
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
