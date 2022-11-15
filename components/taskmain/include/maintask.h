#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include <math.h>

#include "mpu6050.h"
#include "ds18b20.h"
#include "ssd1306.h"

#define PAGINA_PIN 2 // botao de mudar pagina está no GPI O2
#define RESET_PIN 4  // botao de reset está no GPI O4
#define TEMP_PIN 23  // Sensor de temperatua está no GPIO 23

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y)) // função calulo Valor MAXIMO
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y)) // função calulo Valor MINIMO

float calcRMS(float *, int); // Função valor RMS