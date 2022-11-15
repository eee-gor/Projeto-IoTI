#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include <math.h>
#include "esp_sleep.h"

#include "mpu6050.h"
#include "ds18b20.h"
#include "ssd1306.h"

#define SLEEP_PIN 5   // botao de mudar pagina está no GPI 18
#define PAGINA_PIN 18 // botao de mudar pagina está no GPI 18
#define RESET_PIN 4   // botao de reset está no GPI 19
#define TEMP_PIN 23   // Sensor de temperatua está no GPIO 23

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y)) // função calulo Valor MAXIMO
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y)) // função calulo Valor MINIMO

float calcRMS(float *, int); // Função valor RMS

int abs_val(int x); // Retorna valor absoluto
