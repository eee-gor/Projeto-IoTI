#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "ds18b20.h"
#include "ssd1306.h"
#include <math.h>

#define RESET_PIN 12  // botao de reset está no GPIO 12
#define SLEEP_PIN 5   // botao Deep Sleep MODE está no GPIO 5
#define PAGINA_PIN 25 // botao de mudar pagina está no GPIO 25
#define TEMP_PIN 23   // Sensor de temperatua está no GPIO 23

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y)) // função calulo Valor MAXIMO
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y)) // função calulo Valor MINIMO

float calcRMS(float *, int); // Função valor RMS

int abs_val(int x); // Retorna valor absoluto

static void mqtt_app_start(void);
void Publisher_Task(void *);
