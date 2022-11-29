#include <stdio.h>
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_system.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "maintask.h"

static const char *TAG = "MQTT_EXAMPLE";
#define ESP_WIFI_SSID "connect-izaias" // Nome do WIFI
#define ESP_WIFI_PASS "244466666"      // Senha do WIFI
uint32_t MQTT_CONNECTED = 0;           // 0 quando MQTT nao conectar, 1 quando sim

// eventos do WIFI
static esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Wi-Fi connected\n");
        break;

    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip: startibg MQTT Client\n");
        mqtt_app_start();
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "disconnected: Retrying Wi-Fi\n");
        esp_wifi_connect();

        break;

    default:
        break;
    }
    return ESP_OK;
}
// Inicia WIFI, chamada na main
void wifi_init(void)
{
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}
// eventos do MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    // ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    // esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        // ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        MQTT_CONNECTED = 1;
        break;

    case MQTT_EVENT_DISCONNECTED:
        // ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNECTED = 0;
        break;

    // case MQTT_EVENT_ERROR:
    //  ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    // break;
    default:
        // ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
// Inicia MQTT
esp_mqtt_client_handle_t client = NULL;
static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "STARTING MQTT");
    // parâmetros de conexão do broker
    esp_mqtt_client_config_t mqttConfig = {
        .uri = "mqtt://broker.emqx.io:1883"};

    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

float Temperatura[100];  // Temperatura é o valor da temperatura obtido no sensor
float TemperaturaMAX;    // TemperaturaMAX é o valor máximo da temperatura obtido no sensor
float TemperaturaMIN;    // TemperaturaMIN é o valor mínimo da temperatura obtido no sensor
char temperatura[16];    // char temperatura (o display so le char)
char temperaturaMAX[16]; // char da temperatura MAX
char temperaturaMIN[16]; // char da temperatura MIN
char charRMS[16];

int cont = 1; // contador

static bool ON = 1; // variavel que muda de estado quando botao PAGE é pressionado

SSD1306_t dev;     // address of the SSD1306_t structure
void Display(void) // Função que inicia o Display
{
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO); // iniciar o barramento I2C
    ssd1306_init(&dev, 128, 64);                                                // iniciar o display OLED.  This takes in three parameters. The first is the address of the SSD1306_t structure, the second parameter is the width and the third parameter is the height of the display in pixels.
    ssd1306_clear_screen(&dev, false);                                          // Clear do display.        This takes in two parameters. The first is the address of the SSD1306_t structure and the second parameter is invert which is a bool variable.
    ssd1306_contrast(&dev, 0xff);                                               // Configura o contraste do display OLED
    ssd1306_display_text_x3(&dev, 3, " IOT", 4, false);                         // Exibe "IOT" grande por 3 seg
    vTaskDelay(2000 / portTICK_PERIOD_MS);                                      // delay de 2 segundos
    ssd1306_clear_screen(&dev, false);                                          // Clear do display
}

// Interrupçoes dos Botoes
xQueueHandle interruptQueueRESET;
xQueueHandle interruptQueuePAGINA;
xQueueHandle interruptQueueSLEEP;

static void IRAM_ATTR gpio_interrupt_reset(void *args) // Botão de RESET parametros MAX e MIN
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueueRESET, &pinNumber, NULL);
}
void Task_BotaoRESET(void *params)
{
    int pinNumber;
    while (true)
    {
        if (xQueueReceive(interruptQueueRESET, &pinNumber, portMAX_DELAY))
        {
            printf("RESET PRESSIONADO %d\n", pinNumber);
            TemperaturaMAX = -100;
            TemperaturaMIN = 100;
        }
    }
}

static void IRAM_ATTR gpio_interrupt_pagina(void *args) // Botão de mudar pagina
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueuePAGINA, &pinNumber, NULL);
}
void Task_BotaoPAGINA(void *params)
{
    int pinNumber;
    while (true)
    {
        if (xQueueReceive(interruptQueuePAGINA, &pinNumber, portMAX_DELAY))
        {
            printf("PAGINA PRESSIONADO %d\n", pinNumber);
            ON = !ON;
        }
    }
}

static void IRAM_ATTR gpio_interrupt_sleep(void *args) // Botão de ativar DEEP SLEEP MODE
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueueSLEEP, &pinNumber, NULL);
}
void Task_BotaoSLEEP(void *params)
{
    int pinNumber;
    while (true)
    {
        if (xQueueReceive(interruptQueueSLEEP, &pinNumber, portMAX_DELAY)) // Limpa todas as linhas do Display e ativa Deep Sleep Mode
        {
            ssd1306_clear_line(&dev, 0, false);
            ssd1306_clear_line(&dev, 1, false);
            ssd1306_clear_line(&dev, 2, false);
            ssd1306_clear_line(&dev, 3, false);
            ssd1306_clear_line(&dev, 4, false);
            ssd1306_clear_line(&dev, 5, false);
            ssd1306_clear_line(&dev, 6, false);
            ssd1306_clear_line(&dev, 7, false);
            esp_deep_sleep_start();
        }
    }
}

// Interrupção Timer periódico
void timer_callback(void *param)
{
    Temperatura[cont] = ds18b20_get_temp();
    TemperaturaMAX = MAX(Temperatura[cont], TemperaturaMAX);   // Temperatura maxima é o valor retornado da função MAX
    TemperaturaMIN = MIN(Temperatura[cont], TemperaturaMIN);   // Temperatura minima é o valor retornado da função MIN
    sprintf(temperatura, "     %.2f C   ", Temperatura[cont]); // printf pa salvar o valor da temperatura no char
    sprintf(temperaturaMAX, "MAX  %.2f C   ", TemperaturaMAX); // printf pa salvar o valor da temperaturaMAX no char
    sprintf(temperaturaMIN, "MIN  %.2f C   ", TemperaturaMIN); // printf pa salvar o valor da temperaturaMIN no char
    sprintf(charRMS, "RMS  %.2f", calcRMS(Temperatura, cont));
    cont++;
    if (ON == 1) // Se o botao 2 for pressionado, exibe em tela os dados da temperatura
    {
        ssd1306_display_text(&dev, 0, "  Temperatura", 16, false); // Exibe no display "Temperatura"
        ssd1306_display_text(&dev, 1, temperatura, 10, false);     // Exibe no display o valor da temperatura
        ssd1306_display_text(&dev, 3, charRMS, 10, false);         // Exibe no display "Max Temp"
        ssd1306_display_text(&dev, 5, temperaturaMAX, 13, false);  // Aqui é reservado pra exibir a temperatura maxima
        ssd1306_display_text(&dev, 7, temperaturaMIN, 13, false);  // Aqui é reservado pra exibir a temperatura mínima
    }
    if (cont == 100)
        cont = 1;
}

// Função Publisher do MQTT
void Publisher_Task(void *params)
{
    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS); // 10 seg
        if (MQTT_CONNECTED)
        {
            esp_mqtt_client_publish(client, "grupo10/temperatura", charRMS, 0, 0, 0);     // publica RMS temperatura no topico grupoX/temperatura
            esp_mqtt_client_publish(client, "grupo10/rmsvibracao", temperatura, 0, 0, 0); // publica RMS vibracao no topico grupoX/rmsvibracao
        }
    }
}

void app_main(void)
{
    //  Configuração Display
    Display();

    //  Configuração Sensor Temperatura
    DeviceAddress tempSensors[2];
    ds18b20_init(TEMP_PIN);                    // iniciar o sensor de Temperatura (GPIO 23)
    ds18b20_setResolution(tempSensors, 2, 10); // resolução do sensor
    Temperatura[cont] = ds18b20_get_temp();
    TemperaturaMIN = ds18b20_get_temp();

    // Configuração GPIO e Interrupções
    gpio_set_direction(RESET_PIN, GPIO_MODE_INPUT);   // botao RESET, setado como input
    gpio_set_direction(PAGINA_PIN, GPIO_MODE_INPUT);  // botao PAGINA, setado como input
    gpio_set_direction(SLEEP_PIN, GPIO_MODE_INPUT);   // botao SLEEP, setado como input
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // GPIO_NUM_2, setado como output (LED)
    gpio_set_level(GPIO_NUM_2, 1);                    // LED azul serve como LED de energização

    esp_sleep_enable_ext0_wakeup(RESET_PIN, 1); // WAKEUP do ESP (Botão de RESET inicia novamente)

    interruptQueueRESET = xQueueCreate(1, sizeof(int));  // Queue Interrupção botao RESET
    interruptQueuePAGINA = xQueueCreate(1, sizeof(int)); // QueueInterrupção botao PAGINA
    interruptQueueSLEEP = xQueueCreate(1, sizeof(int));  // QueueInterrupção botao SLEEP MODE

    gpio_pulldown_en(RESET_PIN); // Habilita resistor PULLDOWN
    gpio_pullup_dis(RESET_PIN);
    gpio_set_intr_type(RESET_PIN, GPIO_INTR_POSEDGE); // Ativa interrupção em rising edge

    gpio_pulldown_en(PAGINA_PIN); // Habilita resistor PULLDOWN
    gpio_pullup_dis(PAGINA_PIN);
    gpio_set_intr_type(PAGINA_PIN, GPIO_INTR_POSEDGE); // Ativa interrupção em rising edge

    gpio_pulldown_en(SLEEP_PIN);                      // Habilita resistor PULLDOWN
    gpio_set_intr_type(SLEEP_PIN, GPIO_INTR_POSEDGE); // Ativa interrupção em rising edge

    gpio_install_isr_service(0);
    gpio_isr_handler_add(RESET_PIN, gpio_interrupt_reset, (void *)RESET_PIN);
    gpio_isr_handler_add(PAGINA_PIN, gpio_interrupt_pagina, (void *)PAGINA_PIN);
    gpio_isr_handler_add(SLEEP_PIN, gpio_interrupt_sleep, (void *)SLEEP_PIN);

    // Criando a interrupção do TIMER
    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback};
    esp_timer_handle_t timer_handler;
    esp_timer_create(&my_timer_args, &timer_handler);
    esp_timer_start_periodic(timer_handler, 500000); // Timer a cada 0.5s

    nvs_flash_init();
    wifi_init(); // Inicia WIFI

    // Tasks (Acontecem de forma simultanea)
    xTaskCreate(Task_BotaoRESET, "Task BotaoRESET", 2048, NULL, 1, NULL);
    xTaskCreate(Task_BotaoPAGINA, "Task BotaoPAGINA", 2048, NULL, 1, NULL);
    xTaskCreate(Task_BotaoSLEEP, "Task BotaoPAGINA", 2048, NULL, 1, NULL);

    // xTaskCreatePinnedToCore(Publisher_Task, "Publisher_Task", 2048 * 5, NULL, 0, NULL, 1);
    xTaskCreate(Publisher_Task, "Publisher_Task", 2048 * 5, NULL, 2, NULL);
    xTaskCreate(mpu6050_config, "mpu6050_config", 2048, NULL, 1, NULL);
    // xTaskCreate(task_mpu6050, "task_mpu6050", 4096, NULL, 1, NULL);

    // Dados do acelerometro
    i2c_cmd_handle_t cmd;
    uint8_t data[8];

    short accel_x;
    short accel_y;
    short accel_z;
    char char_accel_x[12]; // char aceleração eixo X
    char char_accel_y[12]; // char aceleração eixo Y
    char char_accel_z[12]; // char aceleração eixo Z
    // char charRMSAcelX[17];
    // char charRMSAcelY[17];
    // char charRMSAcelZ[17];
    while (1)
    {
        // Tell the MPU6050 to position the internal register pointer to register
        // MPU6050_ACCEL_XOUT_H.
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1);

        i2c_master_read_byte(cmd, data, 0);
        i2c_master_read_byte(cmd, data + 1, 0);
        i2c_master_read_byte(cmd, data + 2, 0);
        i2c_master_read_byte(cmd, data + 3, 0);
        i2c_master_read_byte(cmd, data + 4, 0);
        i2c_master_read_byte(cmd, data + 5, 1);

        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        accel_x = (data[0] << 8) | data[1];
        accel_y = (data[2] << 8) | data[3];
        accel_z = (data[4] << 8) | data[5];
        // printf("x: %0.3f y: %0.3f  z: %0.3f\n", accel_x / 16384.0, accel_y / 16384.0, accel_z / 16384.0);

        sprintf(char_accel_x, "X  %.3f g ", (accel_x / 16384.0)); // printf pa salvar o valor da aceleração do eixo X no char
        sprintf(char_accel_y, "Y  %.3f g ", (accel_y / 16384.0)); // printf pa salvar o valor da aceleração do eixo Y no char
        sprintf(char_accel_z, "Z  %.3f g ", (accel_z / 16384.0)); // printf pa salvar o valor da aceleração do eixo Z char
        // sprintf(charRMSAcel, "RMS  %.2f       ", RMS_Acel);
        if (ON == 0) // quando ON mudar de estado, exibe os dados da acelerometro
        {
            ssd1306_display_text(&dev, 0, "  ACELEROMETRO", 16, false); // Exibe no display "ACELEROMETRO"
            ssd1306_display_text(&dev, 3, char_accel_x, 12, false);     // Exibe no display Aceleração no eixo X
            ssd1306_display_text(&dev, 5, char_accel_y, 12, false);     // Exibe no display Aceleração no eixo Y
            ssd1306_display_text(&dev, 7, char_accel_z, 12, false);     // Exibe no display Aceleração no eixo Z
        }
    }
}
