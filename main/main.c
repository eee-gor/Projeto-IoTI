#include <stdio.h>
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "maintask.h"

#define ESP_WIFI_SSID "connect-izaias" // Nome do WIFI
#define ESP_WIFI_PASS "244466666"      // Senha do WIFI
#define MAX_RETRY 10
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "MQTT_EXAMPLE";

uint32_t MQTT_CONNECTED = 0; // 0 quando MQTT nao conectar, 1 quando sim

float Temperatura[100];  // Temperatura é o valor da temperatura obtido no sensor
float TemperaturaMAX;    // TemperaturaMAX é o valor máximo da temperatura obtido no sensor
float TemperaturaMIN;    // TemperaturaMIN é o valor mínimo da temperatura obtido no sensor
char temperatura[16];    // char temperatura (o display so le char)
char temperaturaMAX[16]; // char da temperatura MAX
char temperaturaMIN[16]; // char da temperatura MIN
char charRMS[16];
short accel_x;
short accel_y;
short accel_z;
char char_accel_x[12]; // char aceleração eixo X
char char_accel_y[12]; // char aceleração eixo Y
char char_accel_z[12]; // char aceleração eixo Z
float RMSX[1000];
float RMSY[1000];
float RMSZ[1000];
char charRMSAcelX[17];
char charRMSAcelY[17];
char charRMSAcelZ[17];

static short ON = 0; // variavel que muda de estado quando botao PAGE é pressionado

static EventGroupHandle_t s_wifi_event_group;
// eventos do WIFI
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    static int retry_cnt = 0;
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "disconnected: Retrying Wi-Fi\n");
        if (retry_cnt++ < MAX_RETRY)
        {
            esp_wifi_connect();
        }
        else
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        ESP_LOGI(TAG, "Max Retry Failed: Wi-Fi Connection\n");
        break;

    case IP_EVENT_STA_GOT_IP:
        // ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        // ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retry_cnt = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "got ip: startibg MQTT Client\n");
        mqtt_app_start();
        break;
    default:
        break;
    }
}
// Inicia WIFI, chamada na main
void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "FIM DO WIFI INIT\n");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

esp_mqtt_client_handle_t client = NULL;
// Inicia MQTT
static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "STARTING MQTT");
    //  parâmetros de conexão do broker
    esp_mqtt_client_config_t mqttConfig = {
        .uri = "mqtt://broker.emqx.io:1883"};

    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_start(client);
    MQTT_CONNECTED = 1;
    xTaskCreate(Publisher_Task, "Publisher_Task", 2048 * 5, NULL, 5, NULL);
    ESP_LOGI(TAG, "MQTT Publisher_Task is up and running\n");
}
// Função Publisher do MQTT
void Publisher_Task(void *pvParameter)
{
    while (1)
    {
        if (MQTT_CONNECTED)
        {
            esp_mqtt_client_publish(client, "grupo10/temperatura", temperatura, 0, 0, 0); // publica temperatura no topico grupoX/temperatura
            esp_mqtt_client_publish(client, "grupo10/rmstemperatura", charRMS, 0, 0, 0);  // publica RMS temperatura no topico grupoX/rmstemperatura
            esp_mqtt_client_publish(client, "grupo10/RMSX", charRMSAcelX, 0, 0, 0);       // publica temperatura no topico grupoX/temperatura
            esp_mqtt_client_publish(client, "grupo10/RMSY", charRMSAcelY, 0, 0, 0);       // publica RMS temperatura no topico grupoX/rmstemperatura
            esp_mqtt_client_publish(client, "grupo10/RMSZ", charRMSAcelZ, 0, 0, 0);       // publica temperatura no topico grupoX/temperatura

            vTaskDelay(10000 / portTICK_PERIOD_MS); // 10 seg
        }
    }
}
// Função de aquisição de dados do termometro
void Task_Temperatura(void *pvParameter)
{
    //  Configuração Sensor Temperatura
    DeviceAddress tempSensors[2];
    ds18b20_init(TEMP_PIN);                    // iniciar o sensor de Temperatura (GPIO 23)
    ds18b20_setResolution(tempSensors, 2, 10); // resolução do sensor
    TemperaturaMIN = ds18b20_get_temp();
    int cont = 0; // contador
    while (1)
    {
        Temperatura[cont] = ds18b20_get_temp();
        TemperaturaMAX = MAX(Temperatura[cont], TemperaturaMAX);   // Temperatura maxima é o valor retornado da função MAX
        TemperaturaMIN = MIN(Temperatura[cont], TemperaturaMIN);   // Temperatura minima é o valor retornado da função MIN
        sprintf(temperatura, "     %.2f C   ", Temperatura[cont]); // printf pa salvar o valor da temperatura no char
        sprintf(temperaturaMAX, "MAX  %.2f C   ", TemperaturaMAX); // printf pa salvar o valor da temperaturaMAX no char
        sprintf(temperaturaMIN, "MIN  %.2f C   ", TemperaturaMIN); // printf pa salvar o valor da temperaturaMIN no char
        sprintf(charRMS, "RMS  %.2f    ", calcRMS(Temperatura, cont));
        cont++;
        if (cont >= 100)
            cont = 1;
        // vTaskDelay(100 / portTICK_PERIOD_MS); // 1 seg
    }
}
// Função de aquisição de dados do ecelerometro
void Task_acel(void *pvParameter)
{
    int contA = 1;

    // Dados do acelerometro
    i2c_cmd_handle_t cmd;
    uint8_t data[8];
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

        sprintf(char_accel_x, "X  %.3f g ", (accel_x / 16384.0)); // printf pa salvar o valor da aceleração do eixo X no char
        sprintf(char_accel_y, "Y  %.3f g ", (accel_y / 16384.0)); // printf pa salvar o valor da aceleração do eixo Y no char
        sprintf(char_accel_z, "Z  %.3f g ", (accel_z / 16384.0)); // printf pa salvar o valor da aceleração do eixo Z char

        RMSX[contA] = (accel_x / 16384.0);
        RMSY[contA] = (accel_y / 16384.0);
        RMSZ[contA] = (accel_z / 16384.0);
        contA++;
        sprintf(charRMSAcelX, "RMS X  %.3f", calcRMS(RMSX, contA));
        sprintf(charRMSAcelY, "RMS Y  %.3f", calcRMS(RMSY, contA));
        sprintf(charRMSAcelZ, "RMS Z  %.3f", calcRMS(RMSZ, contA));
        if (contA >= 1000)
            contA = 1;
        vTaskDelay(100 / portTICK_PERIOD_MS); // 0.1 seg
    }
}
// Função de exibir dados no DISPLAY
SSD1306_t dev; // address of the SSD1306_t structure
void Task_DISPLAY(void *pvParameter)
{
    // vTaskDelay(100 / portTICK_PERIOD_MS); // 0.1 seg
    while (1)
    {
        if (ON == 0) // Se o botao 2 for pressionado, exibe em tela os dados da temperatura
        {
            ssd1306_display_text(&dev, 0, "  Temperatura", 16, false); // Exibe no display "Temperatura"
            ssd1306_display_text(&dev, 1, temperatura, 10, false);     // Exibe no display o valor da temperatura
            ssd1306_display_text(&dev, 3, charRMS, 12, false);         // Exibe no display "Max Temp"
            ssd1306_display_text(&dev, 5, temperaturaMAX, 13, false);  // Aqui é reservado pra exibir a temperatura maxima
            ssd1306_display_text(&dev, 7, temperaturaMIN, 13, false);  // Aqui é reservado pra exibir a temperatura mínima
            vTaskDelay(100 / portTICK_PERIOD_MS);                      // 0.1 seg
        }

        if (ON == 1) // quando ON mudar de estado, exibe os dados da acelerometro
        {
            ssd1306_display_text(&dev, 0, "  ACELEROMETRO", 16, false);   // Exibe no display "ACELEROMETRO"
            ssd1306_display_text(&dev, 1, "                ", 16, false); // Exibe no display "ACELEROMETRO"
            ssd1306_display_text(&dev, 3, char_accel_x, 12, false);       // Exibe no display Aceleração no eixo X
            ssd1306_display_text(&dev, 5, char_accel_y, 12, false);       // Exibe no display Aceleração no eixo Y
            ssd1306_display_text(&dev, 7, char_accel_z, 12, false);       // Exibe no display Aceleração no eixo Z
        }
        if (ON == 2) // quando ON mudar de estado, exibe os dados da acelerometro
        {
            ssd1306_display_text(&dev, 0, "  ACELEROMETRO", 16, false); // Exibe no display "ACELEROMETRO"
            ssd1306_display_text(&dev, 3, charRMSAcelX, 12, false);     // Exibe no display Aceleração no eixo X
            ssd1306_display_text(&dev, 5, charRMSAcelY, 12, false);     // Exibe no display Aceleração no eixo Y
            ssd1306_display_text(&dev, 7, charRMSAcelZ, 12, false);     // Exibe no display Aceleração no eixo Z
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // 0.1 seg
    }
}
// Função de mudar pagina automaticamente
void Task_PAGINA(void *pvParameter)
{
    while (1)
    {
        switch (ON)
        {
        case (0):
            vTaskDelay(6000 / portTICK_PERIOD_MS); // 6 seg
            ON++;
            printf("Mudou pagina\n");
            break;

        case (1):
            vTaskDelay(6000 / portTICK_PERIOD_MS); // 6 seg
            ON++;
            printf("Mudou pagina\n");
            break;
        default:
            vTaskDelay(6000 / portTICK_PERIOD_MS); //  seg
            ON = 0;
            printf("Mudou pagina\n");
            break;
        }
    }
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
            esp_deep_sleep_start();
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init(); // Inicia WIFI

    //   Configuração Display
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false); // Clear do display
    ssd1306_contrast(&dev, 0xFF);

    // Configuração GPIO e Interrupções
    gpio_set_direction(RESET_PIN, GPIO_MODE_INPUT);   // botao RESET, setado como input
    gpio_set_direction(PAGINA_PIN, GPIO_MODE_INPUT);  // botao PAGINA, setado como input
    gpio_set_direction(SLEEP_PIN, GPIO_MODE_INPUT);   // botao SLEEP, setado como input
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // GPIO_NUM_2, setado como output (LED)
    gpio_set_level(GPIO_NUM_2, 1);                    // LED azul serve como LED de energização

    esp_sleep_enable_ext0_wakeup(RESET_PIN, 1); // WAKEUP do ESP (Botão de RESET inicia novamente)

    interruptQueueRESET = xQueueCreate(10, sizeof(int));  // Queue Interrupção botao RESET
    interruptQueuePAGINA = xQueueCreate(10, sizeof(int)); // QueueInterrupção botao PAGINA
    interruptQueueSLEEP = xQueueCreate(10, sizeof(int));  // QueueInterrupção botao SLEEP MODE

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

    //  Tasks (Acontecem de forma simultanea)
    xTaskCreate(Task_BotaoRESET, "Task RESET", 2048, NULL, 5, NULL);
    xTaskCreate(Task_BotaoPAGINA, "Task PAGINA", 2048, NULL, 5, NULL);
    xTaskCreate(Task_BotaoSLEEP, "Task SLEEP", 2048, NULL, 5, NULL);
    xTaskCreate(mpu6050_config, "mpu6050_config", 2048, NULL, 5, NULL);
    xTaskCreate(Task_acel, "Task_acel", 10240, NULL, 25, NULL);
    xTaskCreate(Task_Temperatura, "Task_Temperatura", 2048, NULL, 25, NULL);
    xTaskCreate(Task_DISPLAY, "Task DISPLAY", 10240, NULL, 25, NULL);
    xTaskCreate(Task_PAGINA, "Task PAGINA2", 4096, NULL, 23, NULL);
}
