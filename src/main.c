#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "iot_servo.h"
#include "driver/i2c.h"
#include "driver/uart.h"

// ---------------------------------------- Definicje dla enkodera obrotowego
#define SIA_GPIO 15
#define SIB_GPIO 5
#define SW_GPIO 4
#define ESP_INTR_FLAG_DEFAULT 0

//----------------------------------------- Definicje dla serwomechanizmu
#define Servo_PWM_GPIO 23
#define Servo_idle 92.0f
#define Servo_right 110.0f // Servo_idle + 18
#define Servo_left 74.0f   // Servo_idle - 18
#define ServoStepTimeMS 25

//------------------------------------------ Definicje dla Lidaru
#define LidarI2CAddr 0x5A
#define I2C_TIMEOUT_MS 1000
#define UART_TX 17
#define UART_RX 16
// #define Data_ready_GPIO 17

// ------------------------------------------ Task Handle
TaskHandle_t ISR_SIA = NULL;
TaskHandle_t ServoTaskHandle = NULL;
TaskHandle_t lidarReadTaskHandle = NULL;

// ------------------------------------------ Zmienne globalne
uint16_t rightBuffer = 0, leftBuffer = 0;
ledc_timer_config_t PWM_timer;
ledc_channel_config_t PWM_channel;
int32_t ServoCurrentState = 0;
int16_t latestLidarValue = 0;

// ------------------------------------ Parametry konfigracyjne sygnału PWM sterującego serwomechanizmem

servo_config_t servo_cfg = {
    .max_angle = 180,
    .min_width_us = 800,
    .max_width_us = 2200,
    .freq = 50,
    .timer_number = LEDC_TIMER_0,
    .channels = {
        .servo_pin = {
            Servo_PWM_GPIO,
        },
        .ch = {
            LEDC_CHANNEL_0,
        },
    },
    .channel_number = 1,
};

const uart_port_t uart_num = UART_NUM_2;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};

// // -------------------------------------- Parametry konfigracyjne magistrali I2C

// i2c_config_t I2C_cfg = {
//     .mode = I2C_MODE_MASTER,
//     .sda_io_num = 21,
//     .scl_io_num = 22,
//     .sda_pullup_en = GPIO_PULLUP_DISABLE,
//     .scl_pullup_en = GPIO_PULLUP_DISABLE,
//     .master.clk_speed = 200000,
// };

// ------------------------------------ Handler przerwania dla SIA impulsatora - wznowienie wątku przerwaniem
void IRAM_ATTR SIA_isr_handler(void *arg)
{
    xTaskResumeFromISR(ISR_SIA);
}
// void IRAM_ATTR Lidar_I2C_Data_Ready_isr_handler(void *arg)
// {

//     xTaskResumeFromISR(lidarReadTaskHandle);
// }

// ------------------------------------ Task wznawiany zboczem SIA impulstora - określanie kierunku obrotu impulsatora - zasada działania w załączniku nr 4
void SIA_call(void *pvParameter)
{
    bool A1, B1;
    while (1)
    {
        vTaskSuspend(NULL);
        A1 = gpio_get_level(SIA_GPIO);
        B1 = gpio_get_level(SIB_GPIO);
        if (B1 == A1)
        {
            ServoCurrentState++;
            vTaskResume(ServoTaskHandle);
            // printf("Lewo\n"); // Debug
        }
        else
        {
            ServoCurrentState--;
            vTaskResume(ServoTaskHandle);
            // printf("Prawo\n"); // Debug
        }

        vTaskDelay(40 / portTICK_RATE_MS);
    }
}
void servoTask(void *pvParameter)
{
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
    int32_t ServoPhysicalState = 0;
    while (1)
    {
        while (ServoPhysicalState > ServoCurrentState)
        {
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_left);
            vTaskDelay(ServoStepTimeMS / portTICK_RATE_MS);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
            // printf("In rightBuffer while\n"); // Debug
            ServoPhysicalState--;
        }
        while (ServoPhysicalState < ServoCurrentState)
        {
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_right);
            vTaskDelay(ServoStepTimeMS / portTICK_RATE_MS);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
            // printf("In leftBuffer while\n"); // Debug
            ServoPhysicalState--;
        }
        // printf("Angle %f\n", angle); // Debug
        vTaskSuspend(NULL);
    }
}
void lidarReadTask()
{
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
    uint8_t data[7];

    while (1)
    {
        uart_read_bytes(uart_num, data, 1, 100);
         //printf("%d, ", data[0]); // debug
        if (data[0] == 89)
        {
            uart_read_bytes(uart_num, data, 1, 100);
            //printf("%d, ", data[0]); // debug
            if (data[0] == 89)
            {
                uart_read_bytes(uart_num, data, 7, 100);
                latestLidarValue = data[0] + data[1] * 16; 
            }
            printf("%d\n", latestLidarValue);
            // vTaskDelay(50 / portTICK_RATE_MS);
        }
        // vTaskDelay(50 / portTICK_RATE_MS);
    }
}

void app_main()
{
    ets_timer_init();

    // ------------------------------------- Konfiguracja GPIO dla impulsatora
    gpio_set_direction(SIA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SIB_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SW_GPIO, GPIO_MODE_INPUT);

    // ------------------------------------- Konfiguracja obsługi przerwania dla SIA impulsatora
    gpio_set_intr_type(SIA_GPIO, GPIO_INTR_ANYEDGE);

    // printf("Impulsator conf - done\n"); // Debug

    // ------------------------------------- Konfiguracja obsługi przerwania dla Odczytu danych LIDAR
    // gpio_set_intr_type(Data_ready_GPIO, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // gpio_isr_handler_add(Data_ready_GPIO, Lidar_I2C_Data_Ready_isr_handler, NULL);
    gpio_isr_handler_add(SIA_GPIO, SIA_isr_handler, NULL);

    // -------------------------------------- Konfiguracja GPIO dla Servo

    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(UART_NUM_2, UART_TX, UART_RX, 18, 19);

    // ------------------------------------- Uruchomienie tasków systemowych
    xTaskCreate(&SIA_call, "SIA_call", 1024, NULL, 10, &ISR_SIA);                       // Task obsługuijący impulator
    xTaskCreate(&servoTask, "servoTask", 2048, NULL, 10, &ServoTaskHandle);             // Task obsługujący servo
    xTaskCreate(&lidarReadTask, "lidarReadTask", 4096, NULL, 10, &lidarReadTaskHandle); // Task obsługujący odczyt z Czujnika odległości
}