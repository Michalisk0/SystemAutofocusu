#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "iot_servo.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "sh1106.h"


// ---------------------------------------- Definicje dla enkodera obrotowego
#define SIA_GPIO 15
#define SIB_GPIO 5
#define SW_GPIO 4
#define ESP_INTR_FLAG_DEFAULT 0

//----------------------------------------- Definicje dla serwomechanizmu
#define Servo_PWM_GPIO 32
#define Servo_idle 92
#define Servo_right 110 // Servo_idle + 18
#define Servo_left 74   // Servo_idle - 18
#define ServoStepTimeMS 20

//------------------------------------------ Definicje dla Lidaru
#define LidarI2CAddr 0x5A
#define I2C_TIMEOUT_MS 1000
#define UART_TX 17
#define UART_RX 16
// #define Data_ready_GPIO 17

// ----------------------------------------- Definicje dla OLED

#define SDA_GPIO 21
#define SCL_GPIO 22

#define tag "SH1106"

// ----------------------------------------- Definicje dla przycisków

#define homeButton_GPIO 14

// ----------------------------------------- Definicje dla pamięci ROM
#define FLASH_SECTOR_SIZE 0x1000
#define measurementsSize 16

// ------------------------------------------ Task Handle
TaskHandle_t SIA_callTaskHandle = NULL;
TaskHandle_t startupTaskHandle = NULL;
TaskHandle_t ServoTaskHandle = NULL;
TaskHandle_t lidarReadTaskHandle = NULL;
TaskHandle_t calibrationTaskHandle = NULL;
TaskHandle_t operationTaskHandle = NULL;

// ------------------------------------------ Zmienne globalne

int32_t ImpulseCurrentState = 0;
int32_t ServoPhysicalState = 0;
int16_t latestLidarValue = 0;
uint8_t measurements[measurementsSize]; // Pomiary kalibracyjne

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

// ------------------------------------ Parametry konfigracyjne UART
const uart_port_t uart_num = UART_NUM_2;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};

// -------------------------------------- Parametry konfigracyjne magistrali I2C (dla OLED)
i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA_GPIO,
    .scl_io_num = SCL_GPIO,
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    .master.clk_speed = 1000000};

// ------------------------------------ ISR przerwań
void IRAM_ATTR SIA_isr(void *arg)
{
    xTaskResumeFromISR(SIA_callTaskHandle);
}
void IRAM_ATTR homeButton_isr(void *arg)
{
    // vTaskSuspend(calibrationTaskHandle);
    // vTaskSuspend(operationTaskHandle);
    xTaskResumeFromISR(startupTaskHandle);
}

void displayMeasurements()
{
    char str[150];
    while (gpio_get_level(SW_GPIO) != 0)
    {
        task_sh1106_display_clear(NULL);
        sprintf(str, "Measurements:\n\
    50: %d\n\
    100: %d\n\
    150: %d\n\
    200: %d\n\
    250: %d\n\
    300: %d\n",
                measurements[0], measurements[1], measurements[2], measurements[3], measurements[4], measurements[5]);
        task_sh1106_display_text(str);
        vTaskDelay(4000 / portTICK_RATE_MS);
        task_sh1106_display_clear(NULL);
        sprintf(str, "Measurements:\n\
    350: %d\n\
    400: %d\n\
    450: %d\n\
    500: %d\n\
    550: %d\n\
    600: %d\n",
                measurements[6], measurements[7], measurements[8], measurements[9], measurements[10], measurements[11]);
        task_sh1106_display_text(str);
        vTaskDelay(4000 / portTICK_RATE_MS);
        task_sh1106_display_clear(NULL);
        sprintf(str, "Measurements:\n\
    650: %d\n\
    700: %d\n\
    750: %d\n\
    800: %d\n",
                measurements[12], measurements[13], measurements[14], measurements[15]);
        task_sh1106_display_text(str);
        vTaskDelay(4000 / portTICK_RATE_MS);
    }
}
void takeMeasurements()
{
    char str[150]; // String do wyświetlania na ekranie
    task_sh1106_display_clear(NULL);
    sprintf(str, "Set lens to\n\
                  minimal\n\
                  distance\n\
                  and press\n\
                  the knob\n");
    task_sh1106_display_text(str);
    while (gpio_get_level(SW_GPIO) != 0)
    {
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    vTaskDelay(1000 / portTICK_RATE_MS);
    ImpulseCurrentState = 0;
    for (int i = 0; i < 16; i++)
    {
        task_sh1106_display_clear(NULL);
        int desiredDistance = i * 100 / 2 + 50;
        while ((latestLidarValue != desiredDistance) && (gpio_get_level(SW_GPIO) != 0))
        {
            sprintf(str, "Focus on the\n\
                  object within\n\
                  %dcm distance\n\
                  Measured:\n\
                  %dcm",
                    desiredDistance, latestLidarValue);
            task_sh1106_display_text(str);
            vTaskDelay(10 / portTICK_RATE_MS);
        }

        task_sh1106_display_clear(NULL);
        vTaskDelay(40 / portTICK_RATE_MS);
        sprintf(str, "Press the\n\
                knob after\n\
                focusing on\n\
                the object\n");
        task_sh1106_display_text(str);
        vTaskDelay(1000 / portTICK_RATE_MS);
        while (gpio_get_level(SW_GPIO) != 0)
        {

            vTaskDelay(10 / portTICK_RATE_MS);
        }
        task_sh1106_display_clear(NULL);
        task_sh1106_display_text("Done!");
        vTaskDelay(1000 / portTICK_RATE_MS);
        measurements[i] = ImpulseCurrentState;
    }
    
    displayMeasurements();
}

uint8_t getServoStep(uint16_t dist)
{
    switch (dist)
    {
    case 0 ... 50:
        return (dist / 50) * measurements[0];
        break;
    case 51 ... 100:
        return measurements[0] + (dist - 50) / 50 * (measurements[1] - measurements[0]);
        break;
    case 101 ... 150:
        return measurements[1] + (dist - 100) / 50 * (measurements[2] - measurements[1]);
        break;
    case 151 ... 200:
        return measurements[2] + (dist - 150) / 50 * (measurements[3] - measurements[2]);
        break;
    case 201 ... 250:
        return measurements[3] + (dist - 200) / 50 * (measurements[4] - measurements[3]);
        break;
    case 251 ... 300:
        return measurements[4] + (dist - 250) / 50 * (measurements[5] - measurements[4]);
        break;
    case 301 ... 350:
        return measurements[5] + (dist - 300) / 50 * (measurements[6] - measurements[5]);
        break;
    case 351 ... 400:
        return measurements[6] + (dist - 350) / 50 * (measurements[7] - measurements[6]);
        break;
    case 401 ... 450:
        return measurements[7] + (dist - 400) / 50 * (measurements[8] - measurements[7]);
        break;
    case 451 ... 500:
        return measurements[8] + (dist - 450) / 50 * (measurements[9] - measurements[8]);
        break;
    case 501 ... 550:
        return measurements[9] + (dist - 500) / 50 * (measurements[10] - measurements[9]);
        break;
    case 551 ... 600:
        return measurements[10] + (dist - 550) / 50 * (measurements[11] - measurements[10]);
        break;
    case 601 ... 650:
        return measurements[11] + (dist - 600) / 50 * (measurements[12] - measurements[11]);
        break;
    case 651 ... 700:
        return measurements[12] + (dist - 650) / 50 * (measurements[13] - measurements[12]);
        break;
    case 701 ... 750:
        return measurements[13] + (dist - 700) / 50 * (measurements[14] - measurements[13]);
        break;
    case 751 ... 800:
        return measurements[14] + (dist - 750) / 50 * (measurements[15] - measurements[14]);
        break;
    default:
        return 0;
        break;
    }
}

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
            ImpulseCurrentState++;
            vTaskResume(ServoTaskHandle);
            printf("Lewo\n"); // Debug
        }
        else
        {
            ImpulseCurrentState--;
            vTaskResume(ServoTaskHandle);
            printf("Prawo\n"); // Debug
        }

        vTaskDelay(40 / portTICK_RATE_MS);
    }
}
void servoTask(void *pvParameter)
{
    //iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
    while (1)
    {

        // printf("Start ServoCurrentState: %d\n", ServoCurrentState); // Debug
        // printf("ServoPhysicalState: %d\n", ServoPhysicalState);     // Debug
        // if (ServoCurrentState != ServoPhysicalState)
        // {
        iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
        while (ServoPhysicalState > ImpulseCurrentState)
        {
            // printf("1 ImpulseCurrentState: %d\n", ImpulseCurrentState); // Debug
            // printf("ServoPhysicalState: %d\n", ImpulseCurrentState);     // Debug
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_left);
            vTaskDelay(ServoStepTimeMS / portTICK_RATE_MS);
            //iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
            // printf("In leftBuffer while\n"); // Debug
            ServoPhysicalState--;
        }
        while (ServoPhysicalState < ImpulseCurrentState)
        {
            // printf("2 ImpulseCurrentState: %d\n", ImpulseCurrentState); // Debug
            // printf("ServoPhysicalState: %d\n", ImpulseCurrentState);     // Debug
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_right);
            vTaskDelay(ServoStepTimeMS / portTICK_RATE_MS);
            //iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
            // printf("In leftBuffer while\n"); // Debug
            ServoPhysicalState++;
        }
        // }
        vTaskDelay(40 / portTICK_RATE_MS);

        // printf("Angle %f\n", angle); // Debug
        // printf("Koniec Start ImpulseCurrentState: %d\n", ImpulseCurrentState); // Debug
        // printf("ServoPhysicalState: %d\n", ServoPhysicalState);            // Debug
        iot_servo_deinit(LEDC_LOW_SPEED_MODE);
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
        // printf("%d, ", data[0]); // debug
        if (data[0] == 89)
        {
            uart_read_bytes(uart_num, data, 1, 100);
            // printf("%d, ", data[0]); // debug
            if (data[0] == 89)
            {
                uart_read_bytes(uart_num, data, 7, 100);
                latestLidarValue = data[0] + data[1] * 16;
            }
            // printf("%d\n", latestLidarValue); // Debug
            //  char str[25];
            //  sprintf(str, "Lidar reading:\n%dcm", latestLidarValue);
            //  task_sh1106_display_clear(NULL);
            //  task_sh1106_display_text(str);
            //  vTaskDelay(100 / portTICK_RATE_MS);
        }
    }
}

void startupTask()
{

    while (1)
    {
        vTaskSuspend(NULL);
        // printf("Home button pressed\n"); // Debug
    }
}

void calibrationTask()
{

    while (1)
    {
        vTaskSuspend(NULL);
    }
}

void operationTask()
{

    while (1)
    {
        vTaskSuspend(NULL);
    }
}

void app_main()
{
    ets_timer_init();

    // ------------------------------------- Konfiguracja GPIO dla impulsatora
    gpio_set_direction(SIA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SIB_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SW_GPIO, GPIO_MODE_INPUT);

    // -------------------------------------- Konfiguracja GPIO dla przycisków
    gpio_pullup_en(homeButton_GPIO);
    gpio_set_direction(homeButton_GPIO, GPIO_MODE_INPUT);

    // ------------------------------------- Konfiguracja obsługi przerwań
    gpio_set_intr_type(SIA_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(homeButton_GPIO, GPIO_INTR_NEGEDGE);

    // printf("Impulsator conf - done\n"); // Debug

    // ------------------------------------- Konfiguracja obsługi przerwania dla Odczytu danych LIDAR

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // -------------------------------------- Konfiguracja ISR
    gpio_isr_handler_add(SIA_GPIO, SIA_isr, NULL);
    gpio_isr_handler_add(homeButton_GPIO, homeButton_isr, NULL);

    // -------------------------------------- Uruchomienie komunikacji UART
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(UART_NUM_2, UART_TX, UART_RX, 18, 19);

    // -------------------------------------- Uruchomienie magistrali I2C oraz wyświetlacza OLED obługiwanego przez magistralę

    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    sh1106_init();

    task_sh1106_display_clear(NULL);
    task_sh1106_display_text("Hello!");

    // -------------------------------------- Konfiguracja GPIO dla Servo

    // ------------------------------------- Uruchomienie tasków systemowych
    xTaskCreate(&SIA_call, "SIA_call", 1024, NULL, 9, &SIA_callTaskHandle); // Task obsługuijący impulator
    xTaskCreate(&servoTask, "servoTask", 4096, NULL, 8, &ServoTaskHandle);  // Task obsługujący servo
    xTaskCreate(&lidarReadTask, "lidarReadTask", 4096, NULL, 10, &lidarReadTaskHandle); // Task obsługujący odczyt z Czujnika odległości
     xTaskCreate(&startupTask, "startupTask", 2048, NULL, 10, &startupTaskHandle);
    //  read_measurements_from_flash();
    //  displayMeasurements();
    //  takeMeasurements();
}