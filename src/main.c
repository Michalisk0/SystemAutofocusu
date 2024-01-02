#include <stdio.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "iot_servo.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "sh1106.h"

//  ---------------------------------------- Definicje dla enkodera obrotowego
#define SIA_GPIO 15
#define SIB_GPIO 5
#define SW_GPIO 4
#define ESP_INTR_FLAG_DEFAULT 0

//----------------------------------------- Definicje dla serwomechanizmu
#define Servo_PWM_GPIO 32
#define Servo_idle 92
#define Servo_right 110
#define Servo_left 73
#define ServoStepTimeMS 50
#define Servo_PWM_frequency 50
#define ServoRestTimeMS 400

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
#define measurementsSize 11

// ------------------------------------------ Task Handle
TaskHandle_t SIA_callTaskHandle = NULL;
TaskHandle_t startupTaskHandle = NULL;
TaskHandle_t ServoTaskHandle = NULL;
TaskHandle_t lidarReadTaskHandle = NULL;
TaskHandle_t operationTaskHandle = NULL;

// ------------------------------------------ Zmienne globalne

int16_t ImpulseCurrentState = 0;
int16_t ServoPhysicalState = 0;
int16_t latestLidarValue = 0;
int16_t measurements[measurementsSize] = {
    5, 8, 12, 17, 24, 30, 34, 40, 46, 52, 58}; // Pomiary kalibracyjne - mockup

uint16_t calibrationDistances[] = {
    25, 50, 75, 100, 125, 175, 250, 350, 500, 650, 800}; // Pomiary kalibracyjne
// ------------------------------------ Parametry konfigracyjne sygnału PWM sterującego serwomechanizmem

servo_config_t servo_cfg = {
    .max_angle = 180,
    .min_width_us = 1000,
    .max_width_us = 2000,
    .freq = Servo_PWM_frequency,
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

// ------------------------------------ ISR
void IRAM_ATTR SIA_isr(void *arg)
{
    xTaskResumeFromISR(SIA_callTaskHandle);
}
void IRAM_ATTR homeButton_isr(void *arg)
{
    xTaskResumeFromISR(startupTaskHandle);
}

void displayMeasurements()
{
    char str[150];
    while (gpio_get_level(SW_GPIO) != 0)
    {
        task_sh1106_display_clear(NULL);
        sprintf(str, "Measurements:\n\
    %dcm: %d\n\
    %dcm: %d\n\
    %dcm: %d\n\
    %dcm: %d\n\
    %dcm: %d\n\
    %dcm: %d\n",
                calibrationDistances[0], measurements[0], calibrationDistances[1], measurements[1], calibrationDistances[2], measurements[2], calibrationDistances[3], measurements[3], calibrationDistances[4], measurements[4], calibrationDistances[5], measurements[5]);
        task_sh1106_display_text(str);
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        task_sh1106_display_clear(NULL);
        sprintf(str, "Measurements:\n\
    %dcm: %d\n\
    %dcm: %d\n\
    %dcm: %d\n\
    %dcm: %d\n\
    %dcm: %d\n",
                calibrationDistances[6], measurements[6], calibrationDistances[7], measurements[7], calibrationDistances[8], measurements[8], calibrationDistances[9], measurements[9], calibrationDistances[10], measurements[10]);
        task_sh1106_display_text(str);
        vTaskDelay(4000 / portTICK_PERIOD_MS);
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
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ImpulseCurrentState = 0;
    ServoPhysicalState = 0;
    for (int i = 0; i < measurementsSize; i++)
    {
        task_sh1106_display_clear(NULL);
        int desiredDistance = calibrationDistances[i];
        while ((latestLidarValue != desiredDistance) && (gpio_get_level(SW_GPIO) != 0))
        {
            sprintf(str, "Set the\n\
                  object within\n\
                  %dcm distance\n\
                  Measured:\n\
                  %dcm",
                    desiredDistance, latestLidarValue);
            task_sh1106_display_text(str);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        task_sh1106_display_clear(NULL);
        vTaskDelay(40 / portTICK_PERIOD_MS);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        while (gpio_get_level(SW_GPIO) != 0)
        {

            sprintf(str, "Press the\n\
                knob after\n\
                focusing on\n\
                the object.\n\
                Measured:\n\
                  %dcm",
                    latestLidarValue);
            task_sh1106_display_text(str);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        task_sh1106_display_clear(NULL);
        task_sh1106_display_text("Done!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        measurements[i] = ImpulseCurrentState;
    }

    displayMeasurements();
}

uint16_t getServoStep(uint16_t dist)
{
    if (dist < calibrationDistances[0])
    {
        float firstBracketResult = ((float)dist / (float)calibrationDistances[0]) * (float)measurements[0];
        printf("Dist: %d, New Impulse: %d, Current Servo:%d\n", dist, (int)firstBracketResult, ServoPhysicalState); // Debug
        return (int)(firstBracketResult);
    }

    for (int i = 1; i < measurementsSize; i++)
    {
        if (dist < calibrationDistances[i])
        {
            int16_t servoBracketDiff = measurements[i] - measurements[i - 1];
            int16_t distBracketDiff = calibrationDistances[i] - calibrationDistances[i - 1];
            int16_t inBracketDist = dist - calibrationDistances[i - 1];

            float distInBracketValue = (float)inBracketDist / (float)distBracketDiff * (float)servoBracketDiff;

            int16_t result = measurements[i - 1] + (int)distInBracketValue;
            printf("Dist: %d, New Impulse: %d, Current Servo:%d\n", dist, result, ServoPhysicalState); // Debug
            return result;
        }
    }
    printf("Error: Distance out of range\n");
    return 0;
}

// ------------------------------------ Task wznawiany zboczem SIA impulstora - określanie kierunku obrotu impulsatora - zasada działania w załączniku nr 4
void SIA_call(void *pvParameter)
{
    bool A1, B1;
    // bool L =0, R=0;
    while (1)
    {
        vTaskSuspend(NULL);
        A1 = gpio_get_level(SIA_GPIO);
        B1 = gpio_get_level(SIB_GPIO);
        if (B1 == A1)
        {
            if (ImpulseCurrentState == ServoPhysicalState && eTaskGetState(ServoTaskHandle) == eSuspended)
            {
                ImpulseCurrentState--;
                vTaskResume(ServoTaskHandle);
            }
            // if (R == 1)
            //     R = 0;
            // L=!L;
            // printf("Lewo %d\n", L); // Debug
        }
        else
        {
            if (ImpulseCurrentState == ServoPhysicalState && eTaskGetState(ServoTaskHandle) == eSuspended)
            {

                ImpulseCurrentState++;
                vTaskResume(ServoTaskHandle);
            }
            // if (L == 1)
            //     L = 0;
            // R=!R;
            // printf("Prawo %d\n", R); // Debug
        }

        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}
void servoStepRight(uint16_t steps)
{
    iot_servo_write_angle(LEDC_HIGH_SPEED_MODE, 0, Servo_right);
    // printf("Servo right\n"); // Debug
    vTaskDelay(ServoStepTimeMS * steps / portTICK_PERIOD_MS);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(ServoRestTimeMS / portTICK_PERIOD_MS);
}
void servoStepLeft(uint16_t steps)
{
    iot_servo_write_angle(LEDC_HIGH_SPEED_MODE, 0, Servo_left);
    // printf("Servo left\n"); // Debug
    vTaskDelay(ServoStepTimeMS * steps / portTICK_PERIOD_MS);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(ServoRestTimeMS / portTICK_PERIOD_MS);
}
void servoStepIdle(uint8_t steps)
{
    iot_servo_write_angle(LEDC_HIGH_SPEED_MODE, 0, Servo_idle);
    // printf("Servo idle step\n"); // Debug
    vTaskDelay(ServoStepTimeMS * steps / portTICK_PERIOD_MS);
}
void servoTask(void *pvParameter)
{

    iot_servo_init(LEDC_HIGH_SPEED_MODE, &servo_cfg);

    // servoStepRight(1200);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    while (1)
    {
        printf("Start\tServo:\t%d\tImpulse:\t%d\n", ServoPhysicalState, ImpulseCurrentState); // Debug
        if (ImpulseCurrentState > ServoPhysicalState)
        {
            servoStepRight(ImpulseCurrentState - ServoPhysicalState);
            ServoPhysicalState = ImpulseCurrentState;
        }
        if (ImpulseCurrentState < ServoPhysicalState)
        {
            servoStepLeft(ServoPhysicalState - ImpulseCurrentState);
            ServoPhysicalState = ImpulseCurrentState;
        }
        printf("Stop\tServo:\t%d\tImpulse:\t%d\n", ServoPhysicalState, ImpulseCurrentState); // Debug
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
        if (data[0] == 89)
        {
            uart_read_bytes(uart_num, data, 1, 100);
            if (data[0] == 89)
            {
                uart_read_bytes(uart_num, data, 7, 100);
                latestLidarValue = (data[1] << 8) | data[0];
            }
        }
    }
}
void operationTask()
{

    while (1)
    {
        if (eTaskGetState(ServoTaskHandle) == eSuspended)
        {
            ImpulseCurrentState = getServoStep(latestLidarValue);
            if (ImpulseCurrentState != ServoPhysicalState)
                vTaskResume(ServoTaskHandle);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        // printf("Dist: %d, Servo: %d\n", latestLidarValue, ImpulseCurrentState); // Debug
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
void startupTask()
{
    bool CalibrationBypass = gpio_get_level(SW_GPIO);
    task_sh1106_display_clear(NULL);
    task_sh1106_display_text("Initializing...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (CalibrationBypass)
        takeMeasurements();
    task_sh1106_display_clear(NULL);
    task_sh1106_display_text("Press the knob\n\
    to start operation");
    while (gpio_get_level(SW_GPIO) != 0)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    task_sh1106_display_clear(NULL);
    task_sh1106_display_text("Operation\nstarted");
    xTaskCreate(&operationTask, "operationTask", 4096, NULL, 10, &operationTaskHandle);
    while (1)
    {
        vTaskSuspend(NULL);
        if (eTaskGetState(operationTaskHandle) == eSuspended)
        {
            vTaskResume(operationTaskHandle);
        }
        else
        {
            vTaskSuspend(operationTaskHandle);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{

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
    xTaskCreate(&SIA_call, "SIA_call", 1024, NULL, 9, &SIA_callTaskHandle);             // Task obsługuijący impulsator
    xTaskCreate(&servoTask, "servoTask", 4096, NULL, 9, &ServoTaskHandle);              // Task obsługujący servo
    xTaskCreate(&lidarReadTask, "lidarReadTask", 4096, NULL, 10, &lidarReadTaskHandle); // Task obsługujący odczyt z Czujnika odległości
    xTaskCreate(&startupTask, "startupTask", 4096, NULL, 10, &startupTaskHandle);
    // gpio_set_direction(Servo_PWM_GPIO, GPIO_MODE_OUTPUT);
    // gpio_set_level(Servo_PWM_GPIO, 1);
}