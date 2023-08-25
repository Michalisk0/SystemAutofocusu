#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "iot_servo.h"

#define SIA_GPIO 15
#define SIB_GPIO 16
#define SW_GPIO 4
#define ESP_INTR_FLAG_DEFAULT 0
#define Servo_PWM_GPIO 23
#define Servo_idle 92.0f
#define Servo_right 110.0f
#define Servo_left 74.0f


TaskHandle_t ISR_SIA = NULL;
TaskHandle_t ServoTaskHandle = NULL;
uint16_t rightBuffer = 0, leftBuffer = 0;
ledc_timer_config_t PWM_timer;
ledc_channel_config_t PWM_channel;

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

// ------------------------------------ Handler przerwania dla SIA impulsatora - wznowienie wątku przerwaniem
void IRAM_ATTR SIA_isr_handler(void *arg)
{
    xTaskResumeFromISR(ISR_SIA);
}

// ------------------------------------ Wątek wznawiany zboczem SIA impulstora - określanie kierunku obrotu impulsatora - zasada działania w załączniku nr 4
void SIA_call(void *pvParameter)
{
    bool B1;
    while (1)
    {
        vTaskSuspend(NULL);
        B1 = gpio_get_level(SIB_GPIO);
        if (B1 == 1)
        {
            leftBuffer++;
            vTaskResume(ServoTaskHandle);
            printf("Lewo\n"); // Debug
        }
        else if (B1 == 0)
        {
            rightBuffer++;
            vTaskResume(ServoTaskHandle);
            printf("Prawo\n"); // Debug
        }
        else
        {

            // printf("Error\n"); // Debug
        }

        vTaskDelay(40 / portTICK_RATE_MS);
    }
}
void servoTask(void *pvParameter)
{
    float angle = Servo_idle;
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);
    while (1)
    {
        while (rightBuffer > 0)
        {
            // PWM_channel.duty+=50;
            // angle += 1.0f;
            rightBuffer--;
            // ledc_channel_config(&PWM_channel);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_right);
            vTaskDelay(50 / portTICK_RATE_MS);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
            //printf("In rightBuffer while\n"); // Debug
        }
        while (leftBuffer > 0)
        {
            // PWM_channel.duty-=50;
            //angle -= 1.0f;
            leftBuffer--;
            // ledc_channel_config(&PWM_channel);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_left);
            vTaskDelay(50 / portTICK_RATE_MS);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, Servo_idle);
            //printf("In leftBuffer while\n"); // Debug
        }
        //printf("Angle %f\n", angle); // Debug
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

    // ------------------------------------- Konfiguracja obsługi przerwania dla SIA impulsatora
    gpio_set_intr_type(SIA_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(SIA_GPIO, SIA_isr_handler, NULL);

    printf("Impulsator conf - done\n"); // Debug

    // -------------------------------------- Konfiguracja GPIO dla Servo

    // PWM_timer.bit_num = LEDC_TIMER_15_BIT;
    // PWM_timer.freq_hz = 50;
    // PWM_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    // PWM_timer.timer_num = LEDC_TIMER_0;
    // ledc_timer_config(&PWM_timer);

    // PWM_channel.channel = LEDC_CHANNEL_0;
    // PWM_channel.duty = Servo_idle;
    // PWM_channel.gpio_num = Servo_PWM_GPIO;
    // PWM_channel.intr_type = LEDC_INTR_DISABLE;
    // PWM_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    // PWM_channel.timer_sel = LEDC_TIMER_0;
    // ledc_channel_config(&PWM_channel);

    // printf("Servo conf - done\n"); // Debug

    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

    // ------------------------------------- Uruchomienie wątków systemowych
    xTaskCreate(&SIA_call, "SIA_call", 1024, NULL, 10, &ISR_SIA);           // Wątek obsługuijący impulator
    xTaskCreate(&servoTask, "servoTask", 2048, NULL, 10, &ServoTaskHandle); // Wątek obsługujący servo
}