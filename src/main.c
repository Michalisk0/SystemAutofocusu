#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
 
#define SIA_GPIO 15
#define SIB_GPIO 16
#define SW_GPIO 4
#define ESP_INTR_FLAG_DEFAULT 0

TaskHandle_t ISR_SIA = NULL;


// ------------------------------------ Handler przerwania dla SIA impulsatora - wznowienie wątku przerwaniem
void IRAM_ATTR SIA_isr_handler(void* arg) {
 xTaskResumeFromISR(ISR_SIA);
}

// ------------------------------------ Wątek wznawiany zboczem SIA impulstora - określanie kierunku obrotu impulsatora - zasada działania w załączniku 
void SIA_call(void *pvParameter)
{
    bool B1;
    while(1){  
        vTaskSuspend(NULL);
        B1 = gpio_get_level(SIB_GPIO);
        if (B1 == 1){

            printf("Lewo\n"); // Debug
        } else if (B1 == 0){

            printf("Prawo\n"); // Debug
        } else {

            printf("Error\n"); // Debug
        }
        vTaskDelay(50 / portTICK_RATE_MS);
 }
}
 
void app_main()
{
    ets_timer_init();


// ------------------------------------- Uruchomienie GPIO dla impulsatora
    gpio_set_direction(SIA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SIB_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SW_GPIO, GPIO_MODE_INPUT);


// ------------------------------------- Uruchomienie obsługi przerwania dla SIA impulsatora
  gpio_set_intr_type(SIA_GPIO, GPIO_INTR_NEGEDGE);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(SIA_GPIO, SIA_isr_handler, NULL);


// ------------------------------------- Uruchomienie wątków systemowych
    xTaskCreate(&SIA_call, "SIA_call", 4096, NULL , 10, &ISR_SIA ); // Wątek obsługuijący impulator

}