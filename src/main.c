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

void IRAM_ATTR SIA_isr_handler(void* arg) {
  
 xTaskResumeFromISR(ISR_SIA);
//portYIELD_FROM_ISR(  );
}
void hello_task(void *pvParameter)
{
 
	while(1)
	{
	    printf("Hello world!\n");
	    vTaskDelay(10 / portTICK_RATE_MS);
	}
}
 
void status(void *pvParameter)
{
    bool lastA=1, lastB=1;
    bool A, B, W;
    while(1) {
        A=gpio_get_level(SIA_GPIO);
        B=gpio_get_level(SIB_GPIO);
        W=gpio_get_level(SW_GPIO);
        if ((lastA != A) || (lastB != B)){

            printf("SIA: %d, SIB: %d, SW: %d\n",A, B, W);
        }
        //printf("%d\n", gpio_get_level(SIB_GPIO));
        lastA= A;
        lastB= B;
        //vTaskDelay(100 / portTICK_RATE_MS);
    }
}
void SIA_call(void *pvParameter)
{
    bool B1;
    while(1){  
        vTaskSuspend(NULL);
        B1 = gpio_get_level(SIB_GPIO);
        if (B1 == 1){
            printf("Lewo\n");
        } else if (B1 == 0){
            printf("Prawo\n");
        } else {
            printf("Error\n");
        }
        vTaskDelay(50 / portTICK_RATE_MS);
 }
}
 
void app_main()
{
    

    ets_timer_init();



    gpio_set_direction(SIA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SIB_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SW_GPIO, GPIO_MODE_INPUT);



    gpio_pad_select_gpio(SIA_GPIO);
  
  // set the correct direction
  gpio_set_direction(SIA_GPIO, GPIO_MODE_INPUT);
  
  // enable interrupt on falling (1->0) edge for button pin
  gpio_set_intr_type(SIA_GPIO, GPIO_INTR_NEGEDGE);

  
  //Install the driver’s GPIO ISR handler service, which allows per-pin GPIO interrupt handlers.
  // install ISR service with default configuration
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(SIA_GPIO, SIA_isr_handler, NULL);



    xTaskCreate(&SIA_call, "SIA_call", 4096, NULL , 10, &ISR_SIA );
    //xTaskCreate(&hello_task, "hello_task", 2048, NULL, 5, NULL);
    //xTaskCreate(&status, "status", 2048,NULL,5,NULL );
}