#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
 
#define SIA_GPIO 15
#define SIB_GPIO 2
#define SW_GPIO 4

 
 
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
 

    gpio_set_direction(SIA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SIB_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(SW_GPIO, GPIO_MODE_INPUT);
    while(1) {

        printf("SIA: %d, SIB: %d, SW: %d\n",gpio_get_level(SIA_GPIO), gpio_get_level(SIB_GPIO), gpio_get_level(SW_GPIO));
        //printf("%d\n", gpio_get_level(SIB_GPIO));
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
 
 
void app_main()
{
    ets_timer_init();
    //xTaskCreate(&hello_task, "hello_task", 2048, NULL, 5, NULL);
    xTaskCreate(&status, "status", 2048,NULL,5,NULL );
}