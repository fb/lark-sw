#ifndef ESP_DELAY_H_
#define ESP_DELAY_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void esp_delay_ms(uint32_t time_ms)
{
    vTaskDelay( time_ms / portTICK_PERIOD_MS );
}

#endif // ESP_DELAY_H_
