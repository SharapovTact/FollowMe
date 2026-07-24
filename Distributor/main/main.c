#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "motors.h"
#include "ultrasonicCommunication.h"

void app_main(void)
{
    UltrasonicCommunicationInit();
    MotorsInit();
    ESP_LOGI("MAIN", "LOGI ACTIVATED");
}