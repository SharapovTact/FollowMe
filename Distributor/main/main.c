#include <stdio.h>
#include "esp_log.h"

#include "motors.h"
#include "ultrasonicCommunication.h"

void app_main(void)
{
    UltrasonicCommunicationInit();
    MotorsInit();
    ESP_LOGI("MAIN", "LOGI ACTIVATED");
}