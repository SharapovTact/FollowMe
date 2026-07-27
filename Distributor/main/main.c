#include <stdio.h>
#include <esp_log.h>

#include "motors.h"
#include "ultrasonicCommunication.h"

void app_main(void)
{
    UltrasonicCommunicationStart();
    MotorsStart();
    ESP_LOGI("MAIN", "LOGI ACTIVATED");
}