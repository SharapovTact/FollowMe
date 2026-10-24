#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "motors.h"
#include "ultrasonicCommunication.h"

void app_main(void)
{
    UltrasonicCommunicationStart();
    MotorsStart();
    ESP_LOGI("MAIN", "LOGI ACTIVATED");
    MotorsDebugSpin(1500, 1500);
     vTaskDelay(pdMS_TO_TICKS(4000));
}