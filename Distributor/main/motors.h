#ifndef MOTORS_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "movingConfig.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include <sys/time.h>

#define MOTORS_H

extern TaskHandle_t MotorLTaskHandler;
extern TaskHandle_t MotorRTaskHandler;

void MotorsInit(void);
void MotorLTask(void *pvParameters);
void MotorRTask(void *pvParameters);
void SetMotorState(int targetAngle);

#endif