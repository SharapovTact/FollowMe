#ifndef ULTRASONIC_COMMUNICATION_H
#define ULTRASONIC_COMMUNICATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "motors.h"
#include "string.h"

void UltrasonicCommunicationInit(void);
void UltrasonicCommunicationReceiverTask(void *arg);

#endif