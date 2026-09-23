#include "movementControl.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/mcpwm_prelude.h>
#include <freertos/projdefs.h>
#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <freertos/queue.h>
#include <sys/time.h>
#include <esp_log.h>

#define MAX_ROTATION_SPEED     60   // <-- ДОБАВЛЕНО: ограничение максимальной скорости поворота
#define MAX_FORWARD_SPEED      60   // <-- ИЗМЕНЕНО: бывший MAX_MOVE_SPEED, ограничение движения вперёд
#define FORWARD_RANGE_DEG      12
#define MOVEMENT_RANGE_DEG     60
#define PID_GAIN_P             2.0f
#define PID_GAIN_I             1.0f
#define PID_GAIN_D             0
#define ACCELERATION_MOVE      1
#define US_PER_SEC             1000000L
#define EPSILON_FLOAT          0.001f

typedef struct {
	int rotationalSpeed;
	float accumRotationalSpeed;
	int accumMoveSpeed;
	
	int angle;
	uint64_t prevTime;
	float deltaTime;
} PIDContext;

int64_t GetTimeUs() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * US_PER_SEC + (int64_t)tv.tv_usec;
}

bool IsForwardDirection(const int angle) {
	return abs(angle) <= FORWARD_RANGE_DEG / 2;
}

bool IsMovementDirection(const int angle) {
	return abs(angle) <= MOVEMENT_RANGE_DEG / 2;
}

int FilterLoadValue(int output) {
	if (output > FULL_LOAD) {
		return FULL_LOAD;
	}
    if (output < -FULL_LOAD) {
		return -FULL_LOAD;
    }
    return output;
}

static void InitPIDConfig(PIDContext *context, bool *isInitialized) {
	context->accumRotationalSpeed = 0;
	context->accumMoveSpeed = 0;
	context->prevTime = GetTimeUs();
	*isInitialized = true;
}

float FilterTime(float time) {
	if (time <= EPSILON_FLOAT) {
        return EPSILON_FLOAT;
    }
    return time;
}

void CalcRotationalSpeed(int *rotationalSpeed, const int *angle) {
	*rotationalSpeed = *angle;
	return;
}

void CalcAccumRotationalSpeed(float *accumRotationalSpeed, const int *angle, const float *deltaTime) {
	if (!IsForwardDirection(*angle)) {
		if (*accumRotationalSpeed < FULL_LOAD){
			*accumRotationalSpeed += (float)*angle * *deltaTime;
		}
	} 
	else {
		if (*accumRotationalSpeed > 0){
			*accumRotationalSpeed -= ACCELERATION_MOVE;	
		}
		else if (*accumRotationalSpeed < 0){
			*accumRotationalSpeed += ACCELERATION_MOVE;
		}
		
	}
	return;
}

void CalcAccumMoveSpeed(int *accumMoveSpeed, const int *angle) {
	if (IsMovementDirection(*angle)) {
		if (*accumMoveSpeed < MAX_FORWARD_SPEED) {   // <-- ИЗМЕНЕНО: MAX_MOVE_SPEED -> MAX_FORWARD_SPEED
			*accumMoveSpeed += ACCELERATION_MOVE;
		}
	}
	else {
		if (*accumMoveSpeed > 0){
			*accumMoveSpeed -= ACCELERATION_MOVE;
		}
	}
	ESP_LOGI("MOTORS", "Move speed: %d", *accumMoveSpeed);
	return;
}

void PIDContextUpdate(PIDContext *context, const int angle) {
	uint64_t time = GetTimeUs();
	context->deltaTime = FilterTime((float)(time - context->prevTime) / US_PER_SEC);
	context->angle = angle;
	context->rotationalSpeed = angle;
	context->prevTime = time;
}

MotorLoad GetLoadForMovement(const int angle) {
	MotorLoad load;
	static bool isInitialized = false;
	static PIDContext context;
	if (!isInitialized){
		InitPIDConfig(&context, &isInitialized);
	}
	
	PIDContextUpdate(&context, angle);
	
	CalcRotationalSpeed(&context.rotationalSpeed, &angle);
	CalcAccumRotationalSpeed(&context.accumRotationalSpeed, &angle, &context.deltaTime);
	CalcAccumMoveSpeed(&context.accumMoveSpeed, &angle);
	
	// Сначала вычисляем movementLoad, чтобы использовать его для ограничения rotationLoad
	int movementLoad = abs(FilterLoadValue(context.accumMoveSpeed));
	if (movementLoad > MAX_FORWARD_SPEED) {
		movementLoad = MAX_FORWARD_SPEED;
	}
	load.movement = movementLoad;

	int rotationLoad = abs(FilterLoadValue(PID_GAIN_P * context.rotationalSpeed +
				    					   PID_GAIN_I * context.accumRotationalSpeed));
	if (rotationLoad > MAX_ROTATION_SPEED) {
		rotationLoad = MAX_ROTATION_SPEED;
	}

	// <-- ДОБАВЛЕНО: ограничение поворотной нагрузки с учётом сложения с движением
	// Итоговая команда на мотор = movementLoad + rotationLoad.
	// Чтобы сумма не превышала MAX_FORWARD_SPEED, уменьшаем rotationLoad.
	if (movementLoad + rotationLoad > MAX_FORWARD_SPEED) {
		rotationLoad = MAX_FORWARD_SPEED - movementLoad;
		if (rotationLoad < 0) {
			rotationLoad = 0;
		}
	}
	load.rotation = rotationLoad;

	return load;
}