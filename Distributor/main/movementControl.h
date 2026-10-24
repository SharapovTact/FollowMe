#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include <stdbool.h>

#define FULL_LOAD 100

typedef struct {
	int rotation;
	int movement;
} MotorLoad;

MotorLoad GetLoadForMovement(const int angle);

bool IsForwardDirection(const int angle);
bool IsMovementDirection(const int angle);

int FilterLoadValue(int output);


#endif