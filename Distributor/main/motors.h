#ifndef MOTORS_H
#define MOTORS_H

void MotorsStart(void);
void MotorLTask(void *pvParameters);
void MotorRTask(void *pvParameters);
void SetMotorState(int targetAngle);
void MotorsDebugSpin(int leftPulse, int rightPulse);

#endif