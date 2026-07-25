#ifndef MOTORS_H
#define MOTORS_H

void MotorsInit(void);
void MotorLTask(void *pvParameters);
void MotorRTask(void *pvParameters);
void SetMotorState(int targetAngle);

#endif