#include "motors.h"
#include "movingConfig.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/mcpwm_prelude.h>
#include <freertos/projdefs.h>
#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <freertos/queue.h>
#include <sys/time.h>

//INIT
#define MOTOR_L_GPIO           18
#define MOTOR_R_GPIO           19
#define MOTOR_TASK_STACK_SIZE  2048
#define MOTOR_TASK_PRIORITY    5
#define TIM_RESOLUTION_HZ      1000000
#define TIM_ACCOUNT_LIMIT      20000
#define US_PER_SEC             1000000L
#define EPSILON_FLOAT          0.001f

//PULSE_TO_LOAD
#define PULSE_STOP		       1500
#define PULSE_WORK_RANGE       500

//MOVEMENT
#define MAX_MOVE_SPEED         60
#define FULL_LOAD              100
#define MAX_REPEAT_COUNT       5
#define DELAY_MS_MOTOR         100
#define FORWARD_RANGE_DEG      4
#define MOVEMENT_RANGE_DEG     10
#define PID_GAIN_P             3.0f
#define PID_GAIN_I             3.0f
#define PID_GAIN_D             0
#define ACCELERATION_MOVE      1

typedef struct {
	int rotationalSpeed;
	float accumRotationalSpeed;
	int accumMoveSpeed;
	
	int angle;
	uint64_t prevTime;
	float deltaTime;
} PIDContext;

typedef struct {
	int pulseR;
	int pulseL;
	int repeatCounter;
} MotorCMD;

typedef struct {
	int rotation;
	int movement;
} PIDLoad;

static QueueHandle_t motorQueue = NULL;
static TaskHandle_t MotorTaskHandler = NULL;

static mcpwm_cmpr_handle_t CMP_L = NULL; 
static mcpwm_cmpr_handle_t CMP_R = NULL;

static void TimerInit(mcpwm_timer_handle_t *TIM, mcpwm_timer_config_t *CNFG) {
    CNFG->group_id = 0;
    CNFG->clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    CNFG->resolution_hz = TIM_RESOLUTION_HZ;
    CNFG->period_ticks = TIM_ACCOUNT_LIMIT;
    CNFG->count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    mcpwm_new_timer(CNFG, TIM);
}

static void OperatorInit(mcpwm_oper_handle_t *OPERATOR, mcpwm_operator_config_t *CNFG) {
    CNFG->group_id = 0;
    mcpwm_new_operator(CNFG, OPERATOR);
}

static void GeneratorInit(mcpwm_oper_handle_t OPERATOR, mcpwm_gen_handle_t *MOTOR, 
    mcpwm_generator_config_t *MOTOR_CNFG, const int MOTOR_GPIO) {
    *MOTOR = NULL;
    MOTOR_CNFG->gen_gpio_num = MOTOR_GPIO;
    mcpwm_new_generator(OPERATOR, MOTOR_CNFG, MOTOR);
}
 
void MotorTask(void *pvParameters) {
    MotorCMD motorCMD;
    while(1) {
        if (xQueueReceive(motorQueue, &motorCMD, portMAX_DELAY) == pdPASS) {
	        mcpwm_comparator_set_compare_value(CMP_L, motorCMD.pulseL);
	        mcpwm_comparator_set_compare_value(CMP_R, motorCMD.pulseR);
	        vTaskDelay(pdMS_TO_TICKS(DELAY_MS_MOTOR));
	        motorCMD.repeatCounter++;
	        if (motorCMD.repeatCounter >= MAX_REPEAT_COUNT){
				mcpwm_comparator_set_compare_value(CMP_L, PULSE_STOP);
	        	mcpwm_comparator_set_compare_value(CMP_R, PULSE_STOP);
			} else if (uxQueueMessagesWaiting(motorQueue) == 0){
				xQueueOverwrite(motorQueue, &motorCMD);
			}
		}
    }
}

int LoadToPulse(const int load) {
	static int k = PULSE_WORK_RANGE / FULL_LOAD;
	static int b = PULSE_STOP;
	return (k * load + b);
}

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

int FilterOutputValue(int output){
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
	else{
		*accumRotationalSpeed = 0;	
	}
	return;
}

void CalcAccumMoveSpeed(int *accumMoveSpeed, const int *angle) {
	if (IsMovementDirection(*angle)) {
		if (*accumMoveSpeed < MAX_MOVE_SPEED) {
			*accumMoveSpeed += ACCELERATION_MOVE;
		}
	}
	else {
		if (*accumMoveSpeed > 0){
			*accumMoveSpeed -= ACCELERATION_MOVE;
		}
	}
	return;
}

void PIDContextUpdate(PIDContext *context, const int angle) {
	uint64_t time = GetTimeUs();
	context->deltaTime = FilterTime((float)(time - context->prevTime) / US_PER_SEC);
	context->angle = angle;
	context->rotationalSpeed = angle;
	context->prevTime = time;
}

PIDLoad PIDFilter(const int angle) {
	PIDLoad load;
	static bool isInitialized = false;
	static PIDContext context;
	if (!isInitialized){
		InitPIDConfig(&context, &isInitialized);
	}
	
	PIDContextUpdate(&context, angle);
	
	CalcRotationalSpeed(&context.rotationalSpeed, &angle);
	CalcAccumRotationalSpeed(&context.accumRotationalSpeed, &angle, &context.deltaTime);
	CalcAccumMoveSpeed(&context.accumMoveSpeed, &angle);
	
	load.rotation = PID_GAIN_P * context.rotationalSpeed +     //TODO Надо вынести скорость движения и скорость поворота + ограничить
				    PID_GAIN_I * context.accumRotationalSpeed; // скорость движения до 70%. Таким образом контроллировать доворот на одной гусле
	load.movement = context.accumMoveSpeed;
	return load;
}

void SetMotorState(int targetAngle) {
	MotorCMD motorCMD;
	motorCMD.repeatCounter = 0;
	PIDLoad load = PIDFilter(targetAngle);
	int leftLoad = load.movement;
	int rightLoad = load.movement;
	if (load.rotation >= 0) {
		rightLoad += load.rotation;
	}
	else {
		leftLoad += load.rotation;
	}
	motorCMD.pulseR = LoadToPulse(rightLoad);
	motorCMD.pulseL = LoadToPulse(leftLoad);
	xQueueOverwrite(motorQueue, &motorCMD);
}

void MotorsStart(void) {
	motorQueue = xQueueCreate(1, sizeof(MotorCMD));
	
    xTaskCreate(MotorTask,
     "MotorLTask",
     MOTOR_TASK_STACK_SIZE,
     NULL,
     MOTOR_TASK_PRIORITY,
     &MotorTaskHandler);

    mcpwm_timer_handle_t TIM0;
    mcpwm_timer_config_t TIM0_CONFIG = {0}; 
    TimerInit(&TIM0, &TIM0_CONFIG);
        
    mcpwm_oper_handle_t OPERATOR;
    mcpwm_operator_config_t OPERATOR_CONFIG = {0}; 
    OperatorInit(&OPERATOR, &OPERATOR_CONFIG);
    mcpwm_operator_connect_timer(OPERATOR, TIM0);
    
    mcpwm_comparator_config_t compareConfig = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(OPERATOR, &compareConfig, &CMP_L);
    mcpwm_new_comparator(OPERATOR, &compareConfig, &CMP_R);
    
    mcpwm_gen_handle_t MOTOR_L;
    mcpwm_gen_handle_t MOTOR_R;
    mcpwm_generator_config_t MOTOR_L_CNFG = {0};
    mcpwm_generator_config_t MOTOR_R_CNFG = {0};
    
    GeneratorInit(OPERATOR, &MOTOR_L, &MOTOR_L_CNFG, MOTOR_L_GPIO);
    GeneratorInit(OPERATOR, &MOTOR_R, &MOTOR_R_CNFG, MOTOR_R_GPIO);
    
    mcpwm_generator_set_action_on_timer_event(MOTOR_L, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(MOTOR_L, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, CMP_L, MCPWM_GEN_ACTION_LOW));
    
    mcpwm_generator_set_action_on_timer_event(MOTOR_R, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(MOTOR_R, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, CMP_R, MCPWM_GEN_ACTION_LOW)); 
        
    mcpwm_timer_enable(TIM0);
    mcpwm_timer_start_stop(TIM0, MCPWM_TIMER_START_NO_STOP);
    
    mcpwm_comparator_set_compare_value(CMP_L, PULSE_STOP);
    mcpwm_comparator_set_compare_value(CMP_R, PULSE_STOP);
}