#include "motors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "movingConfig.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include <sys/time.h>

//INIT
#define MOTOR_L_GPIO           18
#define MOTOR_R_GPIO           19
#define MOTOR_TASK_STACK_SIZE  2048
#define MOTOR_TASK_PRIORITY    5
#define TIM_RESOLUTION_HZ      1000000
#define TIM_MAX_PERIOD_TICKS   20000

//PULSE_TO_LOAD
#define PULSE_STOP		       1500
#define PULSE_WORK_RANGE       500

//MOVEMENT
#define FULL_LOAD              100
#define MAX_REPEAT_COUNT       5
#define DELAY_MS_MOTOR         100
#define FORWARD_RANGE_DEG      10
#define PID_GAIN_P             3.0f
#define PID_GAIN_I             0.5f
#define PID_GAIN_D             0

typedef enum {
	NONE,
	PID,
	FORWARD,
} MoveType;

typedef struct {
	int pulseR;
	int pulseL;
	int repeatCounter;
} MotorCMD;

static QueueHandle_t motorQueue = NULL;
static TaskHandle_t MotorTaskHandler = NULL;

static mcpwm_cmpr_handle_t CMP_L = NULL; 
static mcpwm_cmpr_handle_t CMP_R = NULL;

static void TimerInit(mcpwm_timer_handle_t *TIM, mcpwm_timer_config_t *CNFG) {
    CNFG->group_id = 0;
    CNFG->clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    CNFG->resolution_hz = TIM_RESOLUTION_HZ;
    CNFG->period_ticks = TIM_MAX_PERIOD_TICKS;
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
    return (int64_t)tv.tv_sec * 1000000L + (int64_t)tv.tv_usec;
}

bool InForwardRange(const int angle) {
	return abs(angle) <= FORWARD_RANGE_DEG / 2;
}

bool InPIDRange(const int angle) {
	return !InForwardRange(angle);
}
MoveType SetMoveType(const int angle) {
    if (InForwardRange(angle)) {
		return FORWARD;
	}
	if (InPIDRange(angle)) {
		return PID;
	}
	return NONE;
}

int PIDFilter(const int angle, MoveType prevType){
	static float integral = 0;
	static uint64_t prevTime = 0;
	static int prevError = 0;
	if (prevType != PID){
		integral = 0;
		prevError = 0;
	}
	if (prevTime == 0) {
		prevTime = GetTimeUs();
		prevError = angle;
		return -1;
	}
	uint64_t time = GetTimeUs();
	int error = angle;
    float deltaTime = (float)(time - prevTime) / 1000000.0f;
    if (deltaTime <= 0.0f) {
        deltaTime = 0.001f;
    }
	
	int proportial = error;
	integral += (float)error * deltaTime;
	float derivative = ((float)error - prevError) / (float)deltaTime;
	float output = PID_GAIN_P * proportial + PID_GAIN_I * integral + PID_GAIN_D * derivative;
	
	prevTime = time;
	prevError = error;
    
    if (output > FULL_LOAD) {
		output = FULL_LOAD;
	}
    if (output < -FULL_LOAD) {
		output = -FULL_LOAD;
    }
    return output;
}

void SetMotorState(int targetAngle) {
	MoveType type = NONE;
	static MoveType prevType = NONE;
	
	MotorCMD motorCMD;
	type = SetMoveType(targetAngle);
	motorCMD.repeatCounter = 0;
	
	switch (type){
		case FORWARD:
			motorCMD.pulseR = LoadToPulse(FULL_LOAD);
    		motorCMD.pulseL = LoadToPulse(FULL_LOAD);
    		break;
		case PID:
			int output = PIDFilter(targetAngle, prevType);
			if (output == -1){
				prevType = type;
				return;
			}
			motorCMD.pulseR = LoadToPulse(output);
    		motorCMD.pulseL = LoadToPulse(-output);
    		break;
    	default:
    		return;
	}
	prevType = type;
	xQueueOverwrite(motorQueue, &motorCMD);
    
}

void MotorsInit(void) {
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