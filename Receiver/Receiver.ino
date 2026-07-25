#include <math.h>

#define UART_MARKER            0xFF
#define PIN_SENSOR_L           3
#define PIN_SENSOR_R           2
#define DIST_BETWEEN_SENSORS_M 0.3
#define SPEED_OF_SOUND_MS      343
#define ANGLE_ERROR            5
#define MAX_DELTATIME          10000
#define UART_SPEED             115200

volatile uint32_t mcsRight;
volatile uint32_t mcsLeft;

bool leftFree = true;
bool rightFree = true;

typedef struct {
	int32_t mcs;
	bool sign;
} DeltaTime;

void HandleLeftSensor() {
    mcsLeft = micros();
}

void HandleRightSensor() {
    mcsRight = micros();
}

double CutInvalidSinValue(const sinVal) {
    if (sinVal > 1.0) {
		return 1.0;
	}
    if (sinVal < -1.0) {
		return -1.0;
	}
    return sinVal;
}

int CalcAngle(int deltaTimeMcs) {
	double deltaTimeS = (double)deltaTimeMcs / 1000000;
	double sinVal = (deltaTimeS * SPEED_OF_SOUND_MS) / DIST_BETWEEN_SENSORS_M;
	sinVal = CutInvalidSinValue(sinVal);
    double angleRad = asin(sinVal);
    return (int)(angleRad * (180.0 / M_PI));
}

void setup() {
	Serial.begin(UART_SPEED);

	pinMode(PIN_SENSOR_L, INPUT);
    pinMode(PIN_SENSOR_R, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_L), HandleLeftSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_R), HandleRightSensor, RISING);
}

void loop() {
    uint32_t leftTime = 0;
    uint32_t rightTime = 0;
    
    uint32_t currentTime = micros();
    if (currentTime - mcsRight < MAX_DELTATIME && currentTime - mcsLeft < MAX_DELTATIME && mcsRight != 0 && mcsLeft != 0) { 
        noInterrupts();
        if (rightFree) {
            rightTime = mcsRight;
            rightFree = false;
        }
        if (leftFree) {
            leftTime = mcsLeft;
            leftFree = false;
        }
        interrupts();
    }
    if (leftFree == false && rightFree == false) {
        int32_t deltaMcs = rightTime - leftTime;
        int angle = CalcAngle(deltaMcs);
        mcsRight = 0;
        mcsLeft = 0;
        Serial.write(UART_MARKER);
        Serial.write((uint8_t)(angle + 90));
        Serial.print(" ");
        Serial.println(angle);
        rightFree = true;
        leftFree = true;
    }
}