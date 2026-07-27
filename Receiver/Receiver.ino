#include <math.h>

#define UART_MARKER            0xFF
#define PIN_SENSOR_L           3
#define PIN_SENSOR_R           2
#define DIST_BETWEEN_SENSORS_M 0.3
#define SPEED_OF_SOUND_MS      343
#define ANGLE_ERROR            5
#define MAX_DELTATIME          10000
#define UART_SPEED             115200

volatile uint32_t usRight;
volatile uint32_t usLeft;

bool leftFree = true;
bool rightFree = true;

typedef struct {
	int32_t us;
	bool sign;
} DeltaTime;

void HandleLeftSensor() {
    usLeft = micros();
}

void HandleRightSensor() {
    usRight = micros();
}

double CutInvalidSinValue(const double sinVal) {
    if (sinVal > 1.0) {
		return 1.0;
	}
    if (sinVal < -1.0) {
		return -1.0;
	}
    return sinVal;
}

int CalcAngle(int deltaTimeus) {
	double deltaTimeS = (double)deltaTimeus / 1000000;
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
    if (currentTime - usRight < MAX_DELTATIME && currentTime - usLeft < MAX_DELTATIME && usRight != 0 && usLeft != 0) { 
        noInterrupts();
        if (rightFree) {
            rightTime = usRight;
            rightFree = false;
        }
        if (leftFree) {
            leftTime = usLeft;
            leftFree = false;
        }
        interrupts();
    }
    if (leftFree == false && rightFree == false) {
        int32_t deltaus = rightTime - leftTime;
        int angle = CalcAngle(deltaus);
        usRight = 0;
        usLeft = 0;
        Serial.write(UART_MARKER);
        Serial.write((uint8_t)(angle + 90));
        Serial.print(" ");
        Serial.println(angle);
        rightFree = true;
        leftFree = true;
    }
}