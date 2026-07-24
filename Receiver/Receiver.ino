#include <math.h>

#define UART_MARKER            0xFF
#define PIN_SENSOR_L           3
#define PIN_SENSOR_R           2
#define DIST_BETWEEN_SENSORS_M 0.3
#define SPEED_OF_SOUND_M_S     343
#define ANGLE_ERROR            5
#define MAX_DELTATIME          10000

volatile uint32_t mcsRight;
volatile uint32_t mcsLeft;


bool leftFree = true;
bool rightFree = true;


typedef struct {
	int32_t mcs;
	bool sign;
} DeltaTime;

void handleLeftSensor() {
    mcsLeft = micros();
}

void handleRightSensor() {
    mcsRight = micros();
}

int CalcAngle(int deltaTimeMcs){
	double deltaTimeS = (double)deltaTimeMcs / 1000000;
	double sinVal = (deltaTimeS * SPEED_OF_SOUND_M_S) / DIST_BETWEEN_SENSORS_M;
	if (sinVal > 1.0) {
		sinVal = 1.0;
	}
    if (sinVal < -1.0) {
		sinVal = -1.0;
	}
    double angleRad = asin(sinVal);
    return (int)(angleRad * (180.0 / M_PI));
}

void setup(){
	Serial.begin(115200);

	pinMode(PIN_SENSOR_L, INPUT);
    pinMode(PIN_SENSOR_R, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_L), handleLeftSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_R), handleRightSensor, RISING);
}

void loop() {
    uint32_t leftTime = 0;
    uint32_t rightTime = 0;
    
    uint32_t currentTime = micros();
    if (currentTime - mcsRight < MAX_DELTATIME && currentTime - mcsLeft < MAX_DELTATIME && mcsRight != 0 && mcsLeft != 0){ 
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
        
        int32_t deltaMcs = (int32_t)(rightTime - leftTime);
        int angle = CalcAngle(deltaMcs);
        mcsRight = 0;
        mcsLeft = 0;
        Serial.write(UART_MARKER);
        Serial.write((uint8_t)(angle + 90));
        Serial.print(" ");
        Serial.println(angle);
        
        //Serial.println(deltaMcs);
        rightFree = true;
        leftFree = true;
    }
    
}