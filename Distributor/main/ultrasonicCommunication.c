#include "ultrasonicCommunication.h"

#define READ_TIMEOUT       20
#define UART_PORT_NUM	   UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_MARKER 	   0xFF
#define UART_MARKER_POS    0
#define UART_ANGLE_POS     1
#define TXD_PIN            16
#define RXD_PIN            17
#define RX_BUF_SIZE        1024
#define MS_YELD_TIME       10
#define DATA_LENGTH        2

#define FORWARD_EPSILON   30
#define SMALL_ROT_ROOF    60

void UltrasonicCommunicationInit(void) {
    const uart_config_t uartConfig = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	const int defaultPriority = 10;
    xTaskCreate(
        UltrasonicCommunicationReceiverTask,
        "UARTReceiverTask",
        4096, 
        NULL,           
        defaultPriority,       
        NULL         
    );
}

bool isValidData(uint8_t *data, const int *dataLength){
	if (*dataLength == DATA_LENGTH && data[0] == UART_MARKER){
		return true;
	}
	return false;
} 

void ChangeMovingState(const uint8_t *data){
	const uint8_t angleByte = data[UART_ANGLE_POS];
	static int lastAngle = 0;
	const int angle = angleByte - 90;
	if (angle < 90 && angle > -90 && (abs(lastAngle - angle) < 10)){
		
	}
	SetMotorState(angle);
	lastAngle = angle;
}

void UltrasonicCommunicationReceiverTask(void *arg) {
    uint8_t* rxData = (uint8_t*) malloc(RX_BUF_SIZE);
    while (1) {
		memset(rxData, 0, sizeof(*rxData));
        int dataLength = uart_read_bytes(UART_PORT_NUM, rxData, 2, pdMS_TO_TICKS(READ_TIMEOUT));
        if (isValidData(rxData, &dataLength)) {
			ChangeMovingState(rxData);
		}
    }
    free(rxData);
    vTaskDelete(NULL);
}