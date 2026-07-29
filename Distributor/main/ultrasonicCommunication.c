#include "ultrasonicCommunication.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <string.h>
#include "motors.h"

#define RX_TASK_STACK_SIZE   4096
#define RX_TASK_PRIORITY     5
#define UART_BUF_SIZE        1024

#define READ_TIMEOUT         20
#define UART_PORT_NUM	     UART_NUM_1
#define UART_BAUD_RATE       115200
#define TXD_PIN              16
#define RXD_PIN              17
#define RX_BUF_SIZE          2

#define UART_MARKER 	     0xFF
#define UART_MARKER_POS      0
#define UART_ANGLE_POS       1
#define DATA_LENGTH          2
#define ANGLE_CORRECTION_DEG 90

void UltrasonicCommunicationStart(void) {
    const uart_config_t uartConfig = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(
        UltrasonicCommunicationReceiverTask,
        "UARTReceiverTask",
        RX_TASK_STACK_SIZE, 
        NULL,           
        RX_TASK_PRIORITY,       
        NULL         
    );
}

bool IsValidData(uint8_t *data, const int *dataLength) {
	if (*dataLength == DATA_LENGTH && data[0] == UART_MARKER){
		return true;
	}
	return false;
} 

void ChangeMovingState(const uint8_t *data) {
	const uint8_t angleByte = data[UART_ANGLE_POS];
	const int angle = angleByte - ANGLE_CORRECTION_DEG;
	SetMotorState(angle);
}

void UltrasonicCommunicationReceiverTask(void *arg) {
    uint8_t rxData[RX_BUF_SIZE];
    while (1) {
		memset(rxData, 0, sizeof(uint8_t) * RX_BUF_SIZE);
        int dataLength = uart_read_bytes(UART_PORT_NUM, rxData, RX_BUF_SIZE, pdMS_TO_TICKS(READ_TIMEOUT));
        if (IsValidData(rxData, &dataLength)) {
			ChangeMovingState(rxData);
		}
    }
    vTaskDelete(NULL);
}