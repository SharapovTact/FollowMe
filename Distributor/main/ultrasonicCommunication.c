#include "ultrasonicCommunication.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <string.h>
#include <stdlib.h>
#include "motors.h"
#include <esp_log.h>

#define STOP_DISTANCE_SM     100

#define RX_TASK_STACK_SIZE   4096
#define RX_TASK_PRIORITY     5
#define UART_BUF_SIZE        1024

#define READ_TIMEOUT         20
#define UART_PORT_NUM        UART_NUM_1
#define UART_BAUD_RATE       115200
#define TXD_PIN              16
#define RXD_PIN              17

#define LED_PIN              2
#define LED_ON_LEVEL         1

#define PACKET_MAX_LEN       64
#define PACKET_SEPARATOR     '|'

static void LedSet(bool on) {
    gpio_set_level(LED_PIN, on ? LED_ON_LEVEL : !LED_ON_LEVEL);
}

static void InitLed(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    LedSet(false);
}

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

    InitLed(); 

    xTaskCreate(
        UltrasonicCommunicationReceiverTask,
        "UARTReceiverTask",
        RX_TASK_STACK_SIZE, 
        NULL,           
        RX_TASK_PRIORITY,       
        NULL         
    );
}

bool IsValidData(const char *data) {
    const char *sep = strchr(data, PACKET_SEPARATOR);
    if (sep == NULL) return false;
    if (sep == data) return false;
    if (*(sep + 1) == '\0') return false;
    if (strchr(sep + 1, PACKET_SEPARATOR) != NULL) return false;

    char *endptr;
    strtol(data, &endptr, 10);
    if (endptr != sep) return false;

    strtol(sep + 1, &endptr, 10);
    if (*endptr != '\0') return false;

    return true;
}

void ChangeMovingState(const char *data) {
    const char *sep = strchr(data, PACKET_SEPARATOR);

    long distance = strtol(data, NULL, 10);
    if (distance < 0) distance = 0;
    long angle = strtol(sep + 1, NULL, 10);
	if (distance > STOP_DISTANCE_SM) {
		ESP_LOGD("MAIN", "DISTANCE: %ld cm, ANGLE: %ld", distance, angle);
    	SetMotorState((int)angle);
	}
}

void UltrasonicCommunicationReceiverTask(void *arg) {
    uint8_t rxChunk[16];
    char packet[PACKET_MAX_LEN];
    int packetLen = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, rxChunk, sizeof(rxChunk),
                                  pdMS_TO_TICKS(READ_TIMEOUT));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)rxChunk[i];
                if (c == '\r' || c == '\n') {
                    if (packetLen > 0) {
                        packet[packetLen] = '\0';
                        if (IsValidData(packet)) {
                            LedSet(false);        
                            ChangeMovingState(packet);
                        } else {
                            LedSet(true);   
                            ESP_LOGW("MAIN", "Invalid packet: '%s'", packet);
                        }
                        packetLen = 0;
                    }
                } else if (packetLen < PACKET_MAX_LEN - 1) {
                    packet[packetLen++] = c;
                }
            }
        } else {
            if (packetLen > 0) {
                packet[packetLen] = '\0';
                if (IsValidData(packet)) {
                    LedSet(false);
                    ChangeMovingState(packet);
                } else {
                    LedSet(true);
                    ESP_LOGW("MAIN", "Invalid packet: '%s'", packet);
                }
                packetLen = 0;
            }
        }
    }
    vTaskDelete(NULL);
}