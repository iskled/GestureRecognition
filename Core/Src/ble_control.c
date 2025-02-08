/* ble_control.c - Bluetooth Low Energy (BLE) Communication */
#include "ble_control.h"
#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart4;

void BLE_Init(UART_HandleTypeDef *huart) {
    // Initialize bluetooth module
}

void BLE_SendGesture(int gesture) {
    char msg[20];
    snprintf(msg, sizeof(msg), "Gesture: %d\r\n", gesture);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
