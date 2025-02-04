/* gesture_ai.h - Gesture Recognition Header File */
#ifndef GESTURE_AI_H
#define GESTURE_AI_H

#include "mpu6050.h"

#define GESTURE_NONE 0
#define GESTURE_LEFT 1
#define GESTURE_RIGHT 2
#define GESTURE_UP 3
#define GESTURE_DOWN 4

void GestureAI_Init(void);
int GestureAI_Recognize(MPU6050_Data *data);

#endif
