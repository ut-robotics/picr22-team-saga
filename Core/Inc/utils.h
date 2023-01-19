//
// Created by Artyom on 21.12.2022.
//

#ifndef SERVO_UTILS_H
#define SERVO_UTILS_H
#include "stdint.h"

int32_t max(int32_t a, int32_t b);
int32_t min(int32_t a, int32_t b);
int32_t clamp(int32_t max, int32_t min, int32_t val);
#endif//SERVO_UTILS_H
