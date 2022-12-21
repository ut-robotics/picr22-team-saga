//
// Created by Artyom on 21.12.2022.
//

#ifndef __DSHOT_H__
#define __DSHOT_H__
#include "stdint.h"
#define MOTOR_BIT_0 33
#define MOTOR_BIT_1 77
#define DSHOT_FRAME_SIZE (18 + 10)

void dshot600(uint32_t *motor, uint16_t value);

#endif /* __DSHOT_H__ */