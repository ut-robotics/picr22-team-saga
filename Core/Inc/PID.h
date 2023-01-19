//
// Created by Artyom on 21.12.2022.
//

#ifndef SERVO_PID_H
#define SERVO_PID_H
#include "dshot.h"
#include "utils.h"

typedef struct PIDConfigDef{
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;
    int32_t minOutput;
    int32_t maxOutput;
} PIDConfig;

typedef struct PIDStructDef{
    PIDConfig* config;
    int32_t last_error;
    int32_t error;
    int32_t integral;
    int32_t derivative;
} PIDState;


void updatePIDState(PIDState* state, int32_t new_value, int32_t expected);
int32_t calculateOutput(PIDState* state);

#endif//SERVO_PID_H
