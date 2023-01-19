//
// Created by Artyom on 21.12.2022.
//
#include "PID.h"

void updatePIDState(PIDState *state, int32_t new_value, int32_t expected) {
    state->error = expected - new_value;
    state->integral = clamp(state->config->maxOutput/state->config->Ki,
                            state->config->minOutput/state->config->Ki,
                            state->integral + state->error);
    state->derivative = state->error - state->last_error;
    state->last_error = state->error;
}
int32_t calculateOutput(PIDState *state) {
    return clamp(state->config->maxOutput,state->config->minOutput,
                 (state->config->Kp * state->error) +
                     (state->config->Ki * state->integral) +
                     (state->config->Kd * state->derivative));
}
