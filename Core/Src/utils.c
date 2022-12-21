//
// Created by Artyom on 21.12.2022.
//
#include "utils.h"

int32_t max(int32_t a, int32_t b){
    return a > b ? a : b;
}
int32_t min(int32_t a, int32_t b) {
    return a < b ? a : b;
}
int32_t clamp(int32_t max, int32_t min, int32_t val) {
    if (val>max){
        return max;
    }else if (val < min){
        return min;
    } else {
        return val;
    }
}