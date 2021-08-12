#include "Utils.h"

float Utils::mapf(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int Utils::mapi(int x, int in_min, int in_max, int out_min, int out_max){    
    return (int)roundf(mapf((float)x, (float)in_min, (float)in_max, (float)out_min, (float)out_max));
}

float Utils::absf(float value){
    if(value < 0)
        return -value;
    return value;
}