#ifndef _UTILS_H_
#define _UTILS_H_

#include <math.h>
#include <Arduino.h>

class Utils{
    public:
        static float mapf(float, float, float, float, float);
        static int mapi(int, int, int, int, int);
        static float absf(float);
};

inline float rad2deg(float rad){return (180.0*rad)/PI;}  
inline float deg2rad(float deg){return (PI*deg)/180.0;}  

#endif