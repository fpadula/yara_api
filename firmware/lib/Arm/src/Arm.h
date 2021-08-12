#ifndef _ARM_H_
#define _ARM_H_

#include <Arduino.h>
#include <Joint.h>

#define NO_OF_JOINTS 7

class Arm{
    private:
        Joint joints[NO_OF_JOINTS];
        float limits[NO_OF_JOINTS][2];        
        bool _reached_targets;
    public:        
        void init(uint8_t [NO_OF_JOINTS], float [NO_OF_JOINTS][3]);
        void init(uint8_t [NO_OF_JOINTS], float [NO_OF_JOINTS][3], float [NO_OF_JOINTS]);
        void set_interpolation_funcs(float [NO_OF_JOINTS]);
        void set_max_speeds(float [NO_OF_JOINTS]);
        void set_targets(float [NO_OF_JOINTS]);
        bool reached_targets();
        void step();
        void read_joint_sensor(int);
        float get_joint_position(int);
        float get_joint_velocity(int);
        float get_joint_acceleration(int);
};

#endif