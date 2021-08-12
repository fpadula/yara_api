#ifndef _JOINT_H_
#define _JOINT_H_

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Interpolations.h>

#define MIN_SERVO_PULSE_WIDTH 544.0f
#define MAX_SERVO_PULSE_WIDTH 2400.0f

class Joint{
    private:                 
        float angle, prev_angle, speed, prev_speed, accel;
        float max_speed, inf_lim, sup_lim, offset, int_final_pos, int_starting_pos; 
        float (*int_f)(float, float, float);
        float (*int_fs[4])(float, float, float);
        unsigned long int_duration, int_starting_t, int_final_t;
        unsigned long last_sensor_read;
        uint8_t pin;
        Servo servo;
    public:
        Joint();        
        Joint(uint8_t, float, float, float);        
        void set_position(float);
        void set_max_speed(float);
        void set_interpolation_target(float);
        void set_interpolation_function(float (*)(float, float, float));
        void set_interpolation_function(int);
        bool interpolation_step();
        void read_sensors();
        float get_position();
        float get_velocity();
        float get_acceleration();
        bool reached_limit();
        void init();
};

#endif