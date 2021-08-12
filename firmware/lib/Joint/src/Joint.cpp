#include "Joint.h"

inline float mapf(float value, float src_min, float src_max, float dest_min,
                float dest_max){
    return (value - src_min) * (dest_max - dest_min) / (src_max - src_min)
        + dest_min;
}

inline float clamp(float value, float max, float min){
    return value > max? max : (value < min? min : value);
}

Joint::Joint(){};
Joint::Joint(uint8_t pin, float inf_lim, float sup_lim, float offset){
    this->pin = pin;
    this->inf_lim = inf_lim;
    this->sup_lim = sup_lim;
    this->offset = offset;
    this->angle = 0.0f;
    this->speed = 0.0f;
    this->accel = 0.0f;
    this->prev_angle = this->angle;
    this->prev_speed = this->speed;

    this->max_speed = 5.0f; // 5 degrees/s
    this->int_f = Interpolations::linear;

    this->int_fs[0] = Interpolations::linear;
    this->int_fs[1] = Interpolations::expo;
    this->int_fs[2] = Interpolations::bounce_out;
    this->int_fs[3] = Interpolations::inout_cubic;
}

void Joint::init(){
    this->servo.attach(this->pin);
    this->set_position(this->angle);
}

void Joint::set_max_speed(float max_speed){
    this->max_speed = max_speed;
}

void Joint::set_position(float target_pos){
    int in_us;
    target_pos = clamp(target_pos, this->sup_lim, this->inf_lim);
    in_us = (int) mapf(target_pos+ this->offset, -90.0f, 90.0f, MIN_SERVO_PULSE_WIDTH,
                        MAX_SERVO_PULSE_WIDTH);
    this->servo.writeMicroseconds(in_us);
    /*
    Ideally we would read the angle using encoder information. For now we assume
    that the current joint angle is equal to the angle requested to the servo.
    This should be fine for small increments, but may be throublesome for large
    angle variations because the servo would take too long to reach the given
    position.
    */
    this->angle = target_pos;
}

void Joint::read_sensors(){
    float dt_secs;
        
    dt_secs = ((float) (millis() - this->last_sensor_read))/1000.0f;
    this->last_sensor_read = millis();

    this->speed = (this->angle - this->prev_angle)/dt_secs;
    this->accel = (this->speed - this->prev_speed)/dt_secs;

    this->prev_angle = this->angle;
    this->prev_speed = this->speed;
}

float Joint::get_position(){return this->angle;}

float Joint::get_velocity(){return this->speed;}

float Joint::get_acceleration(){return this->accel;}

bool Joint::reached_limit(){
    return ((this->angle == this->sup_lim) || (this->angle == this->inf_lim));
}

void Joint::set_interpolation_target(float target_pos){
    float delta_pos;

    this->int_starting_pos = this->angle;
    this->int_final_pos = target_pos;
    delta_pos = this->int_final_pos - this->int_starting_pos;
    if(delta_pos < 0)
        delta_pos *= -1;
    this->int_duration = (unsigned long) ((delta_pos/this->max_speed) * 1000.0f);
    // this->int_f = ifunc;
    this->int_starting_t = millis();
    this->int_final_t = this->int_starting_t + this->int_duration;
}

void Joint::set_interpolation_function(float (*int_f)(float, float, float)){
    this->int_f = int_f;
}

void Joint::set_interpolation_function(int function_index){
    this->int_f = int_fs[function_index];
}

bool Joint::interpolation_step(){
    float curr_i_angle, i_value;

    if(this->int_duration != 0){
        i_value = (float)(millis() - this->int_starting_t)
                    /(float)this->int_duration;
        i_value = clamp(i_value, 1.0f, 0);
    }
    else{
        return true;
    }
    curr_i_angle = this->int_f(i_value, this->int_starting_pos,
                                this->int_final_pos);
    this->set_position(curr_i_angle);
    return (i_value == 1.0f);
}