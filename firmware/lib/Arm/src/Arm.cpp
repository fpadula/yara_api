#include "Arm.h"

void Arm::init(uint8_t pins[NO_OF_JOINTS], float specs[NO_OF_JOINTS][3]){    
    for(int i = 0; i < NO_OF_JOINTS;i++){
        this->joints[i] = Joint(pins[i], specs[i][0], specs[i][1], specs[i][2]);
        this->joints[i].init();
        this->joints[i].set_position(0);
    }
    this->_reached_targets = false;
}
void Arm::init(uint8_t pins[NO_OF_JOINTS], float specs[NO_OF_JOINTS][3], float initial_joint_positions[NO_OF_JOINTS]){    
    for(int i = 0; i < NO_OF_JOINTS;i++){
        this->joints[i] = Joint(pins[i], specs[i][0], specs[i][1], specs[i][2]);
        this->joints[i].init();
        this->joints[i].set_position(initial_joint_positions[i]);
    }
    this->_reached_targets = false;
}

void Arm::set_targets(float targets[NO_OF_JOINTS]){
    for(int i = 0; i < NO_OF_JOINTS;i++)
        joints[i].set_interpolation_target(targets[i]);
    this->_reached_targets = false;
}

void Arm::set_max_speeds(float targets[NO_OF_JOINTS]){
    for(int i = 0; i < NO_OF_JOINTS;i++)
        joints[i].set_max_speed(targets[i]);    
}

void Arm::set_interpolation_funcs(float targets[NO_OF_JOINTS]){
    for(int i = 0; i < NO_OF_JOINTS;i++)
        joints[i].set_interpolation_function((int)targets[i]);    
}

bool Arm::reached_targets(){return this->_reached_targets;}

void Arm::step(){
    bool all_reached = true;
    for(int i = 0; i < NO_OF_JOINTS;i++)
        all_reached &= joints[i].interpolation_step();
    this->_reached_targets = all_reached;
}

float Arm::get_joint_position(int joint_no){
    return joints[joint_no].get_position();
}
float Arm::get_joint_velocity(int joint_no){
    return joints[joint_no].get_velocity();
}
float Arm::get_joint_acceleration(int joint_no){
    return joints[joint_no].get_acceleration();
}
void Arm::read_joint_sensor(int joint_no){
    joints[joint_no].read_sensors();
}