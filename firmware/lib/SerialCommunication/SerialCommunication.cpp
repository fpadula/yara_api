#include "SerialCommunication.h"

inline void SerialCommunication::buffer2array(byte *buffer, float *array, int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(&array[i], buffer + sizeof(float)*i, sizeof(float));
}

inline void SerialCommunication::array2buffer(byte *buffer, float *array, int array_size){
    for(int i = 0; i < array_size; i++)
        memcpy(buffer + sizeof(float)*i, &array[i], sizeof(float));
}

bool SerialCommunication::read_bytes(byte *buffer, size_t n){
    size_t bytesRead = 0;
    int result;
    memset(buffer, 0, n);
    while (bytesRead < n){
        result = Serial.readBytes(buffer + bytesRead, n - bytesRead);
        if (result < 1 ){
            return false;
        }
        bytesRead += result;
    }
    return true;
}

char SerialCommunication::get_msg_type(){
    char ret;        
    memcpy(&ret, this->inputb, sizeof(char));
    return ret;
}

bool SerialCommunication::data_available(){
    return (Serial.available() > 0);
}

void SerialCommunication::send_data(char msg_type, float *data, int data_length){
    this->outputb[0] = msg_type;    
    this->array2buffer(&(this->outputb[1]), data, data_length);     
    Serial.write(this->outputb, BUFFER_SIZE);
}

void SerialCommunication::send_data(char msg_type){
    this->outputb[0] = msg_type;        
    Serial.write(this->outputb, BUFFER_SIZE);
}

void SerialCommunication::read_data(char *msg_type, float *data, 
                                    int data_length){
    this->read_bytes(this->inputb, BUFFER_SIZE);    
    *msg_type = this->get_msg_type();
    buffer2array(&(this->inputb[1]), data, data_length);    
}