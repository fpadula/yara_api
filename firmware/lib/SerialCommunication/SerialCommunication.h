#ifndef _SERIALCOMMUNICATION_H_
#define _SERIALCOMMUNICATION_H_

#include <Arduino.h>
#include <Interpolations.h>

#define INFO_REPORT 'r'
#define MOVE_START 's'
#define TASK_OK 'o'
#define TASK_ERROR 'e'
#define MOVE_REQUEST 't'
#define SPEED_REQUEST 'p'
#define INTERPOLATIONS_REQUEST 'i'
#define ARM_STATE_REQUEST 'a'

/*
Buffer size in bytes. We need 1 byte for msg type, 7x4=28 bytes for position
information, 7x4=28 bytes for speed information and 7x4=28 bytes for 
acceleration information. So we need 1 + 28 + 28 + 28 = 85 bytes for our buffer
*/
#define BUFFER_SIZE 85

// We need 7 floats for position information, 7 for velocity and 7 for 
// acceleration
#define INFO_ARRAY_SIZE 21

class SerialCommunication{
    private:
        byte inputb[BUFFER_SIZE];
        byte outputb[BUFFER_SIZE];

        inline void buffer2array(byte *, float *, int);
        inline void array2buffer(byte *, float *, int);
        bool read_bytes(byte *, size_t);        
    public:
        // SerialCommunication();
        char get_msg_type();
        void get_float_array();
        bool data_available();
        void send_data(char);
        void send_data(char, float *, int);
        void read_data(char *, float *, int);
};

#endif