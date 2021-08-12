#ifndef _COMM_H_
#define _COMM_H_

#include <Arduino.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <Joint.h>

class Comm{
    private:
        unsigned int info_rate, info_interval;
        WiFiServer *wifiServer;
        const char data_sep[2] = ",";
        unsigned long last_message_timestamp, current_timestamp;
        WiFiClient client;
    public:

        const char JOINT_POSITION_SET = 'd';
        const char done_msg[5] = "done";
        const unsigned int msgs_length = 5;

        Comm(WiFiServer *wifiServer, unsigned int info_rate);
        bool client_available();
        bool client_connected();
        void publish_joint_info(Joint **joints);
        bool check_incomming_data();
        void process_incomming_data(float *position, float *velocities);
        unsigned int process_incomming_trajectory_data(float position[100][6], float velocities[100][6]);
        void stop_client();
        void send_message(const char type);
};

#endif