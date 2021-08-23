
#include <Arduino.h>
#include <Joint.h>
#include <Interpolations.h>
#include <Comm.h>
#include <SerialCommunication.h>
#include <Arm.h>

// #define WIFI
#ifdef WIFI
#include <OTA.h>
WiFiClient client;
WiFiServer wifiServer(1337);
Comm comm(&wifiServer, 2u);
#define mySSID "VenusWifi"
#define myPASSWORD ""
#endif

/**** Pins definition *****/
#define JOINT_1_PIN 23
#define JOINT_2_PIN 2
#define JOINT_3_PIN 19
#define JOINT_4_PIN 18
#define JOINT_5_PIN 4 
#define JOINT_6_PIN 21
#define GRIPPER_PIN 22
/**************************/

// #define VERBOSE

SerialCommunication comm = SerialCommunication();
Arm arm;
uint8_t arm_pinout[NO_OF_JOINTS] = {
    JOINT_1_PIN,
    JOINT_2_PIN,
    JOINT_3_PIN,
    JOINT_4_PIN,
    JOINT_5_PIN,
    JOINT_6_PIN,
    GRIPPER_PIN
};
// For each joint, specify min and max limits and the joint offset
float joint_specs[NO_OF_JOINTS][3] = {    
    {-90.0f, 90.0f, -4.0f},
    {-90.0f, 90.0f, -2.0f},
    {-90.0f, 90.0f, -5.0f},
    {-90.0f, 90.0f, 0.0f},
    {-90.0f, 90.0f, 6.0f},
    {-90.0f, 90.0f, 0.0f},
    {0.0f, 60.0f, 0.0f}
};
// float joint_specs[NO_OF_JOINTS][3] = {    
//     {-90.0f, 90.0f, -7.0f},
//     {-90.0f, 90.0f, 0.0f},
//     {-90.0f, 90.0f, -5.0f},
//     {-90.0f, 90.0f, 5.0f},
//     {-90.0f, 90.0f, 0.0f},
//     {-90.0f, 90.0f, 0.0f},
//     {0.0f, 90.0f, 0.0f}
// };

// float initial_pos[NO_OF_JOINTS] ={0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float initial_pos[NO_OF_JOINTS] ={0.00, 90.00, -90.00, 0.00, 90.00, 0.00, 0.00};

// Position, velocity and acceleration info for each joint
float info[NO_OF_JOINTS*3];
unsigned long last_report, report_period = 50;

void setup() {
    Serial.begin(115200);    

    arm.init(arm_pinout, joint_specs, initial_pos);

    #ifdef WIFI
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }
    setupOTA("yara");
    wifiServer.begin();
    #endif
}

void loop(){
    int i;
    char msg_type;
    float incoming_data[NO_OF_JOINTS];

    if(comm.data_available()) {
        // Reading message type and data
        comm.read_data(&msg_type, incoming_data, NO_OF_JOINTS);
        switch (msg_type){
            case MOVE_REQUEST:                
                // Telling the controller we are about to start moving
                comm.send_data(MOVE_START, incoming_data, NO_OF_JOINTS);
                arm.set_targets(incoming_data);
                while(!arm.reached_targets()){
                    // While all joints are not in final position, we perform a
                    // interpolation step
                    arm.step();
                    // Checking if we need to report current joint angles
                    if((arm.reached_targets()) 
                        || (millis() - last_report >= report_period)){
                        last_report = millis();
                        for(i = 0; i < NO_OF_JOINTS;i++){
                            arm.read_joint_sensor(i);
                            info[i] = arm.get_joint_position(i);
                            info[i + NO_OF_JOINTS] = arm.get_joint_velocity(i);
                            info[i + 2*NO_OF_JOINTS] = arm.get_joint_acceleration(i);
                        }
                        comm.send_data(INFO_REPORT, info, INFO_ARRAY_SIZE);
                    }
                    delay(1);
                }
                // We do not detect errors right now, so every end is a success
                // Reporting task completion to controller
                comm.send_data(TASK_OK, info, INFO_ARRAY_SIZE);
                break;
            case SPEED_REQUEST:
                // Setting target velocities
                arm.set_max_speeds(incoming_data);
                // Reporting task completion to controller
                comm.send_data(TASK_OK, incoming_data, NO_OF_JOINTS);
                break;
            case INTERPOLATIONS_REQUEST:
                // Setting joint interpolation functions
                arm.set_interpolation_funcs(incoming_data);
                // Reporting task completion to controller
                comm.send_data(TASK_OK, incoming_data, NO_OF_JOINTS);
                break;
            case ARM_STATE_REQUEST:
                for(i = 0; i < NO_OF_JOINTS;i++){
                    arm.read_joint_sensor(i);
                    info[i] = arm.get_joint_position(i);
                    info[i + NO_OF_JOINTS] = arm.get_joint_velocity(i);
                    info[i + 2*NO_OF_JOINTS] = arm.get_joint_acceleration(i);
                }
                comm.send_data(TASK_OK, info, INFO_ARRAY_SIZE);
                break;
            default:
                break;
        }
    }
}