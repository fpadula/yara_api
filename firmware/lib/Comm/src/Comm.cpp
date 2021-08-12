// #include "Comm.h"

// Comm::Comm(WiFiServer *wifiServer, unsigned int info_rate){
//     this->wifiServer = wifiServer;
//     this->info_rate = info_rate;
//     this->info_interval = (1.0/this->info_rate)*1000.0; // Store in ms to save processing
//     this->last_message_timestamp = 0;
//     this->current_timestamp = millis();
// }

// void Comm::publish_joint_info(Joint **joints) {
//     char message_buffer[256];
//     uint8_t message_length_buffer[4];
//     unsigned int length;

//     this->current_timestamp = millis();
//     if(this->current_timestamp - this->last_message_timestamp >= this->info_interval){
//         this->last_message_timestamp = this->current_timestamp;
//         snprintf(message_buffer, sizeof(message_buffer), "%f,%f,%f,%f,%f,%f,",
//             joints[0]->get_current_angle(),
//             joints[1]->get_current_angle(),
//             joints[2]->get_current_angle(),
//             joints[3]->get_current_angle(),
//             joints[4]->get_current_angle(),
//             joints[5]->get_current_angle()
//         );

//         length = strlen(message_buffer);
//         message_length_buffer[0] = (uint8_t) length;
//         message_length_buffer[1] = (uint8_t) (length>>8);
//         message_length_buffer[2] = (uint8_t) (length>>16);
//         message_length_buffer[3] = (uint8_t) (length>>24);
//         if(this->client_connected()){
//             this->client.write(message_length_buffer, sizeof(message_length_buffer));
//             this->client.write(message_buffer, length);
//         }
//     }
// }

// void Comm::process_incomming_data(float *position, float *velocities){
//     char *ptr = NULL;
//     float *curr_array;
//     int i;
//     uint8_t message_length_buffer[4];
//     char message_buffer[256];
//     unsigned int message_length;

//     this->client.readBytes(message_length_buffer, sizeof(message_length_buffer));
//     message_length = (unsigned int)((message_length_buffer[3]<<24) | (message_length_buffer[2]<<16) | (message_length_buffer[1]<<8) | message_length_buffer[0]);
//     // memset(buffer, 0, sizeof(buffer));
//     if(this->client_connected())
//         this->client.readBytes((uint8_t*)message_buffer, message_length);
//     else
//         return;
//     message_buffer[message_length] = 0;

//     ptr = strtok(message_buffer, this->data_sep);
//     i = 0;
//     curr_array = position;
//     while(ptr != NULL){
//         if(i > 5){
//             i = 0;
//             curr_array = velocities;
//         }
//         curr_array[i] = atof(ptr);
//         i++;
//         ptr = strtok(NULL, this->data_sep);
//     }
// }

// unsigned int Comm::process_incomming_trajectory_data(float position[100][6], float velocities[100][6]){
//     char *ptr = NULL;    
//     int i,j;
//     uint8_t no_of_points_buffer[4], message_length_buffer[4];
//     // char message_buffer[25600];
//     char *message_buffer;
//     unsigned int message_length, no_of_points;    
//     // Serial.print("Will read "); Serial.print(this->client.available()); Serial.println(" bytes.");
//     // return 2u;
//     message_buffer = (char *)malloc(sizeof(char) * 25600);
//     if(this->client_connected()){
//         // Read number of waypoints in the trajectory:
//         this->client.readBytes(no_of_points_buffer, 4);
//         no_of_points = (unsigned int)((no_of_points_buffer[3]<<24) | (no_of_points_buffer[2]<<16) | (no_of_points_buffer[1]<<8) | no_of_points_buffer[0]);        
//         // Read message length:
//         this->client.readBytes(message_length_buffer, sizeof(message_length_buffer));
//         message_length = (unsigned int)((message_length_buffer[3]<<24) | (message_length_buffer[2]<<16) | (message_length_buffer[1]<<8) | message_length_buffer[0]);        
//         // Read message of size 'message_length'
//         this->client.readBytes((uint8_t*)message_buffer, message_length);        
//     }
//     else
//         return -1u;

//     ptr = strtok(message_buffer, this->data_sep);
//     for(i = 0; i < no_of_points; i++){
//         // memset(buffer, 0, sizeof(buffer));
//         // message_buffer[message_length] = 0;
//         j = 0;        
//         while ((ptr != NULL) && (j < 6)){
//             position[i][j] = atof(ptr);
//             ptr = strtok(NULL, this->data_sep);
//             j++;
//         }
//         j = 0;        
//         while ((ptr != NULL) && (j < 6)){
//             velocities[i][j] = atof(ptr);
//             ptr = strtok(NULL, this->data_sep);
//             j++;
//         }
//     }
//     free(message_buffer);
//     return no_of_points;
// }

// bool Comm::client_available(){
//     this->client = this->wifiServer->available();
//     return this->client;
// }
// bool Comm::client_connected(){return this->client.connected();}
// bool Comm::check_incomming_data(){return (this->client.available()>0);}
// void Comm::stop_client(){this->client.stop();}

// void Comm::send_message(const char type){
//     uint8_t sb[4];

//     if(type == this->JOINT_POSITION_SET){
//         sb[0] = (char) this->msgs_length;
//         sb[1] = (char) (this->msgs_length>>8);
//         sb[2] = (char) (this->msgs_length>>16);
//         sb[3] = (char) (this->msgs_length>>24);
//         if(this->client_connected()){
//             this->client.write(sb, sizeof(sb));
//             this->client.write(this->done_msg, this->msgs_length);
//         }
//     }
// }