#include <stdio.h>
#include <stdlib.h>

#include "Controller.h" // write(), read(), close()


int main(){
    char cmd;
    float t_pos[7], t_spds[7], single_joint;
    int single_joint_no;
    double eepos[3], eerot[9], joint_angles[6];
    bool run, verbose;
    std::string sts[7];

    // return 0;
    Controller c = Controller("/dev/ttyUSB0");
    run = true;
    verbose = true;
    c.set_verbose(verbose);
    while(run){
        printf(">");
        scanf(" %c", &cmd);
        switch (cmd){
            case 't':
                // Reading target angles
                scanf(" %f %f %f %f %f %f %f", &t_pos[0], &t_pos[1], &t_pos[2],
                        &t_pos[3], &t_pos[4], &t_pos[5], &t_pos[6]);
                if(verbose)
                    printf("Sending move request to arm.\n");

                if(c.set_angles(t_pos) != 0)
                    printf("Error!\n");
                else{
                    if(verbose)
                        printf("Done!\n");
                }
                break;
            case 'j':
                // Reading target angles
                scanf(" %d %f", &single_joint_no, &single_joint);
                for(int i = 0; i < 7; i++){
                    if(i == single_joint_no){
                        t_pos[i] = single_joint;
                    }
                    else{
                        t_pos[i] = c.current_joint_pos[i];
                    }
                }

                if(verbose)
                    printf("Sending move request to arm.\n");

                if(c.set_angles(t_pos) != 0)
                    printf("Error!\n");
                else{
                    if(verbose)
                        printf("Done!\n");
                }
                break;
            case 'b':
                // Reading target angles
                scanf(" %f %f %f %f %f %f %f", &t_pos[0], &t_pos[1], &t_pos[2],
                        &t_pos[3], &t_pos[4], &t_pos[5], &t_pos[6]);
                if(verbose)
                    printf("Sending non blocking move request to arm.\n");
                c.set_angles(t_pos, false);
                while(c.is_executing_task()){
                    printf("\tCurrent joint positions: %.2f, %.2f, %.2f\n",
                            c.current_joint_pos[0], c.current_joint_pos[1],
                            c.current_joint_pos[2]);
                    usleep(0.25 * 1000000);
                }
                c.join_thread();
                break;
            case 's':
                scanf(" %f %f %f %f %f %f %f", &t_spds[0], &t_spds[1], &t_spds[2],
                        &t_spds[3], &t_spds[4], &t_spds[5], &t_spds[6]);
                if(verbose)
                    printf("Setting joints max speeds to: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", t_spds[0], t_spds[1],
                            t_spds[2], t_spds[3], t_spds[4], t_spds[5],
                            t_spds[6]);
                if(c.set_joints_max_speeds(t_spds) != 0)
                    printf("Error!\n");
                else{
                    if(verbose)
                        printf("Done!\n");
                }
                break;
            case 'i':
                std::cin >> sts[0] >> sts[1] >> sts[2] >> sts[3] >> sts[4]
                            >> sts[5] >> sts[6];
                if(verbose)
                    std::cout << "Setting joints interpolation functions to: "
                                << sts[0] << ", " << sts[1] << ", " << sts[2]
                                << sts[3] << ", " << sts[4] << ", " << sts[5]
                                << ", " << sts[6] << std::endl;
                if(c.set_joints_i_funcs(sts) != 0)
                    printf("Error!\n");
                else{
                    if(verbose)
                        printf("Done!\n");
                }
                break;
            case 'f':
                c.update_arm_info();
                printf("\tJoint positions:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
                        c.current_joint_pos[0], c.current_joint_pos[1],
                        c.current_joint_pos[2], c.current_joint_pos[3],
                        c.current_joint_pos[4], c.current_joint_pos[5],
                        c.current_joint_pos[6]);                        
                printf("\tJoint velocities:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
                        c.current_joint_vel[0], c.current_joint_vel[1],
                        c.current_joint_vel[2], c.current_joint_vel[3],
                        c.current_joint_vel[4], c.current_joint_vel[5],
                        c.current_joint_vel[6]);                        
                printf("\tJoint accelerations:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
                        c.current_joint_acc[0], c.current_joint_acc[1],
                        c.current_joint_acc[2], c.current_joint_acc[3],
                        c.current_joint_acc[4], c.current_joint_acc[5],
                        c.current_joint_acc[6]);
                break;
            case 'k':
                scanf("  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf", &eepos[0], &eepos[1], &eepos[2],
                        &eerot[0], &eerot[1], &eerot[2], &eerot[3], &eerot[4], &eerot[5],
                        &eerot[6], &eerot[7], &eerot[8]);
                if(verbose)
                    printf("Requested pos ( %lf,  %lf,  %lf) with orientation ( ( %lf,  %lf,  %lf), ( %lf,  %lf,  %lf), ( %lf, %lf,  %lf) ).\n",  eepos[0],  eepos[1],  eepos[2],
                         eerot[0],  eerot[1],  eerot[2],  eerot[3],  eerot[4],  eerot[5],
                         eerot[6],  eerot[7],  eerot[8]);
                if(c.ik(eerot, eepos, joint_angles)){
                    for(int i = 0; i < 6; i++)
                        t_pos[i] = (float) joint_angles[i];
                    t_pos[6] = c.current_joint_pos[6];

                    if(c.set_angles(t_pos) != 0)
                        printf("Error!\n");
                    else{
                        if(verbose)
                            printf("Done!\n");
                    }
                }
                else{
                    printf("Solution not found\n");
                }
                break;
            case 'h':
                printf("h\t\t Display this help message\n");
                printf("t A1 A2 A3 A4 A5 A6 A7\t Move arm joints to position (A1, A2, A3, A4, A5, A6) and gripper to (A7). Angles are between -90 and 90 \n");
                printf("s S1 S2 S3 S4 S5 S6 S7\t Set arm joint's max speed to (S1, S2, S3, S4, S5, S6, S7). Speeds are in degrees/s\n");
                printf("f\t\t Get current joint info\n");
                printf("v\t\t Toggle verbose (default on)\n");
                printf("q\t\t Exit program\n");
                break;
            case 'v':
                verbose = !verbose;
                printf("Verbose mode: %s\n", verbose? "on": "off");
                break;
            case 'q':
                run = false;
                if(verbose)
                    printf("Exiting...\n");
                break;
            default:
                if(cmd != '\n')
                    printf("Command %c not recognized.\n", cmd);
                break;
        }
    }
    c.close_connection();
    return 0;
}