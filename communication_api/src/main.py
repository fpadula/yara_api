import numpy as np
import quaternion
from Arm import Arm
import time
import yaml
from Xbox360Gamepad import Xbox360Gamepad
from math import fabs

def map(value, from1, to1, from2, to2):
    return value*((to1 - to2)/(from1 - from2)) + (-to1*from2 + from1 * to2)/(from1 - from2)

def button_press(action_array, prev_action_array, index):
    return (action_array[index] == 1) and (prev_action_array[index]==0)

def main():        
    arm = Arm("/dev/ttyUSB0", ifunc="linear", max_spds = [15, 15, 15, 25, 25, 25, 180],
                tool_offset=np.array([0, 0.08103, 0.00156]))    

    possible_trans_speeds = [0.005, 0.001]
    curr_spd_ind = 0
    trans_speed = possible_trans_speeds[curr_spd_ind]
    rot_speed = 1
    eerot, eetrans = arm.get_ee_pose()   
    ee_quat = quaternion.from_rotation_matrix(eerot)
    
    if(fabs(eetrans[0]) < 0.001):
        eetrans[0] = 0.001    
    with open("./src/xbox.yaml") as file:
            key_mappings = yaml.load(file, Loader=yaml.FullLoader)    
    gamepad = Xbox360Gamepad(key_mappings.keys())
    read_gamepad = True
    waypoints = []
    action = np.zeros(len(key_mappings))
    while(read_gamepad):
        prev_action = action
        action = np.zeros(len(key_mappings))
        for key_name in key_mappings:
            key = key_mappings[key_name]
            raw_value = gamepad.read_key_state(key_name)
            if(abs(raw_value) <= key["deadzone"]):
                raw_value = 0
            action[key["action_index"]] = map(raw_value, key["src_min"], key["dest_min"], key["src_max"], key["dest_max"])
        
        action[2] -= action[3]
        action[9] -= action[10]
        eetrans += trans_speed * action[:3].round(4)
        rot_action = rot_speed * action[7:10].round(4)
        rot_action = rot_action[ [1, 2, 0] ]
        # print(rot_action)
        ee_quat = quaternion.from_rotation_vector(np.deg2rad(rot_action))*ee_quat        
        if button_press(action, prev_action, 4):
            arm.close_gripper(True)
        if button_press(action, prev_action, 5):
            arm.close_gripper(False)
        if button_press(action, prev_action, 6):
            read_gamepad = False
        if button_press(action, prev_action, 11):
            _, (eerot, eetrans) = arm.fk(np.repeat(0, 6))  
            ee_quat = quaternion.from_rotation_matrix(eerot)
            eetrans[0] = 0.001
        if button_press(action, prev_action, 12):
            action[12] = 0
            waypoints.append( (eetrans, np.rad2deg(quaternion.as_rotation_vector(ee_quat)).round(4), arm.get_joint_angles()))
        if button_press(action, prev_action, 13):
            curr_spd_ind += 1
            if curr_spd_ind >= len(possible_trans_speeds):
                curr_spd_ind = 0
            trans_speed = possible_trans_speeds[curr_spd_ind]
            print(trans_speed)

        # print(action, ", ", eetrans)
        found_sol, angles = arm.ik(quaternion.as_rotation_matrix(ee_quat), eetrans)
        if(found_sol):
            # print("Solution found:")
            angles = list(angles)+[arm.joint_positions[-1]]
            print("Position:", eetrans.round(4), ", rotation:", np.rad2deg(quaternion.as_rotation_vector(ee_quat)).round(4))
            arm.set_joints_angles(angles)
        else:
            print("Solution not found") 
        time.sleep(0.05) 
    gamepad.stop()
    print("Waypoints:")
    arm.set_joints_ifuncs(["inout_cubic", "inout_cubic", "inout_cubic", "inout_cubic", "inout_cubic", "inout_cubic", "linear"])
    for point in waypoints:
        print("Position:", point[0], ", rotation:", point[1], ", angles:", point[2])
        arm.set_joints_angles(point[2])
    # jcfgs = np.array([
    #     [0, 0, 0, 0, 0, 0, 0],
    #     [45, -45, 45, 45, -45, 45, 0],
    #     [45, -45, 45, -45, 45, -45, 0],
    #     [-45, -45, -45, -45, -45, -45, 0],
    #     [45, 45, 45, 45, 45, 45, 0]
    # ])
    # for j in jcfgs:
    #     angles = j
    #     print(np.round(angles, 4))
    #     arm.set_joints_angles(angles)
    # return

    # found_sol, (eerot, eetrans) = arm.fk(np.repeat(0, 7))
    # if(found_sol):
    #     print("Solution found:")
    #     eerot = np.array(eerot)
    #     eetrans = np.array(eetrans)
    #     print(eerot.round(4))
    #     print(eetrans.round(4))
    
    # eetrans[0] = 1e-6

    # for z in np.arange(eetrans[2], eetrans[2]-0.20, -0.05):
    #     t = [eetrans[0], eetrans[1], z]
    #     print(t)
    #     found_sol, angles = arm.ik(eerot, t)
    #     if(found_sol):
    #         print("Solution found:")
    #         angles = list(angles)        
    #         print(np.round(angles, 2))
    #         arm.set_joints_angles(angles + [0])
    #     else:
    #         print("Solution not found") 
    # arm.set_joints_angles(np.repeat(0, 7))


if __name__ == "__main__":
    main()