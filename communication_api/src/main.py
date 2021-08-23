import numpy as np
import quaternion
from Arm import Arm
import time
from math import fabs

def main():        
    arm = Arm("/dev/ttyUSB0", ifunc="linear", max_spds = [15, 15, 15, 25, 25, 25, 180],
                tool_offset=np.array([0, 0.08103, 0.00156]))    
    waypoints = [
        ( [0.007, 0.18, 0.025], [ [1, 0, 0], [0, 0, 1], [0, -1, 0] ], "close" )
        # ( [0.001, 0.23, 0.025], [ [1, 0, 0], [0, 0, 1], [0, -1, 0] ], "close" ),        
        # ( [0.001, 0.23, 0.15], [ [1, 0, 0], [0, 1, 0], [0, 0, 1] ], "close" )       
    ]

    for point in waypoints:
        position, orientation, gripper_state = np.array(point[0]), np.array(point[1]), point[2]
        print("Current point:")
        print("\t", position)
        print("\t", orientation.ravel())
        found_sol, angles = arm.ik(orientation, position)
        if(found_sol):
            print("Moving joint angles to:", angles)
            angles = list(angles)+[arm.joint_positions[-1]]
            arm.set_joints_angles(angles)
            arm.close_gripper(gripper_state == "close")
        else:
            print("Solution not found!")
        time.sleep(3)



if __name__ == "__main__":
    main()