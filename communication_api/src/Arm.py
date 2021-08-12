import numpy as np
import quaternion
import math
import PyController as pyc
import time

class Arm:
    def __init__(self, port="/dev/ttyUSB0", init_time=0, \
        ifunc="linear", max_spds = [15, 15, 15, 15, 15, 15, 180], tool_offset = [0, 0, 0]) -> None:                
        self.controller = pyc.PyController(port)        
        self.tool_offset = np.array(tool_offset)
        self.controller.update_arm_info()
        self.joint_positions = np.array(self.controller.c_angles)                
                    
        self.limits = np.array([
            [-90.0, 90.0],
            [-70.0, 70.0],
            [-73.0, 73.0]
        ])        
        # Waiting arduino nano initialization
        # time.sleep(init_time)

        self.controller.set_joints_i_funcs(np.repeat(ifunc, 7))
        self.controller.set_joints_max_speeds(max_spds)

    def calculate_offset(self, offset):
        pass

    def close_gripper(self, state):
        if(state):
            self.joint_positions[6] = 90
        else:
            self.joint_positions[6] = 0
        self.set_joints_angles(self.joint_positions)

    def set_joints_ifuncs(self, ifuncs):        
        return self.controller.set_joints_i_funcs(ifuncs)

    def set_joints_max_spds(self, speeds):        
        return self.controller.set_joints_max_speeds(speeds)

    def set_joints_angles(self, angles):
        self.controller.set_angles(angles)
        self.joint_positions = np.array(self.controller.c_angles)
        return self.joint_positions

    def get_joint_angles(self):
        return self.joint_positions
    
    def fk(self, angles, radians = False):        
        if(radians):
            angles = np.degrees(angles)
        success, (eerot, eetrans) = self.controller.fk(angles)
        eerot = np.array(eerot).reshape((3,3))
        eetrans = np.array(eetrans)
        eetrans = eetrans + np.dot(eerot, self.tool_offset)
        return success, (eerot, eetrans)

    def ik(self, eerot, eetrans):
        eetrans = eetrans - np.dot(eerot, self.tool_offset)        
        return self.controller.ik(eerot.ravel(), eetrans)

    def get_ee_pose(self):
        _, (eerot, eetrans) = self.fk(self.joint_positions[:-1])        
        return (eerot, eetrans)