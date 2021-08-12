# distutils: language = c++
from libcpp.string cimport string
from Controller cimport Controller


# Create a Cython extension type which holds a C++ instance
# as an attribute and create a bunch of forwarding methods
# Python extension type.
cdef class PyController:
    cdef Controller ccontroller  # Hold a C++ instance which we're wrapping

    def __cinit__(self, port):
        self.ccontroller = Controller(port.encode('utf-8'))

    def update_arm_info(self):                     
        return self.ccontroller.update_arm_info()        

    def set_angles(self, angles, blocking = True):
        cdef float args[7]
        args[0] = angles[0]
        args[1] = angles[1]
        args[2] = angles[2]        
        args[3] = angles[3]        
        args[4] = angles[4]        
        args[5] = angles[5]        
        args[6] = angles[6]                
        return self.ccontroller.set_angles(args, blocking)        

    def set_joints_max_speeds(self, speeds):
        cdef float args[7]
        args[0] = speeds[0]
        args[1] = speeds[1]
        args[2] = speeds[2]
        args[3] = speeds[3]
        args[4] = speeds[4]
        args[5] = speeds[5]
        args[6] = speeds[6]
        return self.ccontroller.set_joints_max_speeds(args)

    def set_joints_i_funcs(self, func_names):
        cdef string args[7]
        args[0] = func_names[0].encode('utf-8')
        args[1] = func_names[1].encode('utf-8')
        args[2] = func_names[2].encode('utf-8')
        args[3] = func_names[3].encode('utf-8')
        args[4] = func_names[4].encode('utf-8')
        args[5] = func_names[5].encode('utf-8')
        args[6] = func_names[6].encode('utf-8')
        return self.ccontroller.set_joints_i_funcs(args)

    def close_connection(self):
        self.ccontroller.close_connection()

    def is_executing_task(self):
        return self.ccontroller.is_executing_task()

    def join_thread(self):
        return self.ccontroller.join_thread()

    def fk(self, joint_angles):
        cdef double args[6]
        cdef double eetrans[3]
        cdef double eerot[9]
        # cdef bool success
        args[0] = joint_angles[0]
        args[1] = joint_angles[1]
        args[2] = joint_angles[2]
        args[3] = joint_angles[3]
        args[4] = joint_angles[4]
        args[5] = joint_angles[5]

        success = self.ccontroller.fk(args, eerot, eetrans)
        return (success, (eerot, eetrans))

    def ik(self, eerot, eetrans):
        cdef double rotargs[9]
        cdef double transargs[3]
        cdef double solution[6]
        
        for i in range(9):
            rotargs[i] = eerot[i]
        for i in range(3):
            transargs[i] = eetrans[i]        

        success = self.ccontroller.ik(rotargs, transargs, solution)
        return (success, solution)

    # Attribute access
    @property
    def c_angles(self):
        return (self.ccontroller.current_joint_pos[0], 
                self.ccontroller.current_joint_pos[1], 
                self.ccontroller.current_joint_pos[2], 
                self.ccontroller.current_joint_pos[3], 
                self.ccontroller.current_joint_pos[4], 
                self.ccontroller.current_joint_pos[5], 
                self.ccontroller.current_joint_pos[6])
    # Attribute access
    @property
    def c_velocities(self):
        return (self.ccontroller.current_joint_vel[0], 
                self.ccontroller.current_joint_vel[1], 
                self.ccontroller.current_joint_vel[2], 
                self.ccontroller.current_joint_vel[3], 
                self.ccontroller.current_joint_vel[4], 
                self.ccontroller.current_joint_vel[5], 
                self.ccontroller.current_joint_vel[6])
    # Attribute access
    @property
    def c_accelerations(self):
        return (self.ccontroller.current_joint_acc[0], 
                self.ccontroller.current_joint_acc[1], 
                self.ccontroller.current_joint_acc[2], 
                self.ccontroller.current_joint_acc[3], 
                self.ccontroller.current_joint_acc[4], 
                self.ccontroller.current_joint_acc[5], 
                self.ccontroller.current_joint_acc[6])
    # @x0.setter
    # def x0(self, x0):
    #     self.c_rect.x0 = x0
