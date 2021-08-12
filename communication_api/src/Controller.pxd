from libcpp.string cimport string
from libcpp cimport bool

cdef extern from "Controller.h":
    cdef cppclass Controller:
        float[3] current_joint_pos
        float[3] current_joint_vel
        float[3] current_joint_acc
        Controller() except +
        Controller(const char*) except +
        int update_arm_info()
        int set_angles(float[3])
        int set_angles(float[3], bool)
        int set_joints_max_speeds(float [3])
        int set_joints_i_funcs(string *)
        void close_connection()
        void join_thread()
        bool is_executing_task()
        bool fk(double [6], double [9], double [3])
        bool ik(double [9], double [3], double [6])        
        #int set_angles(float [3], float [3])