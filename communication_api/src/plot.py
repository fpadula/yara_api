from numpy.core.fromnumeric import shape
import PyController as pyc
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import collections

def main():

    c = pyc.PyController("/dev/ttyUSB0")
    # Waiting arduino nano initialization
    time.sleep(3)
    c.set_joints_i_funcs(["bounce_out", "inout_cubic", "inout_cubic"])
    c.set_joints_max_speeds([15, 15, 15])

    angles = []
    times = []
    speeds = [0]
    c.set_angles([45, 0, 0], False)
    start = time.time()
    while(c.is_executing_task()):
        angles.append(c.c_angles[0])        
        end = time.time()
        times.append(end - start)
        if(len(angles) > 1):
            speeds.append((angles[-1] - angles[-2])/(times[-1] - times[-2]))
        # start = time.time()
        time.sleep(.025)
    c.join_thread()
    # c.set_angles([0,0,0])
    angles = np.array(angles)
    times = np.array(times)
    speeds = np.array(speeds)
    angles, angles_mask = np.unique(angles, return_index=True)
    times = times[angles_mask]
    speeds = speeds[angles_mask]
    setpoint = np.repeat(45, times.shape[0])
    # print(angles)    
    # print(times)    
    fig = plt.figure(figsize=(10,5))
    plt.subplot(1, 2, 1)
    plt.plot(times, setpoint, 'ro', label="Setpoint")
    plt.plot(times, angles, 'go', label="Joint pos.")
    ax = fig.gca()
    ax.set_title('Joint 0 position')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (deg)')
    plt.legend(loc='lower right') # upper/lower right/left
    plt.subplot(1, 2, 2)
    plt.plot(times, speeds, 'bo')
    ax = fig.gca()
    ax.set_title('Joint 0 speed')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (deg/s)')

    plt.show()
    # c.set_

if __name__ == "__main__":
    main()