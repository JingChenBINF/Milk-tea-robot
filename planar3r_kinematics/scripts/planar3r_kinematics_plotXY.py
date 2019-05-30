#!/usr/bin/env python 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from sensor_msgs.msg import JointState
import tf
import math
import time

plt.ioff()
fig = plt.figure(figsize=(5,40))
ax = plt.subplot(2,1,1)
ax1 = plt.subplot(2,1,2)
ax.set_title('XY-position of end-effector')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.axis('scaled')
ax.axis([-9, 9, -9, 9])
# plt.tight_layout()
ax1.set_title('Configuration')
ax1.set_xlabel('Joint 2 (degrees)')
ax1.set_ylabel('Joint 3 (degrees)')
ax1.axis('scaled')
ax1.axis([-185, 185, -185, 185])
# fig.tight_layout()
x = []
y = []
ang1 = []
ang2 = []
line, = ax.plot(x, y, '-')
line_q, = ax1.plot(ang1, ang2, '-')

fig.canvas.flush_events()
fig.canvas.draw()
fig.show()
# curr_ang1 = None;
# curr_ang2 = None;
global count
count = 0
global BUFFER_SIZE
BUFFER_SIZE = 1000
global listener


def updateXY(msg):
    global count
    global listener
    global x, y, ax, fig
    global ang1, ang2, ax1
    count = count + 1;
    curr_ang1 = msg.position[1] * 180 / math.pi
    curr_ang2 = msg.position[2] * 180 / math.pi
    ang1 = np.append(ang1, curr_ang1)
    ang2 = np.append(ang2, curr_ang2)
    try:
        (trans,rot) = listener.lookupTransform("/base_link", "/endEffector", rospy.Time(0))
        curr_x = trans[0]
        curr_y = trans[1]
        x = np.append(x, curr_x)
        y = np.append(y, curr_y)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        print("TF exception: " + str(e))

    if count > BUFFER_SIZE:
        x = x[-BUFFER_SIZE:];
        y = y[-BUFFER_SIZE:];
        ang1 = ang1[-BUFFER_SIZE:];
        ang2 = ang2[-BUFFER_SIZE:];
        count = BUFFER_SIZE;
    line.set_xdata(x)
    line.set_ydata(y)
    line_q.set_xdata(ang1)
    line_q.set_ydata(ang2)
    plt.pause(0.0000001)

if __name__ == '__main__':

    # ani = animation.FuncAnimation( fig, animate, init_func=init, interval=2, blit=True, save_count=50)

    # To save the animation, use e.g.
    #
    # ani.save("movie.mp4")
    #
    # or
    #
    # from matplotlib.animation import FFMpegWriter
    # writer = FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    # ani.save("movie.mp4", writer=writer)
    count = 0

    rospy.init_node("XYplotter")
    global listener
    listener = tf.TransformListener()
    # time.sleep(10)

    rospy.Subscriber("/joint_states", JointState, updateXY)
    plt.show()
