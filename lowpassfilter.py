#!/usr/bin/env python3

import numpy as np
import rospy
from robot_vision_lectures.msg import XYZarray, SphereParams

# Initialize lists to collect points data
x_points = []
y_points = []
z_points = []

filoutx = -0.01 
filouty = -0.01
filoutz = 0.5
filoutr = 0.00005

filgain = 0.07  # learning rate, aka: alpha

# Flag variable to control pause/resume
pause_flag = False

def lowPassFilter(center, radius):  # this is the low pass filter equation
    global filoutx, filouty, filoutz, filoutr

    filinx = center[0]
    filoutx = filgain * filinx + (1 - filgain) * filoutx
    center[0] = filoutx

    filiny = center[1]
    filouty = filgain * filiny + (1 - filgain) * filouty
    center[1] = filouty

    filinz = center[2]
    filoutz = filgain * filinz + (1 - filgain) * filoutz
    center[2] = filoutz

    filinr = radius
    filoutr = filgain * filinr + (1 - filgain) * filoutr
    radius = filinr
    
    return center, radius

# Callback function to process incoming XYZ data
def xyz_callback(data):
    global x_points, y_points, z_points, fil_out_x, fil_out_y, fil_out_z, fil_out_radius, pause_flag

    if not pause_flag:
        for point in data.points:
            x_points.append(point.x)
            y_points.append(point.y)
            z_points.append(point.z)

        if len(x_points) > 1:  # we need at least 1 point for reference
            center, radius = fitting_the_sphere(x_points, y_points, z_points)
            center, radius = lowPassFilter(center, radius)
            publish_sphere_params(center, radius)

# function for fitted data
def fitting_the_sphere(X, Y, Z):
    # least square fit algorithm
    num_points = len(X)
    A = np.c_[2 * np.array(X), 2 * np.array(Y), 2 * np.array(Z), np.ones(num_points)]
    b = np.array(X)**2 + np.array(Y)**2 + np.array(Z)**2
    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    xc, yc, zc, d = x

    radius = np.sqrt(xc**2 + yc**2 + zc**2 + d)
    center = np.array([xc, yc, zc])

    print("\n\nCenter:	", center, "\nRadius:	", radius)

    return center, radius

def publish_sphere_params(center, radius):
    sphere_msg = SphereParams()
    sphere_msg.xc = center[0]
    sphere_msg.yc = center[1]
    sphere_msg.zc = center[2]
    sphere_msg.radius = radius
    pub.publish(sphere_msg)

# Function to handle pause/resume requests
def toggle_pause():
    global pause_flag
    pause_flag = not pause_flag

    if pause_flag:
        print("Paused")
    else:
        print("Resumed")

# Main loop to listen for keyboard input
def main_loop():
    print("Press 'p' to pause/resume")
    while True:
        user_input = input()
        if user_input == 'p':
            toggle_pause()

# Main ROS node setup
if __name__ == '__main__':
    rospy.init_node('lowpassfilter', anonymous=True)
    rospy.Subscriber('xyz_cropped_ball', XYZarray, xyz_callback)
    pub = rospy.Publisher("sphere_params", SphereParams, queue_size=10)
    
    # Start the main loop to listen for keyboard input
    main_loop()
