#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_sim.py                                                                  
# Description: pure pursuit controller for GEM vehicle in Gazebo                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 07/10/2021                                                                
# Date last modified: 07/15/2021                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_pure_pursuit_sim pure_pursuit_sim.py                                                                    
# Python version: 3.8                                                             
#================================================================

# Python Headers
import os 
import csv
import math
import numpy as np
from pynput import keyboard
from numpy import linalg as la

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState


class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(100)

        self.look_ahead = 6    # meters
        self.wheelbase  = 1.75 # meters
        self.goal       = 0
        self.current_key = ''
        self.read_waypoints() # read waypoints

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0 
        self.ackermann_msg.steering_angle          = 0.0

        self.ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)


    # import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/wps.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # turn path_points into a list of floats to eliminate the need for casts
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_yaw = [float(point[2]) for point in path_points]
        self.dist_arr        = np.zeros(len(self.path_points_x))

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    def get_gem_pose(self):

        rospy.wait_for_service('/gazebo/get_model_state')
        
        try:
            service_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = service_response(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q      = model_state.pose.orientation
        orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        return round(x,4), round(y,4), round(yaw,4)

    def on_press(self, key):
        try:
            self.current_key=key.char
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
            self.current_key=''


    def on_release(self, key):
        self.current_key = ''
        if key == keyboard.Key.esc:
            return False
        if key == keyboard.Key.ctrl:
            return False
        else :
            self.current_key = ''

    def start_pp(self):
        angle = 0.0
        speed = 0.0
        listener = keyboard.Listener(
        on_press=self.on_press,
        on_release=self.on_release)
        listener.start()
        while not rospy.is_shutdown():
            # print("Crosstrack Error: " + str(ct_error))
            if self.current_key == 'w':
                speed += 0.2
            elif self.current_key == 's':
                speed -= 0.2
            elif self.current_key == 'd':
                angle -= 0.05
            elif self.current_key == 'a':
                angle += 0.05

            speed = max(0, speed)
            if angle > 0.4:
                angle = 0.4
            if angle < -0.4:
                angle = -0.4
            print("Speed: ", speed)
            print("Steering Angle: ", angle)
            # implement constant pure pursuit controller
            self.ackermann_msg.speed          = speed
            self.ackermann_msg.steering_angle = angle
            self.ackermann_pub.publish(self.ackermann_msg)

            self.rate.sleep()

def pure_pursuit():

    rospy.init_node('pure_pursuit_sim_node', anonymous=True)
    pp = PurePursuit()
    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pure_pursuit()

