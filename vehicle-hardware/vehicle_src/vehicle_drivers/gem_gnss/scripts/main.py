import rospy
import math
import argparse

from gazebo_msgs.msg import  ModelState
import time

from lidarProcessing import LidarProcessing
from positionDetector import PositionDetector

import numpy as np



def run_model():
    resolution = 0.1
    side_range = (-10, 10)
    fwd_range = (0., 25.)
    height_range = (-1.5, 0.5)

    # init rospy node
    rospy.init_node("model_dynamics")

    lidar = LidarProcessing(resolution=resolution, side_range=side_range, fwd_range=fwd_range, height_range=height_range)
    posDetector = PositionDetector(resolution=resolution)


    rate = rospy.Rate(100)  # 100 Hz
    lidar.processLidar()
    
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        lidar.processLidar()
        pedImgPosition = posDetector.getPosition()

    rospy.spin()

if __name__ == "__main__":
    run_model()
