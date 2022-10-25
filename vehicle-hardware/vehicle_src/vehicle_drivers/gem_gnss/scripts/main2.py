import rospy
import math
import argparse
import torch

from gazebo_msgs.msg import  ModelState
import time

from lidarProcessing2 import LidarProcessing
from positionDetector2 import PositionDetector
from safetyDetector import SafetyDetector
from studentVision import Pedestrian_Detector

import numpy as np


def run_model():
    resolution = 0.1
    side_range = (-10, 10)
    fwd_range = (0., 25.)
    height_range = (-1, 0.5)

    # init rospy node
    rospy.init_node("model_dynamics2")
    model = torch.hub.load(r'/home/gem/team2_ws/src/vehicle_drivers/gem_gnss/scripts/yolov5', 'custom', path=r'/home/gem/team2_ws/src/vehicle_drivers/gem_gnss/scripts/yolov5s.pt',  source = 'local')
    time.sleep(10)
    pedDetectorYolo = Pedestrian_Detector(model)
    lidar = LidarProcessing(resolution=resolution, side_range=side_range, fwd_range=fwd_range, height_range=height_range)
    posDetector = PositionDetector(resolution=resolution)
    safety = SafetyDetector(15, resolution)
    rate = rospy.Rate(60)  # 100 Hz
    lidar.processLidar()
    print("ALL MODULES LOADED")
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        lidar.processLidar()
        pedImgPosition = posDetector.getPosition()
        safe, pedPosition, distance = safety.checkSafety(pedImgPosition)
        M = safety.transformation_matrix
        if M is not None:
            M_inv = np.linalg.inv(M)
            ped_world_p = pedDetectorYolo.getWorldPedPos()
            if ped_world_p is not None:
                ped_world = np.array([ped_world_p[0], ped_world_p[1], 1]).reshape((3, 1))
                ped_cam_coord = np.matmul(M_inv, ped_world)
                posDetector.setPedCamPos(ped_cam_coord[0, 0], ped_cam_coord[1, 0])

    rospy.spin()

if __name__ == "__main__":
    run_model()