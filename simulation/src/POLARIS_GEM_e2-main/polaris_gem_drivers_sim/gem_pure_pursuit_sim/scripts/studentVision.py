#!/usr/bin/env python3


import time
import math
import numpy as np
import cv2
import rospy
import torch

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology
from pure_pursuit_sim import PurePursuit



class Pedestrian_Detector:
    def __init__(self, model):
        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/gem/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("pedestrian_detection/annotate", Image, queue_size=1)
        self.detected = False
        # for vehicle
        # self.H = np.array([[4.20720605e-05, 7.43238136e-04, -3.50213484e+00],
        #                    [2.66506974e-03, -4.76183696e-04, -2.54505122e+00],
        #                    [3.03630183e-05, -1.56734362e-03, 1.00000000e+00]])
        # for simulation
        self.H = np.array([[-6.11729782e-05, -4.78284415e-03, -1.61111569e+00],
                           [ 6.08247254e-03, 2.95044000e-06, -2.90884103e+00],
                           [-4.95465124e-06, -3.71270802e-03, 1.00000000e+00]])
 
        self.model = model
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.world_p = None

    def getWorldPedPos(self):
        if self.world_p is not None:
            return self.world_p
        return None

    def img_callback(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        raw_img = cv_image.copy()
        annotated_image = self.detection(raw_img)
        # cv2.imwrite('annotated_image.jpg', annotated_image)

        if annotated_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)


    def detection(self, img):
        image = img.copy()
        # model = torch.hub.load(r'/home/jdh0312/yolov5', 'custom', path=r'/home/jdh0312/gem_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/scripts/yolov5s.pt', source='local')
        # Inference
        results = self.model(image)
        # Results
        # results.print()
        pixel_coords = results.xyxy
        if len(pixel_coords[0]) > 0:
            pt = np.array([(pixel_coords[0][0][0]+pixel_coords[0][0][2])/2, pixel_coords[0][0][3], 1])
            world_p = self.H @ pt.T
            world_p /= world_p[2]
            # print("Position of the pedestrian relative to the vehicle: ", (world_p[0], world_p[1]))
            self.world_p = (world_p[0], world_p[1])
        # curr_x, curr_y, _ = controller.get_gem_pose()
        # print("CurrPos: ", (curr_x, curr_y))
        image = np.squeeze(results.render())
        # cv2.imwrite('annotated_image.jpg', image)
        # cv2.imshow('Pedestrian Detection', image)
        # cv2.waitKey(1)
        
        return image

# if __name__ == '__main__':
#     # init args
#     rospy.init_node('lanenet_node', anonymous=True)
#     pedestrian_detector()
#     while not rospy.core.is_shutdown():
#         rospy.rostime.wallsleep(0.5)
