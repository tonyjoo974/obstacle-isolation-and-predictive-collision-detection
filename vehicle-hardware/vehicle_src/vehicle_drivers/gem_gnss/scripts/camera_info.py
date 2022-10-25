#!/usr/bin/env python3


import time
import math
import numpy as np
import cv2
import rospy
import torch

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
# from skimage import morphology



class pedestrian_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        # self.sub_image = rospy.Subscriber('/gem/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # self.sub_image = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.img_callback)
        # self.pub_image = rospy.Publisher("pedestrian_detection/annotate", Image, queue_size=1)
        # self.detected = False
        self.sub_info = rospy.Subscriber('/mako_1/mako_1/camera_info', CameraInfo, self.img_callback, queue_size=1)


    def img_callback(self, camera_info):
        camera_info_K = np.array(camera_info.K).reshape([3, 3])
        camera_info_R = np.array(camera_info.R).reshape([3, 3])
        camera_info_P = np.array(camera_info.P).reshape([3, 4])
        print(camera_info_P)
        # K = [[1.45935026e+03 0.00000000e+00 9.36395833e+02]
        #     [0.00000000e+00 1.45452325e+03 6.93027537e+02]
        #     [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
        # R = [[ 0.99302996  0.01100924  0.11734688]
        #      [-0.00982597  0.99989493 -0.01065731]
        #      [-0.11745188  0.00942998  0.9930338 ]]
        # P = [[ 2.04634040e+03  0.00000000e+00  7.20577023e+02 -2.97320048e+02]
             # [ 0.00000000e+00  2.04634040e+03  7.10324986e+02  0.00000000e+00]
             # [ 0.00000000e+00  0.00000000e+00  1.00000000e+00  0.00000000e+00]]

    def detection(self, img):
        cv2.imwrite('calib_img.jpg', img)

        #model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=False)
        #model.load_state_dict(torch.load('/master/yolov5-master/hubconf.py'))
        # model = torch.hub.load(r'/home/gem/team2_ws/src/vehicle_drivers/gem_gnss/scripts/yolov5', 'custom', path=r'/home/gem/team2_ws/src/vehicle_drivers/gem_gnss/scripts/yolov5s.pt',  source = 'local')
        # Inference
        # /home/gem/demo_ws/src/vehicle_drivers/gem_vision/scripts/yolov5
        # /home/gem/demo_ws/src/vehicle_drivers/gem_vision/scripts/yolov5/yolov5s.pt
        # results = model(img)

        # # Results
        # results.print()

        # image = np.squeeze(results.render())
        # dim = (854,480)
        # image = cv2.resize(image, dim)
        # cv2.imshow('Pedestrian Detection', image)
        # cv2.waitKey(1)

        return img

if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    pedestrian_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
