from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from scipy.spatial.transform import Rotation
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# region Constants for the vehicle
WHEEL_BASE = 1.75  # meter
LOOK_AHEAD = 6.0  # meter
STEER_LIM = 0.05  # radian
K_PP = 0.285  # Proportional Gain for Pure Pursuit
K_ST = 0.45  # Proportional Gain for Stanley
CYCLE_SEC = 1  # second
SAFE_ZOOM_RADIUS = 10    # radius of the safety zoom circle
SAFE_TRACK_COLOR = (0,255,0)
PED_COLOR = (0, 0, 255)
# endregion

class predict_track():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        # self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        self.sub_bird = rospy.Subscriber("/gem/PedestrianAnnotate2", Image, self.img_callback, queue_size=1)
        #self.sub_bird = rospy.Subscriber("/gem/BirdsEye2", Image, self.img_callback, queue_size=1)
        self.pub_bird = rospy.Publisher("/gem/PredictedTrack", Image, queue_size=1)
        self.birds_eye_img = None
        self.safety = True
        self.velocity = 0
        self.steeringAngle = 0
        self.acceleration = 0

    def img_callback(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        update_image = cv_image.copy()

        if update_image is not None:
            self.birds_eye_img = update_image

    def updateMotionPara(self, update_steeringAngle, update_velocity, update_acceleration):
        self.velocity = update_velocity
        self.steeringAngle = update_steeringAngle
        self.acceleration = update_acceleration

    def quat_to_yaw(self, quat: Quaternion) -> float:
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        # ZYX is the axis convention for Euler angles. Follow REP-103
        yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler("ZYX")
        return yaw

    def euler_to_quat(self, yaw, pitch=0.0, roll=0.0) -> Quaternion:
        quat = Rotation.from_euler("ZYX", (yaw, pitch, roll)).as_quat()
        return Quaternion(*quat)

    def dynamics(self, curr_pose: Pose, steering: float, velocity : float) -> Pose:
        curr_x = curr_pose.position.x
        curr_y = curr_pose.position.y
        curr_z = curr_pose.position.z
        curr_yaw = self.quat_to_yaw(curr_pose.orientation)

        next_x = curr_x + velocity * CYCLE_SEC * np.cos(curr_yaw)
        next_y = curr_y + velocity * CYCLE_SEC * np.sin(curr_yaw)
        next_yaw = curr_yaw + velocity * np.tan(steering) / WHEEL_BASE * CYCLE_SEC

        return Pose(position=Point(next_x, next_y, curr_z), orientation=self.euler_to_quat(yaw=next_yaw))

    def drawPredictedWaypoints(self, img, waypoints):
        color = (0, 255, 0)
        thickness = 1     
        for centers in waypoints:
            coordinate = tuple(map(int, centers))
            if(self.checkSafety(img, coordinate)):
                img = cv2.circle(img, coordinate, SAFE_ZOOM_RADIUS, color, thickness)  # might have out-of-bound error?
            else:
                return img, False
        return img, True

    def drawPredictedTrack(self, img, waypoints):
        color = SAFE_TRACK_COLOR
        thickness = SAFE_ZOOM_RADIUS
        coordinate = np.array(waypoints)
        coordinate = coordinate.astype(int)
        img = cv2.polylines(img,np.int32([coordinate]),False,color,thickness)
        return img
    
    def checkSafety(self, img, pos):
        size_tuple = img.shape
        height = size_tuple[0]
        width = size_tuple[1]

        r = SAFE_ZOOM_RADIUS
        x = pos[0]
        y = pos[1]
        for j in range(-r, r):
            for i in range(-r, r):
                if 0 < y+j < height and 0 < x+i < width:
                    if (img[y + j, x + i] == PED_COLOR).all():  
                        return False
        return True

    def predictTrajectory(self, img):
        size_tuple = img.shape
        height = size_tuple[0]
        width = size_tuple[1]
        
        curr_x = 0
        curr_y = 0
        curr_z = 0
        curr_yaw = 0.0
        itr = 0

        y_pixel = height - curr_x - 1
        x_pixel = width/2 - curr_y - 1
        itr_steeringAngle = self.steeringAngle
        itr_velocity = self.velocity
        itr_acceleration = self.acceleration

        predicted_waypoints = []

        if abs(itr_steeringAngle) > STEER_LIM:
            if itr_steeringAngle > 0:
                itr_steeringAngle = STEER_LIM
            else:
                itr_steeringAngle = -STEER_LIM 

        while 0 < y_pixel < height and 0 < x_pixel < width and itr < 40 and self.velocity > 0:
            # append current position at the end of the list
            predicted_waypoints.append([x_pixel, y_pixel])
            curr_pose = Pose(position=Point(curr_x, curr_y, curr_z), orientation=self.euler_to_quat(yaw=curr_yaw))
            next_pose = self.dynamics(curr_pose, itr_steeringAngle, itr_velocity)
            curr_x = next_pose.position.x
            curr_y = next_pose.position.y
            curr_z = next_pose.position.z
            curr_yaw = self.quat_to_yaw(next_pose.orientation)
            
            itr += 1
            itr_velocity += itr_acceleration
            y_pixel = height - curr_x - 1
            x_pixel = width/2 - curr_y - 1

        img, safety = self.drawPredictedWaypoints(img, predicted_waypoints)
        print(safety)
        cv2.imshow('image', img)
        cv2.waitKey(1)
        return img, safety

    def publishTrajectory(self):
        if(self.birds_eye_img is not None):
            img, safety = self.predictTrajectory(self.birds_eye_img)
            self.pub_bird.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
            return safety
        else:
            return True



if __name__ == "__main__":
    """
    parser = argparse.ArgumentParser(description = 'Running test')

    velocity_default = 15
    steeringAngle_default = 5
    acceleration_default = 5

    parser.add_argument('--velocity', type = float, help = 'velocity', default = velocity_default)
    parser.add_argument('--steeringAngle', type = float, help = 'steeringAngle', default = steeringAngle_default)
    parser.add_argument('--acceleration', type = float, help = 'acceleration', default = acceleration_default)

    argv = parser.parse_args()

    velocity = argv.velocity
    steeringAngle = argv.steeringAngle
    acceleration = argv.acceleration
    """
    velocity_default = 15
    steeringAngle_default = 0.01        # in radian
    acceleration_default = 2

    wrap = safety_detector()
    wrap.updateMotionPara(steeringAngle_default, velocity_default, acceleration_default)
    img = cv2.imread('/home/lyuxinghe/Desktop/test.png')
    waypoints = wrap.predictTrajectory(img)
    cv2.imwrite('/home/lyuxinghe/Desktop/out.png',img)
