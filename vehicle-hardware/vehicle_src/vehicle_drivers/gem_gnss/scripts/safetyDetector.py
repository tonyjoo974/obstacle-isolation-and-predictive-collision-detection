import numpy as np
import rospy

from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

class SafetyDetector:
    def __init__(self, safetyDistance, resolution):
        self.__safetyDistance = safetyDistance
        self.__resolution = resolution
        self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0
        self.transformation_matrix = np.array([[resolution*np.cos(- 1.57), -1*resolution*np.sin(- 1.57), 0.0],
                                               [resolution*np.sin(- 1.57), resolution*np.cos(- 1.57),  0.0],
                                               [0,          0,           1]])

    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude  # latitude
        self.lon     = inspva_msg.longitude # longitude
        self.heading = inspva_msg.azimuth   # heading in degrees
    
    def heading_to_yaw(self, heading_curr):
        # 0   <= heading < 90  --- 90 to 0     (pi/2 to 0)
        # 90  <= heading < 180 --- 0 to -90    (0 to -pi/2)
        # 180 <= heading < 270 --- -90 to -180 (-pi/2 to -pi)
        # 270 <= heading < 360 --- 180 to 90   (pi to pi/2)
        if (heading_curr >= 0 and heading_curr < 90):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 90 and heading_curr < 180):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 180 and heading_curr < 270):
            yaw_curr = np.radians(90 - heading_curr)
        else:
            yaw_curr = np.radians(450 - heading_curr)
        return yaw_curr

    def checkSafety(self, pedestrianPosition):
        """
            Checks if it is safe for the car to move if a pedestrian is at the
            position specified in the parameter passed in.

            Input: carState - state of the car
                   pedestrianPosition - pixel location of the pesdestrian in the
                                        birds eye view image.

            Output: (safe, world_ped_pos)
                    safe - if the car can move forward safely
                    world_ped_pos - world coordinates of the pedestrian
        """
        # if ped distance is 0 - no pedestrian was detected
        if pedestrianPosition == 0:
            return True, 0, None

        world_car_pos = np.array([0.0, 0.0])

        # Calculate the position of the pedestrian in the world cooridinate system

        # Let the world cooridnate system be X, Y at origin O
        # Let the image coordinate system be x, y at origin o

        # Here, the image and the image coordinate frame are different
        #   - the image has pixel positions

        # yaw gives us the angle between X and y
        # _, _, yaw = self.quaternion_to_euler(carState.pose.orientation.x,
        #                                     carState.pose.orientation.y,
        #                                     carState.pose.orientation.z,
        #                                     carState.pose.orientation.w)
        # yaw = self.heading_to_yaw(self.heading)
        # yaw = round(yaw, 4)
        yaw = 0
        # the angle between x and X is therefore:
        theta = yaw - 1.57

        world_ped_pos = self.apply_transformation(pedestrianPosition, world_car_pos,
                                                theta, self.__resolution)
        print("world_ped_pos from Lidar Birds Eye: ", world_ped_pos)
        distance = np.linalg.norm(world_car_pos - np.array(world_ped_pos))
        # print("distance to pedestrian",distance,distance >= self.__safetyDistance)
        return distance >= self.__safetyDistance, world_ped_pos, distance


    def apply_transformation(self, ped_pos_img, car_pos_world, theta, scale):
        """
        Transforms the given coordinates from the image coordinate system to the
        world coordinate system

        Let the world cooridnate system be X, Y at origin O
        Let the image coordinate system be x, y at origin o

        Input: ped_pos_img: position of the pedestrian in the image coordinate system
               car_pos_world: position of the car in the world coordicate system;
                        also the position of the o in the world coordinate system
               theta: angle of rotation between x and X
               scale: 1 unit in image coordinates is equal to scale units in the world coordinate system

        Output: (ped_X, ped_Y): position of the pedestrian in the world coordinate system

        """

        # The transformation here is an affine transformation
        # first we create the transformation matrix

        pos = np.array([ped_pos_img[0], ped_pos_img[1], 1]).reshape((3, 1))

        transformation = np.array([[scale*np.cos(theta), -1*scale*np.sin(theta), car_pos_world[0]],
                                  [scale*np.sin(theta), scale*np.cos(theta),  car_pos_world[1]],
                                  [0,          0,           1]])

        ped_pos_world = np.matmul(transformation, pos)

        return ped_pos_world[0, 0], ped_pos_world[1, 0]

    def quaternion_to_euler(self, x, y, z, w):
        """
            Converts quaternion to euler angles.

            Input: quaternion
            output: (roll, pitch, yaw)
        """

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]