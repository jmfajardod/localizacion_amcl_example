#!/usr/bin/env python

import rospy

from rospy.numpy_msg import numpy_msg
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Image

class Map_loader:

    #------------------------------------------------------#
    # Callback function for image
    def map_cb(self, data):

        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_res = data.info.resolution
        self.map_origin = data.info.origin

        print("Map origin")
        print(self.map_origin.position.x)
        print(self.map_origin.position.y)
        print(self.map_origin.position.z)

        print(self.map_origin.orientation.x)
        print(self.map_origin.orientation.y)
        print(self.map_origin.orientation.z)
        print(self.map_origin.orientation.w)

        self.map = data.data

        print("Map")
        print(self.map.shape)
        print(self.map)

        self.map = np.resize(self.map, [self.map_height, self.map_width])

        print(self.map.shape)
        print(self.map)

        # Create image publisher
        self.image_pub = rospy.Publisher("map_image", Image, queue_size=1)


    #------------------------------------------------------#
    #------------------------------------------------------#
    #------------------------------------------------------#

    def __init__(self):

        self.map_width = None
        self.map_height = None
        self.map_res = None
        self.map_origin = None

        self.map = None

        # Create CV bridge
        self.bridge = CvBridge()

        # Create image subscriber
        self.image_sub = rospy.Subscriber("/map", numpy_msg(OccupancyGrid), self.map_cb)
        
        self.image_pub = None

        self.spin()

    #------------------------------------------------------#
    # Spin function
    def spin(self):

        rate = rospy.Rate(30)

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer) 

        while (not rospy.is_shutdown()):

            if(self.image_pub is not None):

                cv_image = np.zeros((self.map.shape[0],self.map.shape[1],3), dtype = np.uint8)
                cv_image[:,:,2] = self.map.copy()
                cv_image = cv2.flip(cv_image, 0)

                trans = TransformStamped()
                try:
                    trans = tfBuffer.lookup_transform("map", "base_footprint", rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass

                pos_x_robot = self.map.shape[1] - (-(trans.transform.translation.x/self.map_res) - (self.map_origin.position.x/self.map_res))
                pos_y_robot = self.map.shape[0] - ((trans.transform.translation.y/self.map_res) - (self.map_origin.position.y/self.map_res))
                pos_x_robot = np.uint(pos_x_robot)
                pos_y_robot = np.uint(pos_y_robot)

                #print("Pos robot: ", pos_x_robot, "  ", pos_y_robot)

                cv_image[pos_y_robot,pos_x_robot, 0] = 100 # Hue
                cv_image[pos_y_robot,pos_x_robot, 1] = 255 # Saturation
                cv_image[pos_y_robot,pos_x_robot, 2] = 255 # Value
                cv2.circle(cv_image,(pos_x_robot,pos_y_robot), 8, (100,255,255), 1)
                
                
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)
                try:
                    #self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image2))
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                except CvBridgeError as e:
                    print(e)
                
            rate.sleep()


        print("Shutting down")
        cv2.destroyAllWindows()

#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#

if __name__ == '__main__':

    # Firt init the node and then the object to correctly find the parameters
    rospy.init_node('map_loader', anonymous=True)
    Map_loader()
    