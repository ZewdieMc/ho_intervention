#!/usr/bin/env python3
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from numpy import sin, cos

class ArucoPose:
    def __init__(self):
        """
        Constructor for the ArucoPose class
        """
        # robot pose
        self.eta = np.zeros(3).reshape(-1, 1)

        # camera intrinsic parameters
        self.camera_matrix = None

        # camera distortion coefficients
        self.dist_coeffs = None

        # aruco dictionary name
        self.dict_name = 'DICT_ARUCO_ORIGINAL'

        # aruco dictionary
        self.dictionary = self.getDictionary(self.dict_name)

        # marker size
        self.marker_size = 0.05

        # bridge object
        self.bridge = CvBridge()

        #detector parameters
        self.detector_params = cv2.aruco.DetectorParameters()

        # PUBLISERS
        # publish pose of aruco marker in the world frame
        self.pose_pub = rospy.Publisher('/aruco_pose', PoseStamped, queue_size=10)
        #!publish image with aruco marker
        self.image_pub = rospy.Publisher('/aruco_image', Image, queue_size=10)

        # SUBSCRIBERS
        # subscribe to the camera image
        self.image_sub = rospy.Subscriber('/turtlebot/kobuki/realsense/color/image_color', Image, self.image_callback, queue_size=1)
        # subscribe to the camera info
        self.camera_info_sub = rospy.Subscriber('/turtlebot/kobuki/realsense/color/camera_info', CameraInfo, self.info_callback, queue_size=1)

        # subscribe to the odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)


    def odom_callback(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]).reshape(-1, 1)
    
    def info_callback(self, msg):
        """
        Callback function for the camera info subscriber
        """
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        """
        Callback function for the image subscriber
        """
        # Convert the image message to a cv image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect the aruco markers
        self.estimateArucoPose(cv_image)


    def getDictionary(self, dict_name):
        aruco_dictionaries = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }

        dictionary = aruco_dictionaries[dict_name]
        
        return cv2.aruco.getPredefinedDictionary(dictionary)

    def estimateArucoPose(self, cv_image):
        """
        Estimate the pose of the aruco marker in the image
        """

        # detect the aruco markers
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.dictionary)

        # if there are markers detected
        if ids is not None:
            #draw the markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # estimate the pose of the markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            # draw marker axes
            for i in range(len(rvecs)):
                # cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                self.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                # Convert tvecs to translation vector
                tc = tvecs[i].reshape(-1, 1)

                wc = self.transform_arcuo_pose(tc)
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()#! check
                pose.header.frame_id = 'world_ned'#! check
                pose.pose.position.x = wc[0]
                pose.pose.position.y = wc[1]
                pose.pose.position.z = wc[2]
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = 1

                # Publish the pose of aruco marker
                self.pose_pub.publish(pose)

                #acuco pose in the world frame
                rospy.loginfo('Aruco pose in the world frame: {}'.format(wc))
                #! rospy.loginfo('Aruco pose in the camera frame: {}'.format(tc))

                            # Display the image
            cv2.imshow("Image", cv_image)

            # wait for key press
            cv2.waitKey()


    def transform_arcuo_pose(self, tc):
        """
        Transform the aruco pose to the world frame
        """
        r = self.eta
        wTr = np.array(
            [[cos(r[2,0]), -sin(r[2,0]), 0, r[0,0]], 
             [sin(r[2,0]), cos(r[2,0]), 0, r[1,0]] , 
             [0, 0, 1, 0], 
             [0, 0, 0, 1]])
        # Robot_base-to-camera transformation
        rTc = np.array(
                [[0.0, 0.0, 1.0, 0.136],
                 [1.0, 0.0, 0.0, -0.033],
                 [0.0, 1.0, 0.0, -0.116],
                 [0.0, 0.0, 0.0, 1.0]])
        
        wTc = wTr @ rTc

        wc = wTc @ np.vstack([tc, 1])

        return wc[:3]

    def drawAxis(self, image, camera_matrix, dist_coeffs, rvec, tvec, length):
        # Define the 3D points of the axes
        axis_points = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]]).reshape(-1,3)

        # Project the 3D points to image plane
        img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)

        # Convert image points to integer
        img_points = np.int32(img_points).reshape(-1, 2)

        # Draw axes lines on the image
        # The coordinate system: x-axis (red), y-axis (green), z-axis (blue)
        cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, length)


if __name__ == '__main__':
    rospy.init_node('aruco_pose_node')
    rospy.loginfo('Aruco pose node started')
    aruco_pose = ArucoPose()
    rospy.spin()