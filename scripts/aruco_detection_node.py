#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from numpy import sin, cos
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_euler


class ArucoPose:
    def __init__(self):
        """
        Constructor for the ArucoPose class
        """
        # Initialise TF listener
        self.tf_butter = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_butter)
        # robot pose
        self.eta = np.zeros(3).reshape(-1, 1)

        # camera intrinsic parameters
        self.camera_matrix = None

        # camera distortion coefficients
        self.dist_coeffs = None

        # aruco dictionary name
        self.dict_name = 'DICT_ARUCO_ORIGINAL' #!'DICT_4X4_100'

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

        # frame id of the camera
        self.camera_frame_id = None


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
        # print("image shape: ", cv_image)

        self.camera_frame_id = msg.header.frame_id#"realsense_color_optical_frame"#msg.header.frame_id
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

        print("dictionary: ", dictionary)   
        
        return cv2.aruco.getPredefinedDictionary(dictionary)

    def pose_to_transform(self,pose):
        transform = TransformStamped()

        transform.header = pose.header
        transform.child_frame_id = "aruco"  # replace with appropriate child frame id

        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z

        transform.transform.rotation.x = pose.pose.orientation.x
        transform.transform.rotation.y = pose.pose.orientation.y
        transform.transform.rotation.z = pose.pose.orientation.z
        transform.transform.rotation.w = pose.pose.orientation.w

        return transform


    def estimateArucoPose(self, cv_image):
        """
        Estimate the pose of the aruco marker in the image
        """
        print("Estimating aruco pose...", self.dictionary)

        # detect the aruco markers
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.dictionary)

        # if there are markers detected
        if ids is not None:
            #draw the markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # estimate the pose of the markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            # print("rvecs: ", rvecs.shape)

            # cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
            self.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.1)

            # Convert tvecs to translation vector
            tc = tvecs[0].reshape(-1, 1)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()#! check
            pose.header.frame_id = self.camera_frame_id#! check
            pose.pose.position.x = tc[0]
            pose.pose.position.y = tc[1] 
            pose.pose.position.z = tc[2]

            # # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvecs[0])

            # # Convert rotation matrix to quaternion
            rotation = R.from_matrix(rotation_matrix)
            quaternion = rotation.as_quat()
            qx, qy, qz, qw = quaternion
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            print("aruco pose in camera: ", pose)

            aruco_transform = self.pose_to_transform(pose)

            offset = PoseStamped()
            offset.header.stamp = rospy.Time.now()#! check
            offset.header.frame_id = "aruco"#! check
            offset.pose.position.x = 0
            offset.pose.position.y = 0.076
            offset.pose.position.z = -0.035
            
            # picking pose of aruco marker in camera frame
            offset_in_camera = do_transform_pose(offset, aruco_transform)
            # print(pose)
            # print(offset_in_camera)

            offset_in_camera.pose.orientation.x = qx
            offset_in_camera.pose.orientation.y = qy
            offset_in_camera.pose.orientation.z = qz
            offset_in_camera.pose.orientation.w = qw
            
            # print("aruco pose in camera: ", tc)
            transform = self.tf_butter.lookup_transform('world_ned', self.camera_frame_id, rospy.Time(0), rospy.Duration(1.0))
            aruco_pose_world = do_transform_pose(offset_in_camera, transform)
            # aruco_pose_world = self.tf_butter.transform(offset_in_camera, "world_ned")#! check if these two are the same


            # Publish the pose of aruco marker
            self.pose_pub.publish(aruco_pose_world)

            #aruco pose in the world frame
            rospy.loginfo('Aruco pose in the world frame: {}'.format(aruco_pose_world.pose.position))
        else:
            rospy.loginfo('No aruco...')

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
        # Robot_base-to-camera transformationy
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