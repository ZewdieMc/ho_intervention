import numpy as np
import rospy
from swiftpro_manipulator import *
from swiftpro_manipulator import SwiftProManipulator
from nav_msgs.msg import Odometry
import tf
from utils.kinematics_utils import *
from tf.transformations import euler_from_matrix

class TurtlebotManipulator():
    
    def __init__(self):
        # 
        super(TurtlebotManipulator, self).__init__()
        # DH parameters for the base
        self.d = np.array([-0.198, 0.0507])
        self.theta = np.array([-np.pi/2, 0.0])
        self.a = np.array([0.0, 0.0])
        self.alpha = np.array([-np.pi/2, np.pi/2])
        self.revolute = np.array([True, False, True, False])

        self.arm = SwiftProManipulator()
        # Vector of base pose (position & orientation)
        self.eta = np.zeros((3, 1))

        # Base transformation wrt world
        self.T_WB = np.eye(4)

        # Kinemmatic transformation
        self.T = kinematics(self.d, self.theta, self.a, self.alpha, self.T_WB)

        # Number of DOF of the system
        self.dof = 6

       # Subscriber
        self.js_sub = rospy.Subscriber('/turtlebot/joint_states', JointState, self.js_callback)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

        # Mobile base position with respect to the world  
        self.T_WB = compute_transformation(self.eta)
        self.T = kinematics(self.d, self.theta, self.a, self.alpha, self.T_WB)
        
        ...

    def js_callback(self, msg):
        ...

    def update_kinematics(self):
        ...

    def getBaseJacobian(self):
        Jbase = jacobian(self.T, self.revolute)
        return Jbase

    def getEEJacobian(self):
        Jarm = self.arm.getEEJacobian()
        T = self.T
        T.append(self.T[-1] @ self.arm.T)
        Jbase = jacobian(T, self.revolute)
        J = np.concatenate((Jbase[:,:2], Jarm), axis=1)#! check
        return J
    
    def getEEPose(self):
        Tfinal = self.T[-1] @ self.arm.T
        rotation = Tfinal[:3, :3]
        rpy = euler_from_matrix(rotation)
        return np.array([
            Tfinal[0, 3],
            Tfinal[1, 3],
            Tfinal[2, 3],
            rpy[0],
            rpy[1],
            rpy[2]
        ]).reshape(-1, 1)
        
