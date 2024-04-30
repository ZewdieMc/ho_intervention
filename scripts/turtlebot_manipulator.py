import numpy as np
import rospy
from swiftpro_manipulator import *
from swiftpro_manipulator import SwiftProManipulator
from nav_msgs.msg import Odometry
import tf
from utils.kinematics_utils import *
from tf.transformations import euler_from_matrix

class TurtlebotManipulator:
    
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
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw])

        # Mobile base position with respect to the world  
        self.T_WB = compute_transformation(self.eta)
        self.T = kinematics(self.d, self.theta, self.a, self.alpha, self.T_WB)
        
        ...

    def js_callback(self, msg):
        self.arm.joint_state_callback(msg)
        ...

    def update_kinematics(self):
        ...

    def getBaseJacobian(self):
        # Jbase = jacobian(self.T, self.revolute)

        # Compute the partial derivatives of x, y, z with respect to d, theta_robot
        dx_dd = math.cos(self.eta[3])
        dy_dd = math.sin(self.eta[3])
        dz_dd = 0
        
        d = self.eta[0]
        dx_dtheta_r = ((0.0132 - 0.142 * np.sin(self.arm.q[1]) + 0.1588 * np.cos(self.arm.q[2]) + 0.0565) * np.cos(self.arm.q[0])) * math.cos(self.eta[3]) \
                    - 0.051 * math.sin(self.eta[3]) - d * math.sin(self.eta[3]) + ((0.0132 - 0.142 * np.sin(self.arm.q[1]) + 0.1588 * np.cos(self.arm.q[2]) + 0.0565) * np.sin(self.arm.q[0])) \
                    * (-math.sin(self.eta[3]) - math.cos(self.eta[3]))
        dy_dtheta_r = 0.051 * math.cos(self.eta[3]) + d * math.cos(self.eta[3]) + ((0.0132 - 0.142 * np.sin(self.arm.q[1]) + 0.1588 * np.cos(self.arm.q[2]) + 0.0565) * np.sin(self.arm.q[0])) \
                    * (math.cos(self.eta[3]) - math.sin(self.eta[3])) \
                    + ((0.0132 - 0.142 * np.sin(self.arm.q[1]) + 0.1588 * np.cos(self.arm.q[2]) + 0.0565) * np.cos(self.arm.q[0])) \
                    * (math.sin(self.eta[3]))
        dz_dtheta_r = 0

        Jbase = np.array([
            [float(dx_dtheta_r), float(dx_dd)],
            [float(dy_dtheta_r), float(dy_dd)],
            [float(dz_dtheta_r), float(dz_dd)],
            [0, 0],
            [0, 0],
            [1, 0]
        ])
        return Jbase

    '''
    Get the end-effector Jacobian.
    '''
    def getEEJacobian(self):
        Jarm = self.getArmJacobian()
        # T = self.T
        # T.append(self.T[-1] @ self.arm.T)
        # Jbase = jacobian(T, self.revolute)
        Jbase = self.getBaseJacobian()
        J = np.block([Jbase, Jarm])#! check
        return J
    
    '''
    Get the end-effector pose.
    '''
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

    def getDOF(self):
        return self.dof
 
        
    def getArmJacobian(self):
        J = np.zeros((6,self.arm.dof))
        
        q1,q2,q3,q4 = self.arm.q

        l1p = -self.arm.l1*math.sin(q2)
        l2p = self.arm.l2*math.cos(q3)
        A = self.arm.lee + l1p + l2p + self.arm.lb
        

        dx_dq1 = -A * math.sin(q1)*math.sin(self.eta[3]) + A * math.cos(q1)*math.cos(self.eta[3])
        dy_dq1 = A * math.sin(q1)*math.cos(self.eta[3]) + A * math.cos(q1)*math.sin(self.eta[3])

        dx_dq2 = -self.arm.l1*math.cos(q2)*math.cos(q1)*math.sin(self.eta[3]) + (-self.arm.l1*math.cos(q2)*math.sin(q1))*math.cos(self.eta[3])
        dy_dq2 = -self.arm.l1*math.cos(q2)*math.cos(q1)*-math.cos(self.eta[3]) + (-self.arm.l1*math.cos(q2)*math.sin(q1))*math.sin(self.eta[3])
        dz_dq2 = self.arm.l1*math.sin(q2)

        dx_dq3 = -self.arm.l2*math.sin(q3)*math.cos(q1)*math.sin(self.eta[3]) + (-self.arm.l2*math.sin(q3)*math.sin(q1))*math.cos(self.eta[3])
        dy_dq3 = -self.arm.l2*math.sin(q3)*math.cos(q1)*-math.cos(self.eta[3]) + (-self.arm.l2*math.sin(q3)*math.sin(q1))*math.sin(self.eta[3])
        dz_dq3 = -self.arm.l2*math.cos(q3)

        J[:,0] = np.array([dx_dq1, dy_dq1,      0, 0, 0, 1 ])# derivertive by q1
        J[:,1] = np.array([dx_dq2, dy_dq2, dz_dq2, 0, 0, 0 ]) # derivertive by q2
        J[:,2] = np.array([dx_dq3, dy_dq3, dz_dq3, 0, 0, 0 ]) # derivertive by q3
        J[:,3] = np.array([0, 0, 0, 0, 0, 1 ]) # derivertive by q4
        return J