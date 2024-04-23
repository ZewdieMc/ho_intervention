import numpy as np
import rospy
from sensor_msgs.msg import JointState
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
class SwiftProManipulator:
    """
    Class to contain the kinematics of Swift Pro Manipulator
    """
    def __init__(self):
       
        # Class variables
        self.joint_state_name = "turtlebot/swiftpro/joint1"
        self.publish_tf = True
         # TF
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self.publish_static_tf()

        # Robot dimension
        self.lee = 0.0565
        self.lzee = 0.0722
        self.l1 = 0.142
        self.l2 = 0.1588
        self.lb = 0.0132
        self.lzb = 0.0747 + 0.0333

        # Robot kinematics
        self.dof = 4
        self.q = np.zeros(self.dof).reshape(-1,1)
        self.update_kinematics()


       

        # Subscriber
        self.joint_state_sub = rospy.Subscriber("/turtlebot/joint_states", JointState, self.joint_state_callback)

    ################################
    ### Callbacks Functions
    ################################
    def joint_state_callback(self, msg):
        if msg.name[0] == self.joint_state_name:
            self.q[0] = msg.position[0]
            self.q[1] = msg.position[1]
            self.q[2] = msg.position[2]
            self.q[3] = msg.position[3]

            self.update_kinematics()

    ################################
    ### Class Functions
    ################################
    def update_kinematics(self):
        """
        Update robot kinematic according to q
        """
        q1,q2,q3,q4 = self.q
        A = (self.lee -self.l1*np.sin(q2) + self.l2*np.cos(q3) + self.lb)
        self.x_ee = float(A * np.cos(q1))
        self.y_ee = float(A * np.sin(q1))
        self.z_ee = float(-self.lzb - self.l1*np.cos(q2) - self.l2*np.sin(q3) + self.lzee)
        self.yaw_ee = float(q1+q4)
        
        self.T =  np.array([[np.cos(self.yaw_ee), -np.sin(self.yaw_ee), 0, self.x_ee],
                            [np.sin(self.yaw_ee), np.cos(self.yaw_ee), 0, self.y_ee],
                            [0,              0,             1, self.z_ee],
                            [0,              0,             0,     1]])
        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "swiftpro_base"
            transform.child_frame_id = "swiftpro_ee"

            # Set translation
            transform.transform.translation.x = self.x_ee  
            transform.transform.translation.y = self.y_ee  
            transform.transform.translation.z = self.z_ee  

            # Set rotation (quaternion)
            quaternion = quaternion_from_euler(0, 0, self.yaw_ee)
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            self.tf_broadcaster.sendTransform(transform)


    def publish_static_tf(self):
        """
        Publish static TF for attaching arm_base to /turtlebot/swiftpro/manipulator_base
        """
        static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        static_transformStamped = TransformStamped()
        
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "/turtlebot/swiftpro/manipulator_base_link"
        static_transformStamped.child_frame_id = "swiftpro_base"

        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0

        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0

        static_tf_broadcaster.sendTransform(static_transformStamped)

###################################
##### Getter Functions
###################################

    def getEEJacobian(self):
        J = np.zeros((6,self.dof))
        
        q1,q2,q3,q4 = self.q

        l1p = -self.l1*np.sin(q2)
        l2p = self.l2*np.cos(q3)
        A = self.lee + l1p + l2p + self.lb

        J[:,0] = np.array([A * np.sin(q1),                   A * np.cos(q1),               0,                  0, 0, 1 ])# derivertive by q1
        J[:,1] = np.array([-self.l1*np.cos(q2)*np.cos(q1), -self.l1*np.cos(q2)*np.sin(q1), self.l1*np.sin(q2), 0, 0, 0 ]) # derivertive by q2
        J[:,2] = np.array([-self.l2*np.sin(q3)*np.cos(q1), -self.l2*np.sin(q3)*np.sin(q1), -self.l2*np.cos(q3), 0, 0, 0 ]) # derivertive by q3
        J[:,3] = np.array([0, 0, 0, 0, 0, 1 ]) # derivertive by q4
        return J
    
    def getEEPose(self):
        """
        
        """
        ee_position = np.array([self.x_ee, self.y_ee, self.z_ee,0, 0, self.yaw_ee]).reshape(-1,1)
        return ee_position