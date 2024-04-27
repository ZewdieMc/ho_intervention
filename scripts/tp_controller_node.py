#!/usr/bin/python3

import rospy
from swiftpro_manipulator import SwiftProManipulator
import numpy as np
from utils.kinematics_utils import DLS
from std_msgs.msg import Float64MultiArray
from ho_intervention.srv import PoseGoal, PoseGoalResponse
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from turtlebot_manipulator import TurtlebotManipulator
from geometry_msgs.msg import Twist


def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
class TPController:

    def __init__(self):
        
        
        self.robot = TurtlebotManipulator()

        # Tuning parameters
        self.K = np.eye(6) * 1.3
        self.control_rate = rospy.Rate(10)
        self.time_limit = 50

        # Publisher
        self.joint_vel_pub = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)

        # base cmd_vel publisher
        self.base_cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Sevice
        self.sp_mg_srv = rospy.Service('/swiftpro/move_to_goal', PoseGoal, self.handle_swiftpro_move_to_goal)
        

    def simple_control_loop(self, sigma_d): 
        """
        Controloop for simple resovled-rate motion controllers
        """
        J = self.robot.getEEJacobian()
    
        # Update control
        sigma = self.robot.getEEPose()    # Position of the end-effector
        err =   sigma_d - sigma   # Control error (position error)
        err[-1] = wrap_angle(err[-1])
        
        # Control solutionsgle
        W=np.eye(self.robot.dof)
        W[0, 0] = 10
        W[1, 1] = 100
        dq = (DLS(J,0.1, W) @ (self.K @ err)).reshape(-1,1)
        print("-----")
        # print(dq)
        # publish joint velocity
        dq_msg = Float64MultiArray()
        dq_msg.data = list(dq[2:].flatten())
        self.joint_vel_pub.publish(dq_msg)

        cmd = Twist()
        cmd.linear.x = dq[1]
        cmd.angular.z = dq[0]
        self.base_cmd_vel_pub.publish(cmd)


    def handle_swiftpro_move_to_goal(self,req):
        """
        Service callback for moving swiftpro arm to a goal position
        """
        rospy.loginfo("Received request to move to goal position")
        x = req.goal.position.x
        y = req.goal.position.y
        z = req.goal.position.z
        q = [req.goal.orientation.x, req.goal.orientation.y, req.goal.orientation.z, req.goal.orientation.w]

        _,_,yaw = euler_from_quaternion(q)

        res = PoseGoalResponse()

        sigma_d = np.array([x, y, z, 0, 0, yaw]).reshape(-1,1)
        start_time = rospy.Time.now()
        while True:
            self.simple_control_loop(sigma_d)
            
            if (rospy.Time.now() - start_time).to_sec() > self.time_limit:
                res.success = False
                self.stop_arm()
                return res
            self.control_rate.sleep()
        self.stop_arm()
        # self.stop_base()
        res.success = True
        return res
    
    def stop_arm(self):
        dq_msg = Float64MultiArray()
        dq_msg.data = [0.0,0.0,0.0,0.0]
        self.joint_vel_pub.publish(dq_msg)

    def stop_base(self):
        dq_msg = Float64MultiArray()
        dq_msg.data = [0.0,0.0,0.0,0.0]
        self.joint_vel_pub.publish(dq_msg)
    
    def near_goal(self, sigma_d):
        """
        Check if the robot is near the goal position
        """
        sigma = self.robot.getEEPose()
        #print(np.linalg.norm(sigma - sigma_d))
        return np.linalg.norm(sigma - sigma_d) < 0.004

if __name__ == '__main__':

    rospy.init_node('tp_controller')
    robot = TPController()
    rospy.loginfo("TP Controller node started")

    rospy.spin()