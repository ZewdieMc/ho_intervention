#!/usr/bin/python3

import rospy
from task import JointLimitTask, PositionTask, ConfigurationTask, OrientationTask, JointPositionTask, BaseOrientationTask
from task_priority import TaskPriority
from swiftpro_manipulator import SwiftProManipulator
import numpy as np
from utils.kinematics_utils import DLS
from std_msgs.msg import Float64MultiArray
from ho_intervention.srv import PoseGoal, PoseGoalResponse
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from turtlebot_manipulator import TurtlebotManipulator
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from ho_intervention.msg import DesiredTask



def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
class TPController:

    def __init__(self, TP: TaskPriority):
        
        self.robot = TurtlebotManipulator()

        # Tuning parameters
        self.K = np.eye(6) * 1.3
        self.control_rate = rospy.Rate(10)
        self.time_limit = 50

         # Task Priority

        self.TP = TP


        #Available tasks
        self.available_taks = {
            "PositionTask": PositionTask("Position", np.array([0.0, 0.0, 0.0]).reshape(-1,1)), 
            "ConfigurationTask": ConfigurationTask("Configuration", np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]).reshape(-1,1)),
            "JointPositionTask": JointPositionTask("JointPosition", np.array([0.0]).reshape(-1,1), 0),
            "BaseOrientationTask": BaseOrientationTask("BaseOrientation", np.array([0.0]).reshape(-1,1)),
            "OrientationTask": OrientationTask("Orientation", np.array([0.0, 0.0, 0.0]).reshape(-1,1)),
        }

        # Publisher
        self.joint_vel_pub = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)

        # base cmd_vel publisher
        self.base_cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.ee_pose_pub = rospy.Publisher("~ee_pose", Odometry, queue_size=10)
        # Sevice
        self.sp_mg_srv = rospy.Subscriber('/desired_tasks', DesiredTask , self.task_handler)

        #subscriber
        self.goal_sub = rospy.Subscriber("/goal", PoseStamped, self.stop_arm)
        

    def simple_control_loop(self, sigma_d): 
        """
        Controloop for simple resovled-rate motion controllers
        """
        J = self.robot.getEEJacobian()
    
        # Update control
        sigma = self.robot.getEEPose()    # Position of the end-effector
        err =   sigma_d - sigma   # Control error (position error)
        # err[-1] = wrap_angle(err[-1])
        
        # Control solutionsgle
        W=np.eye(self.robot.dof)
        W[0, 0] = 10#! we are not using this weight..DON'T TUNE HERE
        W[1, 1] = 100#! we are not using this weight..DON'T TUNE HERE
        dq = (DLS(J,0.1, W) @ (self.K @ err)).reshape(-1,1)
        # publish joint velocity
        dq_msg = Float64MultiArray()
        dq_msg.data = list(dq[2:].flatten())
        self.joint_vel_pub.publish(dq_msg)

        cmd = Twist()
        cmd.linear.x = dq[1]
        cmd.angular.z = dq[0]
        self.base_cmd_vel_pub.publish(cmd)

    def task_handler(self, msg):
        self.TP.tasks = self.TP.tasks[:4]
        desired_msg = msg.desireds
        for i, task_name in enumerate(msg.tasks):
            task = self.available_taks[task_name]
            size = task.getDesired().shape[0]
            task.joint = msg.joint[i]
            desired = np.array(desired_msg[0:size]).reshape(-1,1) #! make sure i + size < len(desired_msg)
            desired_msg = desired_msg[size:]
            task.setDesired(desired)
            
            self.TP.tasks.append(task)
        print("Tasks:\n", self.TP.tasks)
        dq = self.TP.recursive_tp(self.robot)
        # publish joint velocity
        dq_msg = Float64MultiArray()
        dq_msg.data = list(dq[2:].flatten())
        self.joint_vel_pub.publish(dq_msg)

        cmd = Twist()
        cmd.linear.x = dq[1]
        cmd.angular.z = dq[0]
        # cmd.linear.x = min(max(dq[1], -0.5), 0.5)
        # cmd.angular.z = min(max(dq[0], -0.3), 0.3)
        self.base_cmd_vel_pub.publish(cmd)

        ee_pose = self.robot.getEEPose()
        self.publishEEpose(ee_pose[:3,0])


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
        TP.tasks[TP.tasks.index([task for task in TP.tasks if task.name == "Position"][0])].setDesired(sigma_d[:3].reshape(-1,1))
        task = self.TP.tasks
        start_time = rospy.Time.now()
        while True:
            #! self.simple_control_loop(sigma_d)
            dq = TP.recursive_tp(self.robot)
            # publish joint velocity
            dq_msg = Float64MultiArray()
            dq_msg.data = list(dq[2:].flatten())
            self.joint_vel_pub.publish(dq_msg)

            cmd = Twist()
            cmd.linear.x = dq[1]
            cmd.angular.z = dq[0]
            # cmd.linear.x = min(max(dq[1], -0.5), 0.5)
            # cmd.angular.z = min(max(dq[0], -0.3), 0.3)
            self.base_cmd_vel_pub.publish(cmd)

            ee_pose = self.robot.getEEPose()
            self.publishEEpose(ee_pose[:3,0])
            if (rospy.Time.now() - start_time).to_sec() > self.time_limit:
                res.success = False
                # self.stop_arm()
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
    
    def publishEEpose(self, ee):
        """
        Publishes the end-effector pose
        """
        ee_pose = Odometry()
        ee_pose.header.stamp = rospy.Time.now()
        ee_pose.header.frame_id = "world_ned"
        ee_pose.pose.pose.position.x = ee[0]
        ee_pose.pose.pose.position.y = ee[1]
        ee_pose.pose.pose.position.z = ee[2]
        self.ee_pose_pub.publish(ee_pose)

if __name__ == '__main__':

    rospy.init_node('tp_controller')
    TP = TaskPriority(
            [
                JointLimitTask("Joint limit", np.array([0.05, 0.09]), np.array([-np.pi/2, np.pi/2]), 0),
                JointLimitTask("Joint limit", np.array([0.05, 0.09]), np.array([-np.pi/2, 0.05]), 1),
                JointLimitTask("Joint limit", np.array([0.05, 0.09]), np.array([-np.pi/2, 0.05]), 2),
                JointLimitTask("Joint limit", np.array([0.05, 0.09]), np.array([-np.pi/2, np.pi/2]), 3),
                PositionTask("Position", np.array([-2.0, 4.0, -0.25]).reshape(-1,1)),
            ]
        )
    robot = TPController(TP)
    

    rospy.loginfo("TP Controller node started")

    rospy.spin()