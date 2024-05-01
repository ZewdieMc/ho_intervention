#!/usr/bin/python3

import rospy
from task import JointLimitTask, PositionTask
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
class TaskPublisher:

    def __init__(self):
        self.current_task = None 
        self.current_desired = None 
        

        
            # "PositionTask": PositionTask("Position", np.array([0.0, 0.0, 0.0]).reshape(-1,1)), 
            # "ConfigurationTask": ConfigurationTask("Configuration", np.array([0.0, 0.0, 0.0, 0.0]).reshape(-1,1)),
            # "JointPositionTask": JointPositionTask("JointPosition", np.array([0.0]).reshape(-1,1)),
            # "BaseOrientationTask": BaseOrientationTask("BaseOrientation", np.array([0.0]).reshape(-1,1)),
            # "OrientationTask": OrientationTask("Orientation", np.array([0.0]).reshape(-1,1)),
        #For testing
        self.tasks_list = [["PositionTask"],["PositionTask"],["PositionTask"],["PositionTask"],["PositionTask"],["PositionTask"],["PositionTask"],["PositionTask"],["PositionTask"],["PositionTask"]]
        self.desireds_list = [
                                [1.2, -0.5, -0.3],
                                [-0.8, 0.7, -0.2],
                                [-0.1, -0.1, -0.4],
                                [0.05, 0.05, -0.25],
                                [0.0, 0.0, -0.3],
                                [0.1, 0.1, -0.2],
                                [-0.1, -0.1, -0.4],
                                [0.05, 0.05, -0.25],
                                [0.0, 0.0, -0.3],
                                [-0.25, -0.3, -0.25]
                            ]

        self.control_interval  = 0.1
        # Publishers
        self.task_pub = rospy.Publisher("/desired_tasks", DesiredTask, queue_size= 10)

        # Timer
        rospy.Timer(rospy.Duration(self.control_interval), self.task_publish_loop)

        # For testing 
        rospy.Timer(rospy.Duration(10), self.test_loop)

    def task_publish_loop(self, _): 
        if self.current_task:
            task_msg = DesiredTask()
            task_msg.tasks = self.current_task
            task_msg.desireds = self.current_desired
            self.task_pub.publish(task_msg)
            
    def test_loop(self, _): 
        if self.tasks_list:
            self.current_task = self.tasks_list.pop(0)
            self.current_desired = self.desireds_list.pop(0)
            print(self.current_task)
            print(self.current_desired)

   

if __name__ == '__main__':

    rospy.init_node('task_publisher_node')
    TPub = TaskPublisher()
    

    rospy.loginfo("Task Publisher node started")

    rospy.spin()