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
from sensor_msgs.msg import JointState
from turtlebot_manipulator import TurtlebotManipulator
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from ho_intervention.msg import DesiredTask, MoveJointAction, MoveToGoalAction
import actionlib
import tf


def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
class TaskPublisher:

    def __init__(self):
        self.current_task = None 
        self.current_desired = None 
        self.current_joint = None 
        self.active = False 
        self.control_interval  = 0.1
        self.ee_pose = [0,0,0,0,0,0]
        self.eta = [0,0,0,0]
        #! Test Position task
        self.tasks_list = [["BaseOnlyPositionTask"]]
        self.desireds_list = [
                                [1.7, -0.12, -0.3, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                                # [1.7, -0.12, 0.0, 0.0, 2.0, 0.0, -0.12, -0.3, 0.0, 0.0],
                            ]
        self.joint_list = [[0, 0]]
    
        #! Test JointPosition task and Configuration task
        # self.tasks_list = [["JointPositionTask","ConfigurationTask"],["JointPositionTask","ConfigurationTask"],["JointPositionTask","ConfigurationTask"],
        #                    ["JointPositionTask","ConfigurationTask"],["JointPositionTask","ConfigurationTask"],["JointPositionTask","ConfigurationTask"],
        #                    ["JointPositionTask","ConfigurationTask"],["JointPositionTask","ConfigurationTask"],["JointPositionTask","ConfigurationTask"],
        #                    ["JointPositionTask","ConfigurationTask"]]
        # self.desireds_list = [
        #                         [-np.pi/4,1.2, -0.5, -0.3, 0.0,0.0,np.pi/4], 
        #                         [-np.pi/4,-0.8, 0.7, -0.2, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,-0.1, -0.1, -0.4, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,0.05, 0.05, -0.25, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,0.0, 0.0, -0.3, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,0.1, 0.1, -0.2, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,-0.1, -0.1, -0.4, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,0.05, 0.05, -0.25, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,0.0, 0.0, -0.3, 0.0,0.0,np.pi/4],
        #                         [-np.pi/4,-0.25, -0.3, -0.25, 0.0,0.0,np.pi/4]
        #                     ]
        # self.joint_list = [[1,0],[1,0],[1,0],[1,0],[1,0],[1,0],[1,0],[1,0],[1,0],[1,0]]

        #! Test Configuration task
        # self.tasks_list = [["ConfigurationTask"],["ConfigurationTask"],["ConfigurationTask"],
        #                    ["ConfigurationTask"],["ConfigurationTask"],["ConfigurationTask"],
        #                    ["ConfigurationTask"],["ConfigurationTask"],["ConfigurationTask"],
        #                    ["ConfigurationTask"]]
        # self.desireds_list = [
        #                         [1.2, -0.5, -0.3, 0.0,0.0,0.0], 
        #                         [-0.8, 0.7, -0.2, 0.0,0.0,0.0],
        #                         [-0.1, -0.1, -0.4, 0.0,0.0,0.0],
        #                         [0.05, 0.05, -0.25, 0.0,0.0,0.0],
        #                         [0.0, 0.0, -0.3, 0.0,0.0,0.0],
        #                         [0.1, 0.1, -0.2, 0.0,0.0,0.0],
        #                         [-0.1, -0.1, -0.4, 0.0,0.0,0.0],
        #                         [0.05, 0.05, -0.25, 0.0,0.0,0.0],
        #                         [0.0, 0.0, -0.3, 0.0,0.0,0.0],
        #                         [-0.25, -0.3, -0.25, 0.0,0.0,0.0]
        #                     ]
        # self.joint_list = [[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]

        # #! Test Orientation task
        # self.tasks_list = [["OrientationTask","PositionTask"],["OrientationTask","PositionTask"],["OrientationTask","PositionTask"],
        #                    ["OrientationTask","PositionTask"],["OrientationTask","PositionTask"],["OrientationTask","PositionTask"],
        #                    ["OrientationTask","PositionTask"],["OrientationTask","PositionTask"],["OrientationTask","PositionTask"],
        #                    ["OrientationTask","PositionTask"]]
        # self.desireds_list = [
        #                         [0.0,0.0,np.pi/16,1.2, -0.5, -0.3], 
        #                         [0.0,0.0,np.pi/8,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi/4,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi/2,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi/2,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi/2,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi/2,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi/2,1.2, -0.5, -0.3],
        #                         [0.0,0.0,np.pi/2,1.2, -0.5, -0.3]
        #                     ]
        # self.joint_list = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
        
        # #! Test BaseOrientation task
        # self.tasks_list = [["BaseOrientationTask","PositionTask"],["BaseOrientationTask","PositionTask"],["BaseOrientationTask","PositionTask"],
        #                    ["BaseOrientationTask","PositionTask"],["BaseOrientationTask","PositionTask"],["BaseOrientationTask","PositionTask"],
        #                    ["BaseOrientationTask","PositionTask"],["BaseOrientationTask","PositionTask"],["BsaseOrientationTask","PositionTask"],
        #                    ["BaseOrientationTask","PositionTask"]]
        # self.desireds_list = [
        #                         [np.pi/16,1.2, -0.5, -0.3], 
        #                         [np.pi/8,1.2, -0.5, -0.3],
        #                         [np.pi/4,1.2, -0.5, -0.3],
        #                         [np.pi/2,1.2, -0.5, -0.3],
        #                         [np.pi,1.2, -0.5, -0.3],
        #                         [np.pi/2,1.2, -0.5, -0.3],
        #                         [np.pi/2,1.2, -0.5, -0.3],
        #                         [np.pi/2,1.2, -0.5, -0.3],
        #                         [np.pi/2,1.2, -0.5, -0.3],
        #                         [np.pi/2,1.2, -0.5, -0.3]
        #                     ]
        # self.joint_list = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]

        self.control_interval  = 0.1

        self.current_task = self.tasks_list.pop(0)
        
        self.current_desired = self.desireds_list.pop(0)
        self.current_joint = self.joint_list.pop(0)
        # Subscribers
        rospy.Subscriber('/tp_controller/ee_pose', Odometry, self.ee_pose_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/turtlebot/joint_states', JointState, self.js_cb)

        # Publishers
        self.task_pub = rospy.Publisher("/desired_tasks", DesiredTask, queue_size= 10)

        # action
        self.move_turtlebot_server = actionlib.SimpleActionServer('move_turtlebot', MoveToGoalAction, self.move_turtlebot, False)
        self.move_turtlebot_server.start()

        self.move_swiftpro_server = actionlib.SimpleActionServer('move_swiftpro', MoveToGoalAction, self.move_swiftpro, False)
        self.move_swiftpro_server.start()
        # self.move_kobuki_server = actionlib.SimpleActionServer('move_kobuki', FollowPathAction, self.move_kobuki, False)
        # self.move_kobuki_server.start()

        self.move_base_server = actionlib.SimpleActionServer('move_base', MoveToGoalAction, self.move_base, False)
        self.move_base_server.start()

        self.move_joint_server = actionlib.SimpleActionServer('move_kobuki', MoveJointAction, self.move_joint, False)
        self.move_joint_server.start()



        # Timer
        rospy.Timer(rospy.Duration(self.control_interval), self.task_publish_loop)

        # For testing 
        # rospy.Timer(rospy.Duration(10), self.test_loop)


    def js_cb(self, msg):
        if msg.name[0] == 'turtlebot/swiftpro/joint1':
            self.q = msg.position

    def odom_cb(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw])

    def ee_pose_cb(self, msg):
        x,y,z,roll,pitch,yaw = self.extract_pose(msg.pose.pose)
        self.ee_pose = [x,y,z,roll,pitch,yaw]

    def task_publish_loop(self, _): 
        if self.active and self.current_task:
            task_msg = DesiredTask()
            task_msg.tasks = self.current_task
            task_msg.desireds = self.current_desired
            task_msg.joint = self.current_joint
            self.task_pub.publish(task_msg)
        else:
            task_msg = DesiredTask()
            task_msg.tasks = []
            task_msg.desireds = []
            task_msg.joint = []
            self.task_pub.publish(task_msg)
            
    def test_loop(self, _): 
        if self.tasks_list:
            self.current_task = self.tasks_list.pop(0)
            self.current_desired = self.desireds_list.pop(0)
            self.current_joint = self.joint_list.pop(0)
            print(self.current_task)
            print(self.current_desired)


    ##########################################
    ### Behaviors functions / Action callbacks
    ##########################################
    def move_turtlebot(self,goal):
        """
        Service to move turtlebot to the goal with configuation tasks
        """
        start_time = rospy.Time.now()
        # Change goal to current task
        x,y,z,roll,pitch,yaw = self.extract_pose(goal.goal.pose)
        self.current_task = ["ConfigurationTask"]
        self.current_desired = [x, y, z, roll, pitch, yaw]
        print(self.current_desired)
        self.current_joint = [0]

        # Activate publisher
        self.active = True
        success = True

        # Keep checking the progress
        while not np.linalg.norm(np.array(self.ee_pose) - np.array(self.current_desired)) < 0.02 and not rospy.is_shutdown():
            print("configuration goal dist: ", np.linalg.norm(np.array(self.ee_pose) - np.array(self.current_desired)))

            if self.move_turtlebot_server.is_preempt_requested() :
                rospy.logerr('Preemptted!!')
                self.move_turtlebot_server.set_preempted()
                self.active = False
                success = False
                break

            if rospy.Time.now() - start_time > rospy.Duration(20):
                rospy.logerr('Time Exceed!!')
                self.move_turtlebot_server.set_preempted()
                self.active = False
                success = False
                break
            
            rospy.sleep(0.1)
        rospy.logerr('Move turtlebot -- Success')
        if success == True:
            self.active = False
            self.move_turtlebot_server.set_succeeded()

    def move_swiftpro(self,goal):
        """
        Service to move turtlebot to the goal with configuation tasks
        """
        start_time = rospy.Time.now()
        # Change goal to current task
        x,y,z,roll,pitch,yaw = self.extract_pose(goal.goal.pose)
        self.current_task = ["ArmOnlyTaskPositionTask"]
        self.current_desired = [x, y, z, roll, pitch, yaw]
        print(self.current_desired)
        self.current_joint = [0]

        # Activate publisher
        self.active = True
        success = True
        rospy.loginfo('Recive Action Move swiftpro')
        # Keep checking the progress
        print(np.linalg.norm(np.array(self.ee_pose) - np.array(self.current_desired)))
        while not np.linalg.norm(np.array(self.ee_pose) - np.array(self.current_desired)) < 0.02  and not rospy.is_shutdown():
            print("Arm task dist: ", np.linalg.norm(np.array(self.ee_pose) - np.array(self.current_desired)))
            if self.move_swiftpro_server.is_preempt_requested() :
                rospy.logerr('Preemptted!!')
                self.move_swiftpro_server.set_preempted()
                self.active = False
                success = False
                break

            if rospy.Time.now() - start_time > rospy.Duration(20):
                rospy.logerr('Time Exceed!!')
                self.move_swiftpro_server.set_preempted()
                self.active = False
                success = False
                break
            
            rospy.sleep(0.1)
        rospy.logerr('Move swiftpro -- Success')
        if success == True:
            self.active = False
            self.move_swiftpro_server.set_succeeded()

    def move_joint(self,goal):
        """
        Service to move turtlebot to the goal with configuation tasks
        """
        start_time = rospy.Time.now()
        # Change goal to current task
        self.current_task = ["JointPositionTask"]
        self.current_desired = goal.position
        self.current_joint = goal.joint

        # Activate publisher
        self.active = True
        success = True

        # Keep checking the progress
        while not np.linalg.norm(np.array(self.q[0]) - np.array(self.current_desired)) < 0.045 and not rospy.is_shutdown():
            print("joint task err: ", np.linalg.norm(np.array(self.q[0]) - np.array(self.current_desired)))
            if self.move_joint_server.is_preempt_requested() :
                rospy.logerr('Preemptted!!')
                self.move_joint_server.set_preempted()
                self.active = False
                success = False
                break

            if start_time - rospy.Time.now() > rospy.Duration(10):
                rospy.logerr('Time Exceed!!')
                self.move_joint_server.set_preempted()
                self.active = False
                success = False
                break
            
            rospy.sleep(0.1)
            
        if success == True:
            self.active = False
            self.move_joint_server.set_succeeded()

    def extract_pose(self,pose):
        x= pose.position.x
        y= pose.position.y
        z= pose.position.z
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        roll,pitch,yaw = euler_from_quaternion(q)

        return x, y, z, roll, pitch, yaw
    
    def move_base(self,goal):
        """
        Service to move turtlebot to the goal with configuation tasks
        """
        start_time = rospy.Time.now()
        # Change goal to current task
        x,y,z,roll,pitch,yaw = self.extract_pose(goal.goal.pose)
        self.current_task = ["BaseOnlyPositionTask"]
        self.current_desired = [x, y, z, yaw]
        print(self.current_desired)
        self.current_joint = [0]

        # Activate publisher
        self.active = True
        success = True

        # Keep checking the progress
        while not np.linalg.norm(np.array(self.eta) - np.array(self.current_desired)) < 0.07 and not rospy.is_shutdown():
            print("move base dist: ", np.linalg.norm(np.array(self.eta) - np.array(self.current_desired)))
            print("current: ", self.eta)
            print("desired: ", self.current_desired)

            if self.move_base_server.is_preempt_requested() :
                rospy.logerr('Preemptted!!')
                self.move_base_server.set_preempted()
                self.active = False
                success = False
                break

            if rospy.Time.now() - start_time > rospy.Duration(20):
                rospy.logerr('Time Exceed!!')
                self.move_base_server.set_preempted()
                self.active = False
                success = False
                break
            
            rospy.sleep(0.1)
        rospy.logerr('Move base -- Success')
        if success == True:
            self.active = False
            self.move_base_server.set_succeeded()
    

if __name__ == '__main__':

    rospy.init_node('task_publisher_node')
    TPub = TaskPublisher()
    

    rospy.loginfo("Task Publisher node started")

    rospy.spin()