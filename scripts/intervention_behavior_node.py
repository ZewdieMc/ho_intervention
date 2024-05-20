#!/usr/bin/env python3

import py_trees
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from ho_intervention.msg import MoveJointAction, MoveToGoalAction, MoveToGoalActionGoal,MoveJointActionGoal, MoveJointGoal
import time
from tf.transformations import quaternion_from_euler
from std_srvs.srv import SetBool
import numpy as np


class setGoal (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        #blackboard variable for move_base behavior
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        # subscribe to the aruco pose
        self.aruco_pose_sub = rospy.Subscriber('/aruco_pose', PoseStamped, self.aruco_pose_callback, queue_size=1)

        #blackboard variable for move_swiftpro behavior
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

        
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("base_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("base_goal", access=py_trees.common.Access.READ)

        self.base_goal = None

    def aruco_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        # base_offset goal
        self.base_goal = PoseStamped()
        self.base_goal.pose.position.x = x 
        self.base_goal.pose.position.y = y                    
        self.base_goal.pose.position.z = 0.0
        # self.aruco_pose_sub.unregister() #! only need to get the pose once...FOR NOW
        q = quaternion_from_euler(0,0,0)
        self.base_goal.pose.orientation.x = q[0]
        self.base_goal.pose.orientation.y = q[1]
        self.base_goal.pose.orientation.z = q[2]
        self.base_goal.pose.orientation.w = q[3]
        if self.base_goal is not None:
            self.blackboard.base_goal = self.base_goal

    def setup(self):
        self.logger.debug("  %s [setGoal::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [setGoal::initialise()]" % self.name)

        

    def update(self):
        # offset_goal = PoseStamped()
        # offset_goal.pose.position.x = 0.8
        # offset_goal.pose.position.y = 0.0
        # offset_goal.pose.position.z = -0.3

        # q = quaternion_from_euler(0,0,0)
        # offset_goal.pose.orientation.x = q[0]
        # offset_goal.pose.orientation.y = q[1]
        # offset_goal.pose.orientation.z = q[2]
        # offset_goal.pose.orientation.w = q[3]
        # self.blackboard.offset_goal = offset_goal


        # goal for the swiftpro to grap the box #! This is now from acuro pose feedback
        # picking_goal = PoseStamped()
        # picking_goal.pose.position.x = 2.0
        # picking_goal.pose.position.y = 0.0
        # picking_goal.pose.position.z = -0.135

        # q = quaternion_from_euler(0, 0, 0)
        # picking_goal.pose.orientation.x = q[0]
        # picking_goal.pose.orientation.y = q[1]
        # picking_goal.pose.orientation.z = q[2]
        # picking_goal.pose.orientation.w = q[3]
        # self.blackboard.picking_goal = picking_goal
        

        # goal for the swiftpro to move the box to platform #! Danger.....dont use this task for platform...the robot will crash
        # platform_goal = PoseStamped()
        # platform_goal.pose.position.x = 1.60775
        # platform_goal.pose.position.y = 0.00
        # platform_goal.pose.position.z = -0.45

        # q = quaternion_from_euler(0, 0, 0)
        # platform_goal.pose.orientation.x = q[0]
        # platform_goal.pose.orientation.y = q[1]
        # platform_goal.pose.orientation.z = q[2]
        # platform_goal.pose.orientation.w = q[3]
        # self.blackboard.platform_goal = platform_goal

        # base_goal = PoseStamped()
        # base_goal.pose.position.x = -1.2
        # base_goal.pose.position.y = 0.00
        # base_goal.pose.position.z = 0.0

        # q = quaternion_from_euler(0, 0, 0)
        # base_goal.pose.orientation.x = q[0]
        # base_goal.pose.orientation.y = q[1]
        # base_goal.pose.orientation.z = q[2]
        # base_goal.pose.orientation.w = q[3]
        self.blackboard.base_goal = self.base_goal
        return py_trees.common.Status.SUCCESS


class moveTurtlebot (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(moveTurtlebot, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [moveTurtlebot::setup()]" % self.name)
        self.goal_sent = False
        self.client = actionlib.SimpleActionClient('move_turtlebot', MoveToGoalAction)
        rospy.loginfo("Waiting for move turtlebot action...")
        self.client.wait_for_server()


    def initialise(self):
        self.logger.debug("  %s [moveTurtlebot::initialise()]" % self.name)

    def update(self):
        print(self.client.get_state())
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            rospy.loginfo('Send goal to turtlebot')
            goal = MoveToGoalActionGoal()
            goal.goal = self.blackboard.offset_goal
            self.client.send_goal(goal)
            self.goal_sent = True
        if self.client.get_state() == actionlib.GoalStatus.PREEMPTED :
            rospy.logerr('Move turtlebot Fail')
            self.goal_sent = False
            return py_trees.common.Status.FAILURE
        
        elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED :
            rospy.loginfo('Move turtlebot Success')
            self.goal_sent = False
            return py_trees.common.Status.SUCCESS
    
        else:
            rospy.loginfo('Move turtlebot working...')
            return py_trees.common.Status.RUNNING
        
class moveTurtlebotAruco (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(moveTurtlebotAruco, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)
        self.picking_goal = None

        self.aruco_pose_sub = rospy.Subscriber('/aruco_pose', PoseStamped, self.aruco_pose_callback, queue_size=1)

    def setup(self):
        self.logger.debug("  %s [moveTurtlebot_aruco::setup()]" % self.name)
        self.goal_sent = False
        self.client = actionlib.SimpleActionClient('move_aruco', MoveToGoalAction)
        rospy.loginfo("Waiting for move turtlebot action...")
        self.client.wait_for_server()

    def aruco_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        rospy.logwarn(f"aruco pose: {x}, {y}, {z}")
        
        # picking goal
        self.picking_goal = PoseStamped()
        self.picking_goal.pose.position.x = x 
        self.picking_goal.pose.position.y = y                    
        self.picking_goal.pose.position.z = z
        
        q = quaternion_from_euler(0,0,0)
        self.picking_goal.pose.orientation.x = q[0]
        self.picking_goal.pose.orientation.y = q[1]
        self.picking_goal.pose.orientation.z = q[2]
        self.picking_goal.pose.orientation.w = q[3]
        self.blackboard.picking_goal = self.picking_goal
        # self.aruco_pose_sub.unregister() #! only need to get the pose once...FOR NOW

    def initialise(self):
        self.logger.debug("  %s [moveTurtlebot::initialise()]" % self.name)

    def update(self):
        # print(self.client.get_state())
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST and self.picking_goal is not None:
            self.blackboard.picking_goal = self.picking_goal
            print(self.blackboard.picking_goal)
            rospy.loginfo('Send picking goal')
            goal = MoveToGoalActionGoal()
            goal.goal = self.blackboard.picking_goal
            self.client.send_goal(goal)
            self.goal_sent = True
        if self.client.get_state() == actionlib.GoalStatus.PREEMPTED :
            rospy.logerr('picking failed')
            self.goal_sent = False
            return py_trees.common.Status.FAILURE
        
        elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED :
            rospy.loginfo('Picking box Success')
            self.goal_sent = False
            return py_trees.common.Status.SUCCESS
    
        else:
            rospy.loginfo('Move to pick working...')
            return py_trees.common.Status.RUNNING
        

class moveBase (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(moveBase, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("base_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("base_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [moveBase::setup()]" % self.name)
        self.goal_sent = False
        self.client = actionlib.SimpleActionClient('move_base', MoveToGoalAction)
        rospy.loginfo("Waiting for move base action...")
        self.client.wait_for_server()


    def initialise(self):
        self.logger.debug("  %s [moveBase::initialise()]" % self.name)

    def update(self):
        print(self.client.get_state())
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            rospy.loginfo('Send goal to base')
            goal = MoveToGoalActionGoal()
            # print(self.blackboard.base_goal)
            goal.goal = self.blackboard.base_goal
            self.client.send_goal(goal)
            self.goal_sent = True
        if self.client.get_state() == actionlib.GoalStatus.PREEMPTED :
            rospy.logerr('Move base Fail')
            self.goal_sent = False
            return py_trees.common.Status.FAILURE
        
        elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED :
            rospy.loginfo('Move base Success')
            self.goal_sent = False
            return py_trees.common.Status.SUCCESS
    
        else:
            rospy.loginfo('Move base working...')
            return py_trees.common.Status.RUNNING
class moveSwiftpro (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(moveSwiftpro, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.name = name
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [moveSwiftpro::setup()]" % self.name)
        self.goal_sent = False
        self.client = actionlib.SimpleActionClient('move_swiftpro', MoveToGoalAction)
        rospy.loginfo("Waiting for move turtlebot action...")
        self.client.wait_for_server()


    def initialise(self):
        self.logger.debug("  %s [moveSwiftpro::initialise()]" % self.name)

    def update(self):
        print(self.client.get_state())
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            rospy.loginfo('Send goal to swiftpro')
            goal = MoveToGoalActionGoal()
            goal.goal = self.blackboard.picking_goal if self.name == "move_down" else self.blackboard.platform_goal
            self.client.send_goal(goal)
            self.goal_sent = True
        if self.client.get_state() == actionlib.GoalStatus.PREEMPTED :
            rospy.logerr('Move swiftpro Fail')
            self.goal_sent = False
            return py_trees.common.Status.FAILURE
        
        elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED :
            rospy.loginfo('Move swiftpro Success')
            self.goal_sent = False
            return py_trees.common.Status.SUCCESS
    
        else:
            rospy.loginfo('Move swiftpro working...')
            return py_trees.common.Status.RUNNING
        
class EnableSuction (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(EnableSuction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [EnableSuction::setup()]" % self.name)
        

    def initialise(self):
        self.logger.debug("  %s [EnableSuction::initialise()]" % self.name)

    def update(self):
        succes = self.enable_suction()
        if succes:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    def enable_suction(self):
        rospy.logwarn("Calling enable suction")
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        path = []
        try:
            enable_suction = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            resp = enable_suction(True)
            
            return resp.success
        
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False
        
class DisableSuction (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(DisableSuction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [DisableSuction::setup()]" % self.name)
        

    def initialise(self):
        self.logger.debug("  %s [DisableSuction::initialise()]" % self.name)

    def update(self):
        succes = self.disable_suction()
        if succes:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    def disable_suction(self):
        rospy.logwarn("Calling disable suction")
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        path = []
        try:
            enable_suction = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            resp = enable_suction(False)
            
            return resp.success
        
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False


class PlaceBox (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PlaceBox, self).__init__(name)
        self.name = name
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [PlaceBox::setup()]" % self.name)
        self.client = actionlib.SimpleActionClient('move_joint', MoveJointAction)
        rospy.loginfo("Waiting for placebox action...")
        self.client.wait_for_server()

    def initialise(self):
        self.logger.debug("  %s [PlaceBox::initialise()]" % self.name)

    def update(self):
    #    print(self.client.get_state())
       if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            goal = MoveJointGoal()
            if self.name == "align_EE":
                rospy.loginfo('align EE orientation')
                goal.joint = [0, 1, 2 ]
                goal.position =  [np.pi/2, 0.05,-0.1]#! realrobot [-np.pi/2, -0.4,-0.1] #! in real robot, it's -np.pi/2
            else:
                rospy.loginfo('Send goal to place box')
                goal.joint = [0]
                goal.position =  [-1.45]
            self.client.send_goal(goal)
            self.goal_sent = True

       if self.client.get_state() == actionlib.GoalStatus.PREEMPTED:
            rospy.logerr('Place box Fail') if self.name != "align_EE" else rospy.logerr('Align EE Fail')
            self.goal_sent = False
            return py_trees.common.Status.FAILURE
       elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo('Place box Success') if self.name != "align_EE" else rospy.loginfo('Align EE Success')
        self.goal_sent = False
        return py_trees.common.Status.SUCCESS
       else:
        rospy.loginfo('Place box working...') 
        return py_trees.common.Status.RUNNING
 


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("intervention_behavior_trees")

    # Create Behaviors
    align_EE = PlaceBox("align_EE")                     #joint1 position
    set_goal = setGoal("set_goal")                      #set bb variables to different tasks
    move_base = moveBase("move_base")                   #move closer to the aruco box
    # move_offset = moveTurtlebot("move_offset")
    # move_down = moveSwiftpro("move_down")
    move_down = moveTurtlebotAruco("move_down")         #move to picking position...config task
    enable_suction = EnableSuction("grab_box")          #enable suction
    move_up = moveTurtlebot("move_up")                  #move up after picking   ...config task
    move_place_box = PlaceBox("move_to_place_box")      #move to placing position...joint1 for now
    place_box = DisableSuction("place_box")             #disable suction


    # go to pickup spot sequence
    go_to_seq = py_trees.composites.Sequence(name="pick_seq", memory=True)
    # go_to_seq.add_children([set_goal,align_EE, move_base,move_down, enable_suction,move_up, move_place_box, place_box])
    # go_to_seq.add_children([set_goal,align_EE])
    go_to_seq.add_children([set_goal,align_EE,move_base, move_down])

    print("Call setup for all tree children")
    py_trees.display.render_dot_tree(go_to_seq)
    go_to_seq.setup_with_descendants() 
    print("Setup done!\n\n")
    py_trees.display.ascii_tree(go_to_seq)

    time.sleep(1)    
    while go_to_seq.status != py_trees.common.Status.SUCCESS and not rospy.is_shutdown():
        go_to_seq.tick_once() 
        time.sleep(1)