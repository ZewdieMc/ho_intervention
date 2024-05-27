#!/usr/bin/env python3

import py_trees
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from ho_intervention.msg import MoveJointAction, MoveToGoalAction, MoveToGoalActionGoal,MoveJointActionGoal, MoveJointGoal
import time
from tf.transformations import quaternion_from_euler
from std_srvs.srv import SetBool, Empty, EmptyResponse, Trigger, TriggerResponse
import numpy as np
from copy import copy
from geometry_msgs.msg import Twist


class setGoalPickUp (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        #blackboard variable for move_base behavior
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        # subscribe to the aruco pose
        self.aruco_pose_sub = rospy.Subscriber('/aruco_pose', PoseStamped, self.aruco_pose_callback, queue_size=1)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.stop_vel = Twist()
        self.turning_vel = Twist()
        self.turning_vel.angular.z = 0.6

        #blackboard variable for move_swiftpro behavior
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

        
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("base_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("base_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.READ)

        self.aruco_pose = None

        self.current_time = rospy.Time.now()
        self.stopping = False

    def aruco_pose_callback(self, msg):
        if self.stopping:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            # base_offset goal
            self.aruco_pose = PoseStamped()
            self.aruco_pose.pose.position.x = x 
            self.aruco_pose.pose.position.y = y                    
            self.aruco_pose.pose.position.z = z
            #! only need to get the pose once...FOR NOW
            q = quaternion_from_euler(0,0,0)
            self.aruco_pose.pose.orientation.x = q[0]
            self.aruco_pose.pose.orientation.y = q[1]
            self.aruco_pose.pose.orientation.z = q[2]
            self.aruco_pose.pose.orientation.w = q[3]
        
    def setup(self):
        self.logger.debug("  %s [setGoalPickUp::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [setGoalPickUp::initialise()]" % self.name)

        

    def update(self):

        if self.aruco_pose is None:
            rospy.logerr("no aruco_pose")
            if (rospy.Time.now() - self.current_time).to_sec() > 5 and not self.stopping:
                self.vel_pub.publish(self.stop_vel)
                print("stop")
                self.current_time = rospy.Time.now() 
                self.stopping = True
                
            elif (rospy.Time.now() - self.current_time).to_sec() > 4 and self.stopping:
                print("turn")
                self.vel_pub.publish(self.turning_vel)
                self.stopping = False
                self.current_time = rospy.Time.now()
                
            


            return py_trees.common.Status.RUNNING

       # self.vel_pub.publish(self.stop_vel)
        # goal for the swiftpro to grap the box #! This is now from acuro pose feedback
        picking_goal = copy(self.aruco_pose)
        self.blackboard.picking_goal = picking_goal

        offset_goal = PoseStamped()
        offset_goal.pose.position.x = self.aruco_pose.pose.position.x
        offset_goal.pose.position.y = self.aruco_pose.pose.position.y
        offset_goal.pose.position.z = self.aruco_pose.pose.position.z - 0.1
        self.blackboard.offset_goal = offset_goal

        base_goal = PoseStamped()
        base_goal.pose.position.x = self.aruco_pose.pose.position.x
        base_goal.pose.position.y = self.aruco_pose.pose.position.y
        base_goal.pose.position.z = self.aruco_pose.pose.position.z 
        self.blackboard.base_goal = base_goal

        drop_goal = PoseStamped()
        drop_goal.pose.position.x = self.aruco_pose.pose.position.x + 0.5
        drop_goal.pose.position.y = self.aruco_pose.pose.position.y 
        drop_goal.pose.position.z = self.aruco_pose.pose.position.z 
        self.blackboard.drop_goal = drop_goal
        

        self.vel_pub.publish(self.stop_vel)
        self.aruco_pose_sub.unregister()
        return py_trees.common.Status.SUCCESS

class setGoalPlaceDown (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        #blackboard variable for move_base behavior
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        # subscribe to the aruco pose
        # self.aruco_pose_sub = rospy.Subscriber('/aruco_pose', PoseStamped, self.aruco_pose_callback, queue_size=1)

        #blackboard variable for move_swiftpro behavior
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

        
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("base_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("base_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.READ)

        self.aruco_pose = None

    def aruco_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        # base_offset goal
        self.aruco_pose = PoseStamped()
        self.aruco_pose.pose.position.x = x 
        self.aruco_pose.pose.position.y = y                    
        self.aruco_pose.pose.position.z = z
        self.aruco_pose_sub.unregister() #! only need to get the pose once...FOR NOW
        q = quaternion_from_euler(0,0,0)
        self.aruco_pose.pose.orientation.x = q[0]
        self.aruco_pose.pose.orientation.y = q[1]
        self.aruco_pose.pose.orientation.z = q[2]
        self.aruco_pose.pose.orientation.w = q[3]
       
    def setup(self):
        self.logger.debug("  %s [setGoalPlaceDown::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [setGoalPlaceDown::initialise()]" % self.name)

        

    def update(self):

        # if self.aruco_pose is None:
        #     return py_trees.common.Status.RUNNING

        # goal for the swiftpro to grap the box #! This is now from acuro pose feedback
        # picking_goal = copy(self.aruco_pose)
        # self.blackboard.picking_goal = picking_goal

        # offset_goal = PoseStamped()
        # offset_goal.pose.position.x = self.aruco_pose.pose.position.x
        # offset_goal.pose.position.y = self.aruco_pose.pose.position.y
        # offset_goal.pose.position.z = self.aruco_pose.pose.position.z - 0.1
        # self.blackboard.offset_goal = offset_goal

        # base_goal = PoseStamped()
        # base_goal.pose.position.x = self.aruco_pose.pose.position.x
        # base_goal.pose.position.y = self.aruco_pose.pose.position.y
        # base_goal.pose.position.z = self.aruco_pose.pose.position.z 
        # self.blackboard.base_goal = base_goal

        drop_goal = PoseStamped()
        drop_goal.pose.position.x = 0
        drop_goal.pose.position.y = 0
        drop_goal.pose.position.z = -0.2
        self.blackboard.drop_goal = drop_goal
        

  
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
        self.name = name
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("picking_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.READ)
        self.picking_goal = None


    def setup(self):
        self.logger.debug("  %s [moveTurtlebot_aruco::setup()]" % self.name)
        self.goal_sent = False
        self.client = actionlib.SimpleActionClient('move_turtlebot', MoveToGoalAction)
        rospy.loginfo("Waiting for move turtlebot action...")
        self.client.wait_for_server()


    def initialise(self):
        self.logger.debug("  %s [moveTurtlebot::initialise()]" % self.name)

    def update(self):
        # print(self.client.get_state())
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            # self.blackboard.picking_goal = self.picking_goal
            rospy.loginfo('Send picking goal')
            

            # self.picking_goal = PoseStamped()
            # self.picking_goal.pose.position.x = 1.726 
            # self.picking_goal.pose.position.y = -0.021                   
            # self.picking_goal.pose.position.z = -0.154
            
            # q = quaternion_from_euler(0,0,0)
            # self.picking_goal.pose.orientation.x = q[0]
            # self.picking_goal.pose.orientation.y = q[1]
            # self.picking_goal.pose.orientation.z = q[2]
            # self.picking_goal.pose.orientation.w = q[3]
            # self.blackboard.picking_goal = self.picking_goal
            
            if self.name == "move_pick":
                print("move_pick" )
                goal = MoveToGoalActionGoal()
                goal.goal = self.blackboard.picking_goal
            elif self.name == "move_up":
                print("move_up")
                goal = MoveToGoalActionGoal()
                goal.goal = self.blackboard.offset_goal

            elif self.name == "move_drop":
                print("move_drop")
                goal = MoveToGoalActionGoal()
                goal.goal = self.blackboard.drop_goal
            print(goal.goal)

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

        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("drop_goal", access=py_trees.common.Access.READ)

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
            if self.name == "move_base":
                goal.goal = self.blackboard.base_goal
            else:
                goal.goal = self.blackboard.drop_goal
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
            goal.goal = self.blackboard.picking_goal if self.name == "move_pick" else self.blackboard.platform_goal
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
                # goal.position =  [np.pi/2, 0.05,-0.1]#! realrobot [-np.pi/2, -0.4,-0.1] #! in real robot, it's -np.pi/2
                goal.position =  [-np.pi/2, -0.4,-0.1]
            elif self.name == "place_box" or "contract_arm":
                rospy.loginfo('Send goal to place box')
                goal.joint = [2,0,1]
                goal.position =  [0.0,np.pi/2, 0.0]
            else:
                rospy.loginfo('Send goal to place box')
                goal.joint = [0]
                goal.position =  [-np.pi/2]
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
 
def handle_start_pickup(req):
    # Create Behaviors
    align_EE = PlaceBox("align_EE")                     #joint1 position
    set_goal = setGoalPickUp("set_goal")                      #set bb variables to different tasks
    move_base = moveBase("move_base")                   #move closer to the aruco box
    move_base_drop = moveBase("move_base_drop")                   #move closer to the aruco box

    # move_offset = moveTurtlebot("move_offset")
    # move_pick = moveSwiftpro("move_pick")
    move_pick = moveTurtlebotAruco("move_pick")         #move to picking position...config task
    enable_suction = EnableSuction("grab_box")          #enable suction
    move_up = moveTurtlebotAruco("move_up")                  #move up after picking   ...config task
    place_box = PlaceBox("place_box")      #move to placing position...joint1 for now
    contract_arm = PlaceBox("contract_arm")      #move to placing position...joint1 for now

    disable_suction = DisableSuction("disable_suction")             #disable suction
    move_drop = moveBase("move_drop")
    align_drop = PlaceBox("align_drop")      #move to placing position...joint1 for now

    drop = moveTurtlebotAruco("move_drop")

    # go to pickup spot sequence
    pick_up_seq = py_trees.composites.Sequence(name="pick_seq", memory=True)
    # pick_up_seq.add_children([set_goal,align_EE, move_base,move_pick, enable_suction,move_up, move_place_box, place_box])
    # pick_up_seq.add_children([set_goal,align_EE])
    pick_up_seq.add_children([set_goal,align_EE,move_base, move_pick,enable_suction,move_up,place_box,move_base_drop,align_drop,move_drop,disable_suction,contract_arm])

    print("Call setup for all tree children")
    py_trees.display.render_dot_tree(pick_up_seq)
    pick_up_seq.setup_with_descendants() 
    print("Setup done!\n\n")
    py_trees.display.ascii_tree(pick_up_seq)

    time.sleep(1)    
    while not rospy.is_shutdown():
        print("tick")
        pick_up_seq.tick_once() 
        if pick_up_seq.status == py_trees.common.Status.SUCCESS :
            res = TriggerResponse()
            res.success = True
            return res
        elif pick_up_seq.status == py_trees.common.Status.FAILURE:
            res = TriggerResponse()
            res.success = False
            return res
        time.sleep(1)
    res = TriggerResponse()
    res.success = False
    return res

def handle_start_placedown(req):
    # Create Behaviors
    align_EE = PlaceBox("align_EE")                     #joint1 position
    set_goal = setGoalPlaceDown("set_goal")                      #set bb variables to different tasks
    move_base = moveBase("move_base")                   #move closer to the aruco box
    # move_offset = moveTurtlebot("move_offset")
    # move_pick = moveSwiftpro("move_pick")
    move_pick = moveTurtlebotAruco("move_pick")         #move to picking position...config task
    enable_suction = EnableSuction("grab_box")          #enable suction
    move_up = moveTurtlebotAruco("move_up")                  #move up after picking   ...config task
    move_place_box = PlaceBox("place_box")      #move to placing position...joint1 for now
    disable_suction = DisableSuction("place_box")             #disable suction
    move_drop = moveBase("move_drop")
    move_drop_box = PlaceBox("dropping")      #move to placing position...joint1 for now

    drop = moveTurtlebotAruco("move_drop")

    # go to pickup spot sequence
    place_down_seq = py_trees.composites.Sequence(name="pick_seq", memory=True)
    # place_down_seq.add_children([set_goal,align_EE, move_base,move_pick, enable_suction,move_up, move_place_box, place_box])
    # place_down_seq.add_children([set_goal,align_EE])
    place_down_seq.add_children([move_drop_box,drop,disable_suction,move_place_box])

    print("Call setup for all tree children")
    py_trees.display.render_dot_tree(place_down_seq)
    place_down_seq.setup_with_descendants() 
    print("Setup done!\n\n")
    py_trees.display.ascii_tree(place_down_seq)

    time.sleep(1)    
    while not rospy.is_shutdown():
        place_down_seq.tick_once() 
        if place_down_seq.status == py_trees.common.Status.SUCCESS :
            res = TriggerResponse()
            res.success = True
            return res
        elif place_down_seq.status == py_trees.common.Status.FAILURE:
            res = TriggerResponse()
            res.success = False
            return res
        time.sleep(1)
    res = TriggerResponse()
    res.success = False
    return res
    

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("intervention_behavior_trees")
    
    # Service to start behavior
    rospy.Service("start_pickup", Trigger, handle_start_pickup)
    rospy.Service("start_placedown", Trigger, handle_start_placedown)

    rospy.loginfo("Intervention Behavior Initilised!!!")
    rospy.spin()

    