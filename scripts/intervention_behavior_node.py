#!/usr/bin/env python3

import py_trees
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from ho_intervention.msg import MoveJointAction, MoveToGoalAction, MoveToGoalActionGoal,MoveJointActionGoal
import time
from tf.transformations import quaternion_from_euler
from std_srvs.srv import SetBool

class setGoal (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        #blackboard variable for move_base behavior
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        #blackboard variable for move_swiftpro behavior
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.READ)

        
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("offset_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("platform_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [setGoal::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [setGoal::initialise()]" % self.name)

        

    def update(self):
        # goal for the base
        goal = PoseStamped()
        goal.pose.position.x = 1.7
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0

        q = quaternion_from_euler(0,0,0)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        self.blackboard.goal = goal

        # goal for configuration with offset
        offset_goal = PoseStamped()
        offset_goal.pose.position.x = 2
        offset_goal.pose.position.y = 0.0
        offset_goal.pose.position.z = -0.3

        q = quaternion_from_euler(0,0,0)
        offset_goal.pose.orientation.x = q[0]
        offset_goal.pose.orientation.y = q[1]
        offset_goal.pose.orientation.z = q[2]
        offset_goal.pose.orientation.w = q[3]
        self.blackboard.offset_goal = offset_goal


        # goal for the swiftpro to grap the box 
        pickup_goal = PoseStamped()
        pickup_goal.pose.position.x = 2.0
        pickup_goal.pose.position.y = 0.0
        pickup_goal.pose.position.z = -0.135

        q = quaternion_from_euler(0, 0, 0)
        pickup_goal.pose.orientation.x = q[0]
        pickup_goal.pose.orientation.y = q[1]
        pickup_goal.pose.orientation.z = q[2]
        pickup_goal.pose.orientation.w = q[3]
        self.blackboard.pickup_goal = pickup_goal
        

        # goal for the swiftpro to move the box to platform #! Danger.....dont use this task for platform...the robot will crash
        platform_goal = PoseStamped()
        platform_goal.pose.position.x = 1.60775
        platform_goal.pose.position.y = -0.00
        platform_goal.pose.position.z = -0.45

        q = quaternion_from_euler(0, 0, 0)
        platform_goal.pose.orientation.x = q[0]
        platform_goal.pose.orientation.y = q[1]
        platform_goal.pose.orientation.z = q[2]
        platform_goal.pose.orientation.w = q[3]
        self.blackboard.platform_goal = platform_goal
       



        return py_trees.common.Status.SUCCESS

# class getGoal (py_trees.behaviour.Behaviour):
#     def __init__(self, name):
#         super().__init__(name)
#         self.counter = 0

#         self.blackboard = self.attach_blackboard_client(name=self.name)
#         self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

#     def update(self):
#         self.counter += 1
#         print("Get Goal")
#         return py_trees.common.Status.SUCCESS


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
        

class moveBase (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(moveBase, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

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
            goal.goal = self.blackboard.goal
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

        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.READ)

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
            goal.goal = self.blackboard.pickup_goal if self.name == "move_down" else self.blackboard.platform_goal
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
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(EnableSuction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.READ)

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


class PlaceBox (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(PlaceBox, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [PlaceBox::setup()]" % self.name)
        self.client = actionlib.SimpleActionClient('move_kobuki', MoveJointAction)
        rospy.loginfo("Waiting for placebox action...")
        self.client.wait_for_server()

    def initialise(self):
        self.logger.debug("  %s [PlaceBox::initialise()]" % self.name)

    def update(self):
       print(self.client.get_state())
       if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            rospy.loginfo('Send goal to place box')
            goal = MoveJointActionGoal()
            goal.joint = 0
            goal.position = -1.5
            self.client.send_goal(goal)
            self.goal_sent = True
       if self.client.get_state() == actionlib.GoalStatus.PREEMPTED:
            rospy.logerr('Place box Fail')
            self.goal_sent = False
            return py_trees.common.Status.FAILURE
       elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo('Place box Success')
        self.goal_sent = False
        return py_trees.common.Status.SUCCESS
 
 
    # def place_box(self):
    #     rospy.logwarn("Calling place box")
    #     rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
    #     path = []
    #     try:
    #         enable_suction = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
    #         resp = enable_suction(False)
            
    #         return resp.success
        
    #     except rospy.ServiceException as e:
    #         print(f"Service call failed: {e}")
    #         return False


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("intervention_behavior_trees")

    # Create Behaviors
    set_goal = setGoal("set_goal")
    move_base = moveBase("move_base")
    move_offset = moveTurtlebot("move_offset")
    move_down = moveSwiftpro("move_down")
    enable_suction = EnableSuction("grab_box")
    move_up = moveTurtlebot("move_up")
    place_box = PlaceBox("place_box")


    # go to pickup spot sequence
    go_to_seq = py_trees.composites.Sequence(name="pick_seq", memory=True)
    go_to_seq.add_children([set_goal,move_base, move_offset, move_down,enable_suction,move_up, place_box])
    # retry_pickup = py_trees.decorators.Retry(name="retry_pickup",child=go_to_seq, num_failures=6)

    # pick_place_seq = py_trees.composites.Sequence(name="Pick and Place Object", memory=True)
    # pick_place_seq.add_children([is_finished,retry_pickup,get_object,find_drop_spot,move_to_drop,let_object])
    # repeat_pick_place = py_trees.decorators.Repeat(name="repeat_pick_place",child=pick_place_seq,num_success=100)


    # root = py_trees.composites.Parallel(name="root",policy= py_trees.common.ParallelPolicy.SuccessOnSelected([is_finished]))
    # root.add_children([is_finished,repeat_pick_place])
    
    print("Call setup for all tree children")
    py_trees.display.render_dot_tree(go_to_seq)
    go_to_seq.setup_with_descendants() # call setup() of all behaviors in the tree (set up ROS topic, service ...)
    print("Setup done!\n\n")
    py_trees.display.ascii_tree(go_to_seq)

    time.sleep(1)    
    while not rospy.is_shutdown():
        go_to_seq.tick_once() # untill the root return sth.
        time.sleep(1)