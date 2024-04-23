import numpy as np
import rospy
class TurtlebotManipulator:
    
    def __init__(self):
       # Robot kinematics
       self.dof = 6

       # Subscriber
       self.js_sub = rospy.Subscriber()

    def getEEJacobian(self):
        J = [q]
        return J