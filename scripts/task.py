import numpy as np

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''

    def __init__(self, name, desired) -> None:
        self.name = name # task title
        self.sigma_d = desired # desired sigma
        self.FF = np.zeros(desired.shape) # feed forward velocity
        self.K = np.eye(desired.shape[0]) # gain matrix
        self.joint = None
        self.is_active = True
    
        '''
        Method setting the feed forward velocity vector.
        '''
    def setFeedForward(self, FF):
            self.FF = FF

    '''
        Method returning the feed forward velocity vector.
    '''
    def getFeedForward(self):
        return self.FF

    '''
        Method setting the gain matrix.
    '''    
    def setGain(self, K):
        self.K = K


    '''
        Method returning the gain matrix.
    '''
    def getGain(self):
        return self.K
    

    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def update(self, robot):

        pass

    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value


    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J


    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err
    
    '''
        Method returning the status of the task.
    '''

    def isActive(self):
        return self.is_active

class PositionTask(Task):
    '''
    Subclass of Task, representing a 3D position task.
    '''
    def __init__(self, name, desired) -> None:
        super().__init__(name, desired)
        self.err = np.zeros(desired.shape) # error vector
        self.J = np.zeros(0)


    '''
    Method updating the task variables.
    '''
    def update(self, robot):
        self.err = self.getDesired() - robot.getEEPose()[:3]
        self.J = robot.getEEJacobian()[:3, :]
        print("J:",self.J)
        # self.J[:, 0:2] = 0


class OrientationTask(Task):
    '''
    Subclass of Task, representing a 3D orientation task.
    '''
    def __init__(self, name, desired,) -> None:
        super().__init__(name, desired)
        self.err = np.zeros(desired.shape) # error vector
        self.J = np.zeros(0)

    '''
    Method updating the task variables.
    '''
    def update(self, robot):
        self.err = self.getDesired() - robot.getEEPose()[3:]
        self.err[-1] = wrap_angle(self.err[-1])
        self.J = robot.getEEJacobian()[3:, :]

class ConfigurationTask(Task):
    '''
    Subclass of Task, representing a configuration task.
    '''
    def __init__(self, name, desired) -> None:
        super().__init__(name, desired)
        self.err = np.zeros(desired.shape) # error vector
        self.J = np.zeros(0)

    '''
    Method updating the task variables.
    '''
    def update(self, robot):
        self.J = robot.getEEJacobian()
        self.err = self.getDesired() - robot.getEEPose()
        self.err[-1] = wrap_angle(self.err[-1])

class JointLimitTask(Task):
    '''
    Subclass of Task, representing a joint limit task.
    '''
    def __init__(self, name, threshold, Qli, joint):
        super().__init__(name, threshold)
        self.joint = joint
        self.alpha = threshold[0]
        self.delta = threshold[1]
        self.qi_min = Qli[0]
        self.qi_max = Qli[1]
        self.active = -1
        self.K  = np.eye(1)
        self.err = np.array([1])    

    def update(self, robot):
        self.J = np.zeros((1, robot.getDOF()))
        self.J[:, self.joint+2] = 1

        qi = robot.arm.getJointPos(self.joint)

        if (self.active == 0) and (qi >= (self.qi_max - self.alpha)):
            self.active = -1
        elif (self.active == 0) and (qi <= (self.qi_min + self.alpha)):
            self.active = 1
        elif (self.active == -1) and (qi <= (self.qi_max - self.delta)):
            self.active = 0
        elif (self.active == 1) and (qi >= (self.qi_min + self.delta)):
            self.active = 0

        self.err[0] = self.active
        print("activation:{} joint{} q{} ".format(self.active, self.joint, qi)) 

    def isActive(self):
        return True if self.active != 0 else False
    

class JointPositionTask(Task):
    '''
    Subclass of Task, representing a joint position task.
    '''
    def __init__(self, name, desired, joint) -> None:
        super().__init__(name, desired)
        self.joint = joint
    
    def update(self, robot):
        """
        Update the task variables
        """
        self.err = (self.getDesired() - robot.arm.getJointPos(self.joint))
        self.J = np.zeros((1, robot.getDOF()))
        self.J[:, self.joint + 2] = 1

class BaseOrientationTask(Task):
    '''
    Subclass of Task, Orientation of the base
    '''
    def __init__(self, name, desired) -> None:
        super().__init__(name, desired)
    
    def update(self, robot):
        """
        Update the task variables
        """
        self.err = np.array([wrap_angle(self.getDesired() - robot.eta[-1])]).reshape(-1,1)
        self.J = np.zeros((1, robot.getDOF()))
        self.J[:, 0] = 1

    
#! Use Planning to go to the aruco box
class BaseConfigurationTask(Task):
    '''
    Subclass of Task, representing mobile base position task.
    '''
    def __init__(self, name, desired) -> None:
        super().__init__(name, desired)
    
    def update(self, robot):
        """
        Update the task variables
        """
        print("desired:", self.getDesired())
        print("eta:", robot.eta.reshape(-1, 1))
        self.err = (self.getDesired() - robot.eta.reshape(-1, 1))
        self.err[-1] = wrap_angle(self.err[-1])
        self.J = robot.getBaseOnlyJacobian()
        self.J = np.block([[self.J[:3, :]], [self.J[-1, :]]])
        print("JBaseOnly:\n", self.J)
        

class ArmOnlyPositionTask(Task):
    '''
    Subclass of Task, representing an arm only task.
    '''
    def __init__(self, name, desired) -> None:
        super().__init__(name, desired)
    
    def update(self, robot):
        """
        Update the task variables
        """
        self.err = (self.getDesired() - robot.getEEPose())
        self.J = robot.getEEJacobian()
        self.J[:, :2] = 0
