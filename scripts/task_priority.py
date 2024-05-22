from task import *
from utils.kinematics_utils import *
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray


class TaskPriority:
    def __init__(self, tasks) -> None:
        self.tasks = tasks
        self.sigmad_pub = rospy.Publisher('sigmad_marker', Marker, queue_size=10)
        self.joint_vel_limits = np.array([0.2, 0.5, 0.2, 0.2, 0.2, 0.2]).reshape(-1,1)
    def recursive_tp(self, robot):
        print("number of tasks: ", len(self.tasks))
        P = np.eye(robot.dof)
        dq = np.zeros(robot.dof).reshape(-1,1)
        W=np.eye(robot.dof)
        W[0, 0] = 1
        W[1, 1] = 1
        for i, task in enumerate(self.tasks):
            task.update(robot)
            sigmad = self.tasks[i].getDesired()
            print("sigmad: {}".format(task.name), sigmad)
            if task.name == "Configuration" or task.name == "Position" or task.name == "ArmOnly" or task.name == "BaseOnly":
                # self.publish_sigmad(sigmad)
                ...
            # #! 
            # if task.name == "BaseOnlyPositionTask":
            #     for i in range(2, 6):
            #         W[i, i] = 0
            # if task.name == "Configuration":
                # for i in range(0, 2):
                    # W[i, i] = 10

            if task.isActive():
                J = task.getJacobian()  # Task Jacobian
                print("Jacobian: ", J.shape)
                print("p: ", P.shape)
                Ji_q = J @ P            # Augmented Jacobian

                #compute task velocity
                print("task gain: ", task.getGain().shape)
                print("error: ", task.getError().shape)
                xi_bar = task.getGain()*0.7 @ task.getError()
                print("error {} ".format(task.name), task.getError())

                dq_i = DLS(Ji_q, 0.1 , W) @ (xi_bar - J @ dq)
                print("dq_i: ", dq_i.shape)
                print("dq: ", dq.shape)
                dq += dq_i

                # Update null space projector
                P = P - np.linalg.pinv(Ji_q) @ J #! solution weighting in Ji_q

        print("dq before: ", dq)
        print("dq before: ", dq.shape)

        dq = self.velocity_scaling(dq)
        print("dq after: ", dq)
        return dq       
    
    def velocity_scaling(self, dq):
        s = max(np.abs(dq[2:]) / self.joint_vel_limits[2:])

        dq[0] = max(min(dq[0], 0.15), -0.15)
        dq[1] = max(min(dq[1], 0.3), -0.3)

        if dq[0]> 0.03:
            dq[0] = max(dq[0], 0.5)
        elif dq[0]< -0.03:
            dq[0] = min(dq[0], -0.5)

        # if dq[1]> 0:
        #     dq[1] = max(dq[1], 0.15)
        # elif dq[1]< 0:
        #     dq[1] = min(dq[1], -0.15)
            
        if s > 1:
            dq[2:] =  (dq[2:]) / float(s)
            return dq
        else:
            return dq

    def goal_reached(self, error):
        return np.linalg.norm(error) < 0.01

    def publish_sigmad(self, msg):

        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = msg[0]
        marker.pose.position.y = msg[1]
        marker.pose.position.z = msg[2]
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.g = 1.0
        marker.color.r = 0.0
        marker.color.a = 0.5
        self.sigmad_pub.publish(marker)