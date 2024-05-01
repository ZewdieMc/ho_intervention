from task import *
from utils.kinematics_utils import *
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray


class TaskPriority:
    def __init__(self, tasks) -> None:
        self.tasks = tasks
        self.sigmad_pub = rospy.Publisher('sigmad_marker', Marker, queue_size=10)

    def recursive_tp(self, robot):
        P = np.eye(robot.dof)
        dq = np.zeros(robot.dof).reshape(-1,1)
        W=np.eye(robot.dof)
        W[0, 0] = 1
        W[1, 1] = 1
        for i, task in enumerate(self.tasks):
            task.update(robot)
            sigmad = self.tasks[-1].getDesired()
            print("sigmad: ", sigmad)
            if task.name == "Configuration" or task.name == "Position":
                self.publish_sigmad(sigmad)
            if task.isActive():
                J = task.getJacobian()  # Task Jacobian
                Ji_q = J @ P            # Augmented Jacobian

                #compute task velocity
                xi_bar = task.getGain()*0.7 @ task.getError()
                print("error {} ".format(task.name), xi_bar)

                dq_i = DLS(Ji_q, 0.1 , W) @ (xi_bar - J @ dq)

                dq += dq_i

                # Update null space projector
                P = P - np.linalg.pinv(Ji_q) @ J
                # if self.goal_reached(xi_bar):
                #     dq = np.zeros(robot.dof).reshape(-1,1)
        print("dq: ", dq)
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