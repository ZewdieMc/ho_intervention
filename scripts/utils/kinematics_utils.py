import tf2_ros
from geometry_msgs.msg import TransformStamped
import rospy
import numpy as np
from numpy import cos, sin

def DH(d, theta, a, alpha):
    '''
        Function builds elementary Denavit-Hartenberg transformation matrices 
        and returns the transformation matrix resulting from their multiplication.

        Arguments:
        d (double): displacement along Z-axis
        theta (double): rotation around Z-axis
        a (double): displacement along X-axis
        alpha (double): rotation around X-axis

        Returns:
        (Numpy array): composition of elementary DH transformations
    '''
    # 1. Build matrices representing elementary transformations (based on input parameters).
    # 2. Multiply matrices in the correct order (result in T).
    A = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

    B = np.array([
        [cos(theta), -sin(theta), 0, 0],
        [sin(theta), cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    C = np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    D = np.array([
        [1, 0, 0, 0],
        [0, cos(alpha), -sin(alpha), 0],
        [0, sin(alpha), cos(alpha), 0],
        [0, 0, 0, 1]
    ])

    return A @ B @ C @ D
# Damped Least-Squares
def DLS(A, damping,W=None):
    '''
        Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

        Arguments:
        A (Numpy array): matrix to be inverted
        damping (double): damping factor

        Returns:
        (Numpy array): inversion of the input matrix
    '''    
    if W is None:
        W = np.eye(A.shape[1])
    dls = np.linalg.inv(W)@ A.T @ np.linalg.inv(A@np.linalg.inv(W)@A.T + damping**2 * np.eye(A.shape[0]))
    return dls


def kinematics(d, theta, a, alpha, Tb):
    T = [Tb] # Base transformation
    # For each set of DH parameters:
    for i in range(len(d)):
        # 1. Compute the DH transformation matrix.
        dh = DH(d[i],theta[i], a[i], alpha[i])

        # 2. Compute the resulting accumulated transformation from the base frame.
        T_accumulated = T[-1] @ dh
        
        # 3. Append the computed transformation to T.
        T.append(T_accumulated)
    return T

def jacobian(T, revolute):
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint

        Returns:
        (Numpy array): end-effector Jacobian
    '''
    # 1. Initialize J and O.
    J = np.zeros((6, len(T)-1)) # Empty Jacobian (6 x N) where N - number of joints
    O = T[-1][:3, -1]# End-effector position    
    # 2. For each joint of the robot[1.11747103 0.47111084 0.        ]
    for i in range(len(T) - 1):
        # a. Extract z and o.
        z = T[i][0:3, 2].reshape(1, 3).flatten()
        o = T[i][0:3, 3].reshape(1, 3).flatten()
  
        # b. Check joint type.
        if revolute[i]:
            # Revolute joint
            j_3 = np.cross(z, (O - o)) #cross product between z and o
            J[:, i] = np.concatenate((j_3, z), axis=0).reshape(6).tolist()
        else:
            # Prismatic joint
            J[:, i] = np.concatenate((z, np.zeros(3)), axis=0).reshape(6).tolist()

    return J

# Compute the transformation matrix given the rotation and translation 
def compute_transformation(eta):
            # Mobile base position with respect to the world  
        T = np.array([
                            [cos(eta[2]) , -sin(eta[2]), 0, eta[0]],
                            [sin(eta[2]), cos(eta[2]), 0, eta[1]],
                            [0             ,  0               , 1 , 0],
                            [0             ,  0               , 0, 1]
                                                                    ])
        return T
