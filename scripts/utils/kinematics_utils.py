import tf2_ros
from geometry_msgs.msg import TransformStamped
import rospy
import numpy as np


def DLS(A, damping, W = None):
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