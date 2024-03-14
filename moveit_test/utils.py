import math
from typing import List, Tuple
import PyKDL
from dataclasses import dataclass
from geometry_msgs.msg import (
    Pose,
    PoseStamped, 
    Vector3,
    Quaternion
)


def multiply_pose(p1: Pose, p2: Pose) -> Pose:
    '''
    Use KDL to multiply two poses together.
    Args:
        p1 (Pose): Pose of the first frame
        p2 (Pose): Pose of the second frame
    Returns:
        Pose: Pose of the resulting frame
    '''

    o1 = p1.orientation
    frame1 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o1.x, o1.y, o1.z, o1.w),
        PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z))

    o2 = p2.orientation
    frame2 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o2.x, o2.y, o2.z, o2.w),
        PyKDL.Vector(p2.position.x, p2.position.y, p2.position.z))

    frame3 = frame1 * frame2

    # return the resulting pose from frame3
    pose = Pose()
    pose.position.x = frame3.p.x()
    pose.position.y = frame3.p.y()
    pose.position.z = frame3.p.z()

    q = frame3.M.GetQuaternion()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def rpy_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        q (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    '''
    
    R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    return R.GetRPY()

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    q_msg = Quaternion()
    q_msg.w = q[0]
    q_msg.x = q[1]
    q_msg.y = q[2]
    q_msg.z = q[3]

    return q_msg

def build_pose(x,y,z,q : Quaternion)->Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = q
    return p


def rad_to_deg_str(radians: float) -> str:
    '''
    Converts radians to degrees in the domain [-PI, PI]
    Args:
        radians (float): value in radians
    Returns:
        str: String representing the value in degrees
    '''
    
    degrees = math.degrees(radians)
    if degrees > 180:
        degrees = degrees - 360
    elif degrees < -180:
        degrees = degrees + 360

    if -1 < degrees < 1:
        degrees = 0 
    
    return f'{degrees:.0f}' + chr(176)