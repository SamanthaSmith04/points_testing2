#! /usr/bin/env python3
#python implementation of the ramer douglas peucker algorithm

import numpy as np
import math
from geometry_msgs.msg import Pose

def main():
    print("RDP Implementation in python")

"""
    Parameters:
        poses: a pose list to be processed
        epsilon: the maximum spacing between the original data points and the line between the correction points
        angleThreshold: the maximum angle between the original data points and the corrected data points about each axis
"""
global reasons
reasons = []
def rdp_run(poses, epsilon, angleThreshold):
    max_dist = 0
    max_rotation = 0
    rotIndex = 0
    index = 0
    d = 0
    for i in range(1, len(poses)-1):
        d = perpendicular_distance(poses[i], poses[0], poses[-1])
        r, t = angular_distance(poses[i], poses[0])
        if d > max_dist:
            max_dist = d
            index = i
        if r > max_rotation:
            max_rotation = r
            rotIndex = i 
    #if there is a point that exceeds epsilon, split the list and run rdp on both halves
    if max_dist > epsilon: #GETTING TRAPPED HERE :/   
        print("Distance threshold exceeded!")     
        poses1 = poses[:index+1]
        poses2 = poses[index:]
        results1 = rdp_run(poses1, epsilon, angleThreshold) 
        
        results2 = rdp_run(poses2, epsilon, angleThreshold)
        results = np.vstack((results1[:-1],results2))
        
    #correct spot found
    else:
        if max_rotation > angleThreshold: #HERE
            print("Angle threshold exceeded by ", (np.rad2deg(max_rotation) - np.rad2deg(angleThreshold)), " degrees!")
            print("the angle was ", np.rad2deg(max_rotation), " degrees about the ", t, " axis")
            poses1 = poses[:rotIndex+1]
            poses2 = poses[rotIndex:]

            results1 = rdp_run(poses1, epsilon, angleThreshold) 
            
            results = np.vstack((results1[:-1], poses[-1]))
            print("section complete")
        else:
            print("Point accepted!")
            results = np.vstack((poses[0],poses[-1]))
    return results
    
def reasons_for_change():
    return reasons

"""
    Calculates the perpendicular distance between a point and a line segment in 3D space
    Parameters:
        current_pose: point to be measured
        first_pose: first point of the line segment
        last_pose: last point of the line segment
"""
def perpendicular_distance(current_pose, first_pose, last_pose):

    last_point = np.array([
        last_pose.position.x,
        last_pose.position.y,
        last_pose.position.z   
    ])

    current_point = np.array([
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z
    ])

    first_point = np.array([
        first_pose.position.x,
        first_pose.position.y,
        first_pose.position.z
    ])
    line = np.subtract(last_point, first_point)
    out = np.linalg.norm(np.cross(line, np.subtract(first_point, current_point)))/np.linalg.norm(line)
    return out

"""
    Angular distance function
    Calculates the maximum rotation about each axis between two poses
    Parameters:
        current_pose: the current pose
        first_pose: the first pose
    Returns:
        maximum: the maximum rotation about an axis
        max_type: the axis that the maximum rotation occurs about
"""
def angular_distance(current_pose, first_pose):
    difference = Pose()
    #get the difference between quaternion orientations
    result = multiply_inv(current_pose, first_pose)
    difference.orientation.x = result[0]
    difference.orientation.y = result[1]
    difference.orientation.z = result[2]
    difference.orientation.w = result[3]

    #convert to euler angles
    euler_angles = quaternion_to_euler(difference)
    r = euler_angles[0]
    p = euler_angles[1]
    y = euler_angles[2]
    #determine maximum rotation
    max_type = ""
    maximum = max(abs(r),abs(p),abs(y)) 
    """
    print("Maximum: ", np.rad2deg(maximum))
    print("Roll: ", np.rad2deg(r))
    print("Pitch: ", np.rad2deg(p))
    print("Yaw: ", np.rad2deg(y))
    """

    if (maximum == abs(r)):
        #maximum = r
        max_type = "roll"
    elif (maximum == abs(p)):
        #maximum = p
        max_type = "pitch"
    elif (maximum == abs(y)):
        #maximum = y
        max_type = "yaw"

    #print(max_type)

    return maximum, max_type
    

"""
    Quaternion to Euler angles conversion function
    Parameters:
        q: the quaternion to be converted
    Returns:
        roll: the roll angle
        pitch: the pitch angle
        yaw: the yaw angle
"""
def quaternion_to_euler(q):
    x = q.orientation.x
    y = q.orientation.y
    z = q.orientation.z
    w = q.orientation.w

    #roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    #pitch
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi/2.0
    else:
        pitch = np.arcsin(sinp)
    
    #yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

"""
    Normalize Quaternion function
    Parameters:
        quaternion: the quaternion to be normalized
    Returns:
        norm: the normalized quaternion
"""
def normalize(quaternion):
    magnitude = np.linalg.norm(quaternion)
    norm = quaternion / magnitude
    norm /= np.linalg.norm(norm)

    return norm


"""
    Quaternion multiplication function
    Multiplies q0 by the inverse of q1
    Parameters:
        q0: the first quaternion
        q1: the second quaternion
"""
def multiply_inv(q0,q1):

    w0 = q0.orientation.w
    x0 = q0.orientation.x
    y0 = q0.orientation.y
    z0 = q0.orientation.z

    w1 = q1.orientation.w
    x1 = -q1.orientation.x
    y1 = -q1.orientation.y
    z1 = -q1.orientation.z

    w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    y = w0 * y1 + y0 * w1 + z0 * x1 - x0 * z1
    z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    return np.array([x,y,z,w])



if __name__ == '__main__':
    main()