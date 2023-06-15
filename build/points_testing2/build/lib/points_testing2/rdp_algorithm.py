#! /usr/bin/env python3
#python implementation of the ramer douglas peucker algorithm

import numpy as np
import math
from geometry_msgs.msg import Pose

def main():
    print("RDP Implementation in python")

"""
    Parameters:
        pose_list: list of points to be simplified
        epsilon: maximum distance between a point and a line segment
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
        r, type = angular_distance(poses[i], poses[0])
        if d > max_dist:
            max_dist = d
            index = i
        if r > max_rotation:
            max_rotation = r
            rotIndex = i 
    #if there is a point that exceeds epsilon, split the list and run rdp on both halves
    if max_dist > epsilon: #GETTING TRAPPED HERE :/
        
        poses1 = poses[:index+1]
        poses2 = poses[index:]
        results1 = rdp_run(poses1, epsilon, angleThreshold) 
        
        results2 = rdp_run(poses2, epsilon, angleThreshold)
        results = np.vstack((results1[:-1],results2))
    #correct spot found
    else:
        if (max_rotation > angleThreshold): #HERE
            print(max_rotation)
            poses1 = poses[:rotIndex+1]
            poses2 = poses[rotIndex:]

            results1 = rdp_run(poses1, epsilon, angleThreshold) 
            results2 = rdp_run(poses2, epsilon, angleThreshold)
            
            results = np.vstack((results1[:-1],results2))
        else:
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

def angular_distance(current_pose, first_pose):
    difference = Pose()
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
    max_type = ""
    maximum = max(abs(r),abs(p),abs(y))
    if (maximum == r):
        max_type = "roll"
    elif (maximum == p):
        max_type = "pitch"
    elif (maximum == y):
        max_type = "yaw"

    return maximum, max_type
    
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
        pitch = np.sign(sinp) * np.pi/2
    else:
        pitch = np.arcsin(sinp)
    
    #yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def normalize(quaternion):
    magnitude = np.linalg.norm(quaternion)
    norm = quaternion / magnitude
    norm /= np.linalg.norm(norm)

    return norm

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