#! /usr/bin/env python3
#python implementation of the ramer douglas peucker algorithm

import numpy as np
from geometry_msgs.msg import Quaternion

def main():
    print("RDP Implementation in python")

"""
    Parameters:
        pose_list: list of points to be simplified
        epsilon: maximum distance between a point and a line segment
"""
def rdp_run(poses, epsilon, angleThreshold):
    max_dist = 0
    max_rotation = -360
    pointIndex = 0
    rotIndex = 0
    index = 0
    d = 0
    for i in range(1, len(poses)-1):
        d = perpendicular_distance(poses[i], poses[0], poses[-1])
        r = angular_distance(poses[i], poses[0])
        if d > max_dist:
            max_dist = d
            pointIndex = i
        if r > max_rotation:
            max_rotation = r
            rotIndex = i
    
    if (rotIndex > pointIndex):
        index = pointIndex
    else:
        index = rotIndex
    if max_dist > epsilon or max_rotation > angleThreshold:
        
        poses1 = poses[:index+1]
        poses2 = poses[index:]
        results1 = rdp_run(poses1, epsilon, angleThreshold) #test ERROR HERE
        
        results2 = rdp_run(poses2, epsilon, angleThreshold)
        results = np.vstack((results1[:-1],results2))
    #correct spot found
    else:
        results = np.vstack((poses[0],poses[-1]))
    return results
    
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

    line = last_point - first_point
    return np.linalg.norm(np.cross(line, first_point-current_point))/np.linalg.norm(line)

def angular_distance(current_pose, first_pose):
    """current_angle = euler_from_quaternion(current_pose.orientation)
    first_angle = euler_from_quaternion(first_pose.orientation)

    dot_product = np.dot(current_angle, first_angle)
    if dot_product < -1:
        dot_product = -1
    elif dot_product > 1:
        dot_product = 1
    angle = np.arccos(2 * (dot_product ** 2) - 1)
    angle = np.degrees(angle)
    #if (angle >= 180):
     #   angle = 180 - angle
    return angle"""
    current_angle = current_pose.orientation
    w = first_pose.orientation.w
    x = -1* first_pose.orientation.x
    y = -1*first_pose.orientation.y
    z = -1*first_pose.orientation.z
    first_angle_inverse = Quaternion(x,y,z,w)
    difference = current_angle * first_angle_inverse
    angle_difference = 2 * np.arccos(np.abs(difference.w))
    return angle_difference

def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    magnitude = np.sqrt(w**2 + x**2 + y**2 + z**2)

    w = correctValues(w) / magnitude
    x = correctValues(x) / magnitude
    y = correctValues(y) / magnitude
    z = correctValues(z) / magnitude

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))   
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return roll, pitch, yaw    

def correctValues(value):
    if (value > 1):
        value = 1
    elif (value < -1):
        value = -1
    return value

if __name__ == '__main__':
    main()