#! /usr/bin/env python3
#python implementation of the ramer douglas peucker algorithm

import numpy as np


def main():
    print("RDP Implementation in python")

"""
    Parameters:
        pose_list: list of points to be simplified
        epsilon: maximum distance between a point and a line segment
"""
def rdp_run(poses, epsilon, angleThreshold):
    angleThreshold = np.radians(angleThreshold)
    max_dist = 0
    max_rotation = 0
    pointIndex = 0
    rotIndex = 0
    index = 0
    d = 0
    for i in range(1, len(poses)-1):
        d = perpendicular_distance(poses[i], poses[0], poses[-1])
        r = angular_distance(poses[i], poses[0], poses[-1])
        if d > max_dist:
            max_dist = d
            pointIndex = i
        if r > max_rotation:
            max_rotation = r
            print(np.degrees(max_rotation))
            rotIndex = i
    
    if (rotIndex < pointIndex and max_rotation > angleThreshold):
        index = rotIndex
    else:
        index = pointIndex
    #find next smallest gap
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
    line = np.array([
        last_pose.position.x - first_pose.position.x,
        last_pose.position.y - first_pose.position.y,
        last_pose.position.z - first_pose.position.z
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
    return np.linalg.norm(np.cross(line, first_point-current_point))/np.linalg.norm(line)

def angular_distance(current_pose, first_pose, last_pose):
    current_angle = euler_from_quaternion(current_pose.orientation)
    first_angle = euler_from_quaternion(first_pose.orientation)
    last_angle = euler_from_quaternion(last_pose.orientation)


    dot_product1 = np.dot(current_angle, first_angle)
    dot_product2 = np.dot(current_angle, last_angle)
    if (dot_product1 > 1):
        angle1 = 0
    elif (dot_product1 < -1):
        angle1 = np.pi
    else:
        angle1 = 2 * np.arccos(np.abs(dot_product1))
    
    if (dot_product2 > 1):
        angle2 = 0
    elif (dot_product2 < -1):
        angle2 = np.pi
    else:
        angle2 = 2 * np.arccos(np.abs(dot_product2))
    difference = abs(angle1 - angle2)
    return angle1

def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return roll, pitch, yaw    



if __name__ == '__main__':
    main()