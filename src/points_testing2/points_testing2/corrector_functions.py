#! /usr/bin/env python3
#A collection of functions used to downsample a set of poses and report the difference between the original and corrected poses
#Author: Samantha Smith, smith.15485@osu.edu

import numpy as np
from points_testing2 import rdp_algorithm
import time
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

global file_path
file_path = "/home/samubuntu/AA_DEVEL/ws_points_testing2/src/points_testing2/points_testing2/test_values/"



#main
def main():
    print("=====================================================")
    print("                     Point Tests                     ")
    print("=====================================================")
    print("Should be called with downsample(epsilon, angle_threshold, inputFileName)")

"""
    Downsample function
    Reads/Generates a list of Poses to process with the rdp algorithm
    Parameters:
        epsilon: the maximum spacing between the original data points and the line between the correction points
        angle_threshold: the maximum angle between the original data points and the corrected data points about each axis
        initial_poses: a PoseArray of poses to be downsampled
    Returns:
        pose_array: a geometry_msgs.msg PoseArray of the corrected poses
"""
def downsample(epsilon, angle_threshold, initial_poses):
    startTime = time.perf_counter()


    
    print("Processing " + len(initial_poses.poses).__str__() + " points...")
    angle_threshold = np.deg2rad(angle_threshold) #convert angle threshold to radians
    
    #run the rdp algorithm on the poses
    corrections = rdp_algorithm.rdp_run(initial_poses.poses, epsilon, angle_threshold)
    
    #calculate the difference between the original poses and the corrected poses
    print("Calculating delta values...")
    max_dist, max_angle = delta(initial_poses, corrections)
    end_time = time.perf_counter()
    print("Time to run: " + (end_time - startTime).__str__() + "s")

    print("Reformatting data...")
    #write corrected points to a pose array
    pose_array = PoseArray()
    for i in range(len(corrections)):
        pose_array.poses.append(corrections[i,0])

    print("Number of points used in correction: " + len(corrections).__str__())
    #print out the delta values
    ##TODO - TEMPORARY PRINT MAY BE SENT AS A SERVICE RESPONSE IN THE FUTURE
    print("Maximum Values between each correction pose:")
    for i in range(len(max_dist)):
        print("Delta " + (i+1).__str__() + ": " + max_dist[i].__str__())
        print("Max Angle " + (i+1).__str__() + ": " + np.rad2deg(max_angle[i]).__str__())
    
    return pose_array
    

"""
    Delta function
    Calculates the delta values for each correction pose
    Parameters:
        points: the original data poses
        corrections: the corrected data pose
    Returns:
"""
def delta(points, corrections):
    index = 0
    corr_len = len(corrections) -1
    dist = np.zeros(corr_len)
    angles = np.zeros(corr_len)
    #calculate the delta values for distance for each correction pose
    for cPos in range(corr_len):
        while points.poses[index] != corrections[cPos+1,0]:
            d = rdp_algorithm.perpendicular_distance(points.poses[index], corrections[cPos,0], corrections[cPos+1,0])
            if (abs(d) > abs(dist[cPos])):
                dist[cPos] = d
            index +=1
    #calculate the delta values for angle for each correction pose
    index = 1
    for cPos in range(corr_len):
        while points.poses[index] != corrections[cPos+1,0]:
            angle, type = rdp_algorithm.angular_distance(points.poses[index], corrections[cPos,0])
            if (abs(angle) > abs(angles[cPos])):
                angles[cPos] = angle
            index += 1
    print("Maximum angle between each correction pose:")
    for cPos in range(corr_len): 
        angle_between_poses = rdp_algorithm.angular_distance(corrections[cPos+1,0], corrections[cPos,0])
        angle = np.degrees(angle_between_poses[0])
        print("Max rotation about the ", angle_between_poses[1], " between poses ",cPos+1, " and ",(cPos+2), ": ", angle)

    print("Delta values calculated!")
    return dist, angles

"""
    Write corrections to a file
    Outputs the corrected points to a file
    Parameters:
        corrections: the corrected points
        outputFileName: the name of the file to write to
"""
def write_corrections_to_file(corrections, outputFileName):
    outFile = open(file_path + outputFileName, "w")
    print("Writing corrected points to " + outputFileName + "...")
    for i in range(len(corrections)):
        outFile.write(corrections[i,0].__str__() + " " + corrections[i,1].__str__() + " " + corrections[i,2].__str__() + "\n")
    outFile.close()

"""
    Write delta values to a file
    Outputs the delta values to a file
    Parameters:
        deltaOutput: the name of the file to write to
        deltas: the delta values
"""
def write_delta_to_file(deltaOutput, deltas):
    deltaFile = open(file_path + deltaOutput, "w")
    for i in range (len(deltas)):
        deltaFile.write(deltas[i].__str__() + "\n")
    deltaFile.close()

#main start
if __name__ == '__main__':
    main()