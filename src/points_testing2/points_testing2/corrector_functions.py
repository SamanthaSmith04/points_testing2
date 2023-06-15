#! /usr/bin/env python3
#TODO FIX FUNCTION HEADERS

import numpy as np
from points_testing2 import rdp_algorithm
import time
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

global file_path
file_path = "/home/samubuntu/AA_DEVEL/ws_points_testing2/src/points_testing2/points_testing2/"



#main
def main():
    print("=====================================================")
    print("                     Point Tests                     ")
    print("=====================================================")
    t = downsample(0.05, 20.0, "points2.txt")

"""
    Downsample function
    Reads/Generates a list of points to process with the rdp algorithm and plots them using matpltlib
    Outputs to either a file or the console
    Parameters:
        epsilon: the maximum spacing between the original data points and the line between the correction points
        inputFileName: the name of the file to read the points from
        outputFileName: the name of the file to write the corrected points to
        minYValue: the minimum y value to generate points for
        maxYValue: the maximum y value to generate points for
        inPointSpacing: the spacing between the points to generate
"""
def downsample(epsilon, angleThreshold, inputFileName):
    startTime = time.perf_counter()

    if (inputFileName != ""):
        points = PoseArray()
        points.poses = get_points_from_file(inputFileName)
    else:
        print("Invalid file name!")
        exit()
    print("Processing " + len(points.poses).__str__() + " points...")
    angleThreshold = np.deg2rad(angleThreshold)
    corrections = rdp_algorithm.rdp_run(points.poses, epsilon, angleThreshold) #test PROBLEM HERE
    print("Calculating delta values...")
    max_dist, max_angle = delta(points, corrections)
    endTime = time.perf_counter()
    print("Time to run: " + (endTime - startTime).__str__() + "s")

    print("Reformatting data...")
        #write corrected points to a pose array
    pose_array = PoseArray()
    for i in range(len(corrections)):
        pose_array.poses.append(corrections[i,0])

    print("Number of points used in correction: " + len(corrections).__str__())
    for i in range(len(max_dist)):
        print("Delta " + (i+1).__str__() + ": " + max_dist[i].__str__())
        print("Angle " + (i+1).__str__() + ": " + np.rad2deg(max_angle[i]).__str__())
    
    ##TEMPORARY PRINT CORRECTIONS
    for i in range(len(pose_array.poses)):
        print("Pose " + (i+1).__str__() + ":")
        print("Position: " + pose_array.poses[i].position.__str__())
        print("Orientation: " + pose_array.poses[i].orientation.__str__())
    
    return pose_array#, max_dist
    

"""
    Delta function
    Calculates the delta values for each correction point
    Parameters:
        points: the original data points
        corrections: the corrected data points
"""
def delta(points, corrections):
    index = 0
    corr_len = len(corrections) -1
    print(":)")
    dist = np.zeros(corr_len)
    angles = np.zeros(corr_len)

    for cPos in range(corr_len):
        while points.poses[index] != corrections[cPos+1,0]:
            d = rdp_algorithm.perpendicular_distance(points.poses[index], corrections[cPos,0], corrections[cPos+1,0])
            if (abs(d) > abs(dist[cPos])):
                dist[cPos] = d
            index +=1 
    print("Delta values calculated!")

    index = 1
    for cPos in range(corr_len):
        while points.poses[index] != corrections[cPos+1,0]:
            angle, type = rdp_algorithm.angular_distance(points.poses[index], corrections[cPos,0])
            #print("comparing: \n" + points.poses[index].__str__() + "\n to \n" + corrections[cPos,0].__str__())
            #print("angle: " + angle.__str__())
            if (abs(angle) > abs(angles[cPos])):
                angles[cPos] = angle
            index += 1
    print(rdp_algorithm.reasons_for_change())
    for cPos in range(corr_len): 
        angle_between_poses = rdp_algorithm.angular_distance(corrections[cPos,0], corrections[cPos+1,0])
        angle = np.degrees(angle_between_poses[0])
        print("Max rotation about the ", angle_between_poses[1], " between poses ",cPos, " and ",(cPos+1), ": ", angle)

    return dist, angles


"""
    Read points from a file
    Parameters:
        inputFileName: the name of the file to read from
"""
def get_points_from_file(inputFileName):

    poses_array = []
    #read in data from the input file
    print("Reading in data from " + inputFileName + "...")
    inFile = open(file_path + inputFileName, "r")
    for line in inFile:
        current_pose = Pose()
    
        line = line.split()

        current_pose.position.x = float(line[0])
        current_pose.position.y = float(line[1])
        current_pose.position.z = float(line[2])
        current_pose.orientation.x = float(line[3])
        current_pose.orientation.y = float(line[4])
        current_pose.orientation.z = float(line[5])
        current_pose.orientation.w = float(line[6])
        poses_array.append(current_pose)
    inFile.close()
    return poses_array

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