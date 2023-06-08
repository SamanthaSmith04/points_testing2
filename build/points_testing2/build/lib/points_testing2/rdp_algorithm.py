#! /usr/bin/env python3
#python implementation of the ramer douglas peucker algorithm

import numpy as np


def main():
    print("RDP Implementation in python")

"""
    Parameters:
        points_list: list of points to be simplified
        epsilon: maximum distance between a point and a line segment
"""
def rdp_run(points_list, epsilon):
    max = 0
    index = 0
    d=0
    for i in range(1, len(points_list)-1):
        d = perpendicular_distance(points_list[i], points_list[0], points_list[-1])
        if d > max:
            max = d
            index = i

    #find next smallest gap
    if max > epsilon:
        results1 = rdp_run(points_list[:index+1], epsilon)
        results2 = rdp_run(points_list[index:], epsilon)

        results = np.vstack((results1[:-1],results2))
    #correct spot found
    else:
        results = np.vstack((points_list[0],points_list[-1]))

    return results
    
"""
    Calculates the perpendicular distance between a point and a line segment in 3D space
    Parameters:
        current_point: point to be measured
        first_point: first point of the line segment
        last_point: last point of the line segment
"""
def perpendicular_distance(current_point, first_point, last_point):
    line = last_point - first_point
    return np.linalg.norm(np.cross(line, first_point-current_point))/np.linalg.norm(line)

if __name__ == '__main__':
    main()