#! /usr/bin/env python3
#A program that is used for testing purposes to generate datasets of poses for testing
#Author: Samantha Smith, smith.15485@osu.edu

import numpy as np

def main():
    file = open("line.txt", "w")
    """
    xvalues = np.cos(np.arange(0,10,.05))
    yvalues = np.sin((np.arange(0,10,.05)))
    #zvalues = np.zeros(len(xvalues))
    zvalues = np.arange(0,100, .05)
    zvalues = np.divide(zvalues, 10)
    quaternion = np.zeros((len(xvalues),4))
    for i in range(len(xvalues)):
        angle = i * np.pi / 50  # Angle for spiral motion
        quaternion[i] = [1.0, np.cos(angle), np.cos(angle),1.0]  # Quaternion values
    """
    xvalues, yvalues, zvalues, quaternion = generate_raster(.005)
    for i in range(len(xvalues)):
        file.write("" + xvalues[i].__str__() + " " + yvalues[i].__str__() + " " + zvalues[i].__str__() + " " + quaternion[i,0].__str__() + " " + quaternion[i,1].__str__() + " " + quaternion[i,2].__str__() + " " + quaternion[i,3].__str__() + "\n")
    file.close()

def generate_raster(inPointSpacing):
    x1 = np.arange(0, 5, inPointSpacing)
    y1 = np.zeros(len(x1))

    y2 = np.arange(0, 3, inPointSpacing)
    x2 = np.full(len(y2), 5)

    x3 = np.arange(3, 5, inPointSpacing)
    x3 = np.flip(x3)
    y3 = np.full(len(x3), 3)

    y4 = np.arange(3, 5, inPointSpacing)
    x4 = np.full(len(y4), 3)

    x5 = np.arange(3, 7, inPointSpacing)
    y5 = np.full(len(x5), 5)

    xpoints = np.concatenate((x1, x2, x3, x4, x5))
    ypoints = np.concatenate((y1, y2, y3, y4, y5))
    zpoints = np.zeros((len(xpoints)))

    quaternion = np.zeros((len(xpoints),4))
    for i in range(len(xpoints)):
        angle = i /10
        #quaternion[i] = [i/len(xpoints), (5*i)/(len(xpoints)**2), (2*i/len(xpoints))**2,1.0]  # Quaternion values
        quaternion[i] = [np.sin(angle)**2, np.cos(angle), np.cos(angle),1.0]  # Quaternion values
    return xpoints, ypoints, zpoints, quaternion

def generate_line(inPointSpacing):
    xpos = np.arange(0, 5, inPointSpacing)
    ypos = np.zeros(len(xpos))
    zpos = np.zeros(len(xpos))

    quaternion = np.zeros((len(xpos),4))
    for i in range(1,len(xpos)):
        angle = i * np.pi / 10000  # Angle for spiral motion
        quaternion[i] = [i/len(xpos), (5*i)/(len(xpos)**2), (2*i/len(xpos))**2,1.0]  # Quaternion values
    return xpos, ypos, zpos, quaternion

if __name__ == '__main__':
    main()