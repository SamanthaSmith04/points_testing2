import numpy as np
def main():
    file = open("points.txt", "w")
    xvalues = np.cos(np.arange(0,100,.05))
    yvalues = np.sin(np.arange(0,100,.05))
    zvalues = np.arange(0,100, .05)
    zvalues = np.divide(zvalues, 10)
    quaternion = np.zeros((len(xvalues),4))
    for i in range(len(xvalues)):
        angle = i * np.pi / 50  # Angle for spiral motion
        quaternion[i] = [np.cos(i/100), np.sin(angle/2), 0, np.cos(angle/2)]  # Quaternion values

    for i in range(len(xvalues)):
        file.write("" + xvalues[i].__str__() + " " + yvalues[i].__str__() + " " + zvalues[i].__str__() + " " + quaternion[i,0].__str__() + " " + quaternion[i,1].__str__() + " " + quaternion[i,2].__str__() + " " + quaternion[i,3].__str__() + "\n")
    file.close()
if __name__ == '__main__':
    main()