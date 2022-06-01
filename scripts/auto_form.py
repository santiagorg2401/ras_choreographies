#!/usr/bin/env python3

import numpy as np
import sys
import os
import rospy

sys.path.append('/home/' + os.getlogin() +
                '/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts')
os.chdir("/home/" + os.getlogin() +
         "/crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts/")

from pycrazyswarm import *

# Collision avoidance (between Crazyflies) parameters.
xy_radius = 0.30
radii = xy_radius * np.array([1.0, 1.0, 3.0])


class sender():
    def __init__(self, path, speed):
        # Constructor of the class sender.
        # Create the needed objects and initialize a ROS Node.
        # I nstance from the class Crazyswarm in pycrazyswarm.crazyswarm
        self.swarm = Crazyswarm()
        # Instance from the class CrazyflieServer in pycrazyswarm.crazyflie
        self.allcfs = self.swarm.allcfs
        # List of objects  from the class Crazyflie in pycrazyswarm.crazyflie, it has as many objects as declared in crazyflies_yaml, in the same order starting from 0.
        self.cfs = self.allcfs.crazyflies
        # Instance from the class timeHelper in pycrazyswarm.crazyflie
        self.timeHelper = self.swarm.timeHelper

        self.path = path
        self.speed = speed

        # Enable collision avoidance between all Crazyflies.
        for i, cf in enumerate(self.cfs):
            others = self.cfs[:i] + self.cfs[(i+1):]
            cf.enableCollisionAvoidance(others, radii)

        # Set solid color effect on all  LED-Decks (see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/ for more parameters).
        self.led(0)

    def publisher(self):
        # Set up parameters.
        data = np.genfromtxt(self.path, delimiter=',')

        # Confirmation.
        print("Press any button to start with the sequence.")
        self.swarm.input.waitUntilButtonPressed()
        self.led(1)

        for i in range(0, len(data)):
            takeOffHeight = data[i][2]
            initialPos = data[:, 0:3][i] - np.array([0, 0, takeOffHeight])

            # Take off.
            self.cfs[i].takeoff(targetHeight=takeOffHeight,
                                duration=takeOffHeight/self.speed)
            self.timeHelper.sleep(takeOffHeight/self.speed*1.2)

            self.cfs[i].goTo(data[:, 0:3][i], 0, 1)
            self.timeHelper.sleep(1.2)

        # Confirmation.
        print("Press any button to finish the sequence.")
        self.swarm.input.waitUntilButtonPressed()

        # Land.
        self.allcfs.land(targetHeight=0.02, duration=(
            takeOffHeight)/speed)
        self.timeHelper.sleep((takeOffHeight)/speed*1.2)

    def calculateTime(self, speed, initialPos, finalPos):
        distance = abs(np.linalg.norm(finalPos - initialPos))
        Time = distance/speed

        return Time

    def led(self, enable):
        # Change colour.
        for cf in self.allcfs.crazyflies:
            cf.setLEDColor(255*enable/255.0, 0/255.0, 0/255.0)
            cf.setParam("ring/headlightEnable", enable)


if __name__ == "__main__":
    try:
        args = rospy.myargv(argv=sys.argv)
        file_path = args[1]
        speed = 0.6

        sen = sender(path=file_path, speed=speed)
        sen.publisher()

    except KeyboardInterrupt:
        print("Emergency cancel registred, landing UAVs.")
        sen.allcfs.land(targetHeight = 0.02, duration = 5)
        sen.timeHelper.sleep(5*1.2)
        sen.allcfs.emergency()
